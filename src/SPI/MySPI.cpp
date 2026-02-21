#include "MySPI.h"

#include <ESP32SPISlave.h>      // https://github.com/hideakitai/ESP32SPISlave


// ########### Public variables (in .h-file, too) #############

ESP32SPISlave slave;
constexpr uint32_t BUFFER_SIZE {32};
uint8_t spi_slave_tx_buf[BUFFER_SIZE];
uint8_t spi_slave_rx_buf[BUFFER_SIZE];

constexpr uint8_t CORE_TASK_SPI_SLAVE {1};

static TaskHandle_t task_handle_spi = 0;

uint32_t transactionNo = 0;
volatile bool cs_last = false;
volatile bool cs = false;

uint8_t rk1Stell [38];
int rkIdx = 0;

bool requestDataFromMaster = false;
uint32_t pollDataDelay_ms = 5000;
int64_t lastDataRequestTime = 0;

// Stage counter:
// 0 - first handshake transaction (tx=0xFF, tell master slave is present)
// 1 - send slaveReadyCommand (0xF9)
// 2..N+1 - send register read commands, receive results
// N+2 - send coil read command, receive result
// N+3 - all done, print summary and reset
uint8_t requestDataFromMaster_stage = 0;

// ########### Private variables #############

#define pin_inv_SS 25
#define pin_inv_SS_OUT 26

// Slave answers this on MISO so master sees that slave wants to send commands
uint8_t slaveRadyCommand[] = {
    0xF9
};

uint8_t readARegister_cmd [] = {
    0x06, 0x0C, 0x00, 0xFA, 0x08, 0xFF,
    0x03, // FunctionID 3 = read
    0xFF, 0xFF, // start register No (replaced by revolveReadRegisterCommand())
    0x00, 0x01, // number of registers to read (= 1)
    0xFF, 0xFF, // CRC checksum (replaced by revolveReadRegisterCommand())
    0x55
};
int NUM_READAREGISTER_CMD = sizeof(readARegister_cmd);

uint16_t registersToRead[] = {
    0x0000, // Gerätekennung (5575)
    0x0009, // AF1
    0x000C, // Vorlauftemp VF1
    0x0010, // RueckltempRueF1
    0x0016, // SpeichertempSF1
    0x001C, // Meßwert [Imp/h] am Eingang WMZ
    0x006A, // StellsignalRk1 [0...100%] (CL90)
    0x03E7  // VorlSollwRk1 (CL116)
};
int NUM_REGISTERS_TO_READ = (sizeof(registersToRead) / sizeof(uint16_t));

const char* registerNames[] = {
    "Geraetekennung",
    "Aussenfuehler_1_AF1",
    "Vorlauftemperatur_VF1",
    "Rueckltemperatur_RueF1",
    "Speichertemperatur_SF1",
    "Eingang_WMZ_Imp_h",
    "Stellsignal_Rk1",
    "Vorlaufsollwert_Rk1",
    "Coils"
};

uint16_t registerValues[] = {
    0x1FFF,
    0x1FFF,
    0x1FFF,
    0x1FFF,
    0x1FFF,
    0x1FFF,
    0x1FFF,
    0x1FFF,
    0x1FFF // coils
};

// Coils 56-71
uint8_t readCoils[] = {
    0x06, 0x0B, 0xFA, 0x08, 0xFF, 0x01, 0x00, 0x38, 0x00, 0x0F, 0xE8, 0x1D, 0x55
};

uint8_t readCommand_crc[] =
    {0xFF, 0x03, 0x00, 0x00, 0x00, 0x01}
;

// ########### Serial output helpers #############

static void printSPITransaction(uint32_t txnNo, uint8_t stage) {
    Serial.printf("[TXN #%lu | stage %d]\n", txnNo, stage);
    Serial.print("  TX (MISO):");
    for (uint32_t i = 0; i < BUFFER_SIZE; i++) {
        Serial.printf(" %02X", spi_slave_tx_buf[i]);
    }
    Serial.println();
    Serial.print("  RX (MOSI):");
    for (uint32_t i = 0; i < BUFFER_SIZE; i++) {
        Serial.printf(" %02X", spi_slave_rx_buf[i]);
    }
    Serial.println();
}

static void printAllRegisters() {
    Serial.println("===== TROVIS 5575 DATA =====");
    Serial.printf("  %-30s: %u  (0x%04X)\n",
        registerNames[0], (unsigned int)registerValues[0], registerValues[0]);
    Serial.printf("  %-30s: %.1f C  (0x%04X)\n",
        registerNames[1], (signed int)registerValues[1] / 10.0f, registerValues[1]);
    for (int i = 2; i <= 4; i++) {
        Serial.printf("  %-30s: %.1f C  (0x%04X)\n",
            registerNames[i], (unsigned int)registerValues[i] / 10.0f, registerValues[i]);
    }
    Serial.printf("  %-30s: %u  (0x%04X)\n",
        registerNames[5], (unsigned int)registerValues[5], registerValues[5]);
    Serial.printf("  %-30s: %u %%  (0x%04X)\n",
        registerNames[6], (unsigned int)registerValues[6], registerValues[6]);
    Serial.printf("  %-30s: %.1f C  (0x%04X)\n",
        registerNames[7], (unsigned int)registerValues[7] / 10.0f, registerValues[7]);
    uint16_t coils = registerValues[8];
    Serial.printf("  %-30s: (0x%04X)\n", registerNames[8], coils);
    Serial.printf("    C56 Umwaelzpumpe Rk1         : %d\n", (coils >> 15) & 1);
    Serial.printf("    C57 Umwaelzpumpe Rk2         : %d\n", (coils >> 14) & 1);
    Serial.printf("    C58                          : %d\n", (coils >> 13) & 1);
    Serial.printf("    C59 Speicherladepumpe TW     : %d\n", (coils >> 12) & 1);
    Serial.printf("    C60 Zirkulationspumpe        : %d\n", (coils >> 11) & 1);
    Serial.printf("    C61 Rk1 3-Pkt Zu-Signal      : %d\n", (coils >> 10) & 1);
    Serial.printf("    C62 Rk1 3-Pkt Auf-Signal     : %d\n", (coils >>  9) & 1);
    Serial.printf("    C63 Rk2 3-Pkt Zu-Signal      : %d\n", (coils >>  8) & 1);
    Serial.printf("    C64 Rk2 3-Pkt Auf-Signal     : %d\n", (coils >>  7) & 1);
    Serial.printf("    C65                          : %d\n", (coils >>  6) & 1);
    Serial.printf("    C66                          : %d\n", (coils >>  5) & 1);
    Serial.printf("    C67 Pumpenmanag. UP1 Ein/Aus : %d\n", (coils >>  4) & 1);
    Serial.printf("    C68 Pumpenmanag. Drehzahl UP1: %d\n", (coils >>  3) & 1);
    Serial.printf("    C69                          : %d\n", (coils >>  2) & 1);
    Serial.printf("    C70                          : %d\n", (coils >>  1) & 1);
    Serial.printf("    C71                          : %d\n", (coils >>  0) & 1);
    Serial.println("============================");
}

// ########### SPI helpers #############

void set_tx_buffer(uint8_t value) {
    for (uint32_t i = 0; i < BUFFER_SIZE; i++) {
        spi_slave_tx_buf[i] = value;
    }
}

void set_tx_buffer(uint8_t* command, uint32_t size) {
    memset(spi_slave_tx_buf, 0xFF, BUFFER_SIZE);
    for (uint32_t i = 0; i < size; i++) {
        spi_slave_tx_buf[i] = command[i];
    }
}

// ISR: inverts the Trovis CS signal (idle-low, active-high) onto G26 → G5
IRAM_ATTR void slave_signal_from_master() {
    cs = !cs;
    if (!cs) {
        digitalWrite(pin_inv_SS_OUT, HIGH); // Idle
    } else {
        digitalWrite(pin_inv_SS_OUT, LOW);  // Select
    }
}

void revolveReadRegisterCommand() {
    uint16_t registerToRead = registersToRead[requestDataFromMaster_stage - 2];

    uint8_t p1 = (uint8_t)((registerToRead >> 8) & 0xFF);
    uint8_t p2 = (uint8_t)(registerToRead & 0xFF);

    readARegister_cmd[7] = p1;
    readARegister_cmd[8] = p2;
    readCommand_crc[2] = p1;
    readCommand_crc[3] = p2;

    uint16_t crc = MODBUS_CRC16_v3(readCommand_crc, sizeof(readCommand_crc));
    readARegister_cmd[11] = (uint8_t)((crc >> 8) & 0xFF);
    readARegister_cmd[12] = (uint8_t)(crc & 0xFF);

    uint8_t read_cmd_inv[NUM_READAREGISTER_CMD];
    for (uint8_t k = 0; k < NUM_READAREGISTER_CMD; k++) {
        read_cmd_inv[k] = 0xFF - readARegister_cmd[k];
    }
    set_tx_buffer(read_cmd_inv, sizeof(read_cmd_inv));
}

void revolveReadCoilCommand() {
    uint8_t read_cmd_inv[sizeof(readCoils)];
    for (uint8_t k = 0; k < sizeof(readCoils); k++) {
        read_cmd_inv[k] = 0xFF - readCoils[k];
    }
    set_tx_buffer(read_cmd_inv, sizeof(read_cmd_inv));
}

// ########### Main SPI task #############

// Single task: queue TX, block until master completes transaction, process RX, repeat.
void spi_slave_main_task(void* pvParameters) {
    while (1) {
        // Poll trigger: start a new Modbus read cycle every pollDataDelay_ms
        if (!requestDataFromMaster && (millis() - lastDataRequestTime) > pollDataDelay_ms) {
            requestDataFromMaster_stage = 0;
            requestDataFromMaster = true;
            Serial.printf("\n--- Begin poll cycle #%lu at %lu ms ---\n", transactionNo, (uint32_t)millis());
        }

        // Prepare current TX buffer and wait for master to clock in a transaction
        slave.queue(spi_slave_tx_buf, spi_slave_rx_buf, BUFFER_SIZE);
        slave.wait(); // blocks until master completes the transaction

        // Print raw bytes for every transaction
        printSPITransaction(transactionNo, requestDataFromMaster_stage);

        if (requestDataFromMaster) {

            // --- Process received data ---
            if (requestDataFromMaster_stage <= 1) {
                Serial.printf("  [stage %d] Handshake\n", requestDataFromMaster_stage);

            } else if (requestDataFromMaster_stage < (NUM_REGISTERS_TO_READ + 2)) {
                registerValues[requestDataFromMaster_stage - 2] =
                    ((uint16_t)spi_slave_rx_buf[5 + NUM_READAREGISTER_CMD] << 8) |
                    spi_slave_rx_buf[6 + NUM_READAREGISTER_CMD];
                Serial.printf("  [stage %d] %-26s [0x%04X] = 0x%04X\n",
                    requestDataFromMaster_stage,
                    registerNames[requestDataFromMaster_stage - 2],
                    registersToRead[requestDataFromMaster_stage - 2],
                    registerValues[requestDataFromMaster_stage - 2]);

                if (requestDataFromMaster_stage - 2 == 6) { // Stellsignal Rk1
                    rk1Stell[rkIdx] = spi_slave_rx_buf[6 + NUM_READAREGISTER_CMD];
                    if (++rkIdx > 37) rkIdx = 0;
                }

            } else if (requestDataFromMaster_stage == (NUM_REGISTERS_TO_READ + 2)) {
                registerValues[requestDataFromMaster_stage - 2] =
                    ((uint16_t)spi_slave_rx_buf[5 + NUM_READAREGISTER_CMD] << 8) |
                    spi_slave_rx_buf[6 + NUM_READAREGISTER_CMD];
                Serial.printf("  [stage %d] Coils = 0x%04X\n",
                    requestDataFromMaster_stage,
                    registerValues[requestDataFromMaster_stage - 2]);

            } else {
                Serial.printf("  [stage %d] THIS SHOULD NOT HAPPEN\n", requestDataFromMaster_stage);
            }

            // --- Prepare TX for next transaction ---
            requestDataFromMaster_stage++;

            if (requestDataFromMaster_stage == 1) {
                set_tx_buffer(slaveRadyCommand, sizeof(slaveRadyCommand));
            } else if (requestDataFromMaster_stage < (NUM_REGISTERS_TO_READ + 2)) {
                revolveReadRegisterCommand();
            } else if (requestDataFromMaster_stage == (NUM_REGISTERS_TO_READ + 2)) {
                revolveReadCoilCommand();
            } else {
                // All data collected
                printAllRegisters();
                requestDataFromMaster_stage = 0;
                lastDataRequestTime = millis();
                requestDataFromMaster = false;
                set_tx_buffer(0x00);
            }

        } else {
            set_tx_buffer(0x00); // idle: tell master clock can relax
        }

        transactionNo++;
    }
}


void setupSPISlave() {
    // VSPI pins: CS=5, CLK=18, MOSI=23, MISO=19
    // Trovis CS is inverted: G25 (input) → ISR inverts → G26 (output) → G5 (ESP32 SPI CS)
    pinMode(pin_inv_SS, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(pin_inv_SS), slave_signal_from_master, CHANGE);

    pinMode(pin_inv_SS_OUT, OUTPUT);
    digitalWrite(pin_inv_SS_OUT, HIGH); // Idle = HIGH

    slave.setDataMode(SPI_MODE3); // Trovis uses CPOL=1, CPHA=1
    slave.setQueueSize(1);

    set_tx_buffer(0x00);

    slave.begin(VSPI, SCK, MISO, MOSI, SS);

    xTaskCreatePinnedToCore(
        spi_slave_main_task,    // task function
        "spi_slave_main",       // name
        4096,                   // stack size (words) — larger for Serial.printf
        NULL,                   // parameter
        2,                      // priority
        &task_handle_spi,       // handle
        CORE_TASK_SPI_SLAVE     // core
    );

    lastDataRequestTime = millis();

    Serial.println("SPI Slave ready. Waiting for poll trigger...");
    Serial.printf("Poll interval: %lu ms\n", pollDataDelay_ms);
}


// https://github.com/LacobusVentura/MODBUS-CRC16
uint16_t MODBUS_CRC16_v3(const unsigned char *buf, unsigned int len) {
	const uint16_t table[256] = {
	0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
	0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
	0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
	0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
	0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
	0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
	0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
	0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
	0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
	0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
	0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
	0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
	0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
	0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
	0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
	0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
	0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
	0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
	0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
	0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
	0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
	0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
	0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
	0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
	0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
	0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
	0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
	0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
	0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
	0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
	0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
	0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040 };

	uint8_t xor_ = 0;
	uint16_t crc = 0xFFFF;

	while (len--) {
		xor_ = (*buf++) ^ crc;
		crc >>= 8;
		crc ^= table[xor_];
	}

    uint16_t swapped = (crc >> 8) | (crc << 8);
	return swapped;
}
