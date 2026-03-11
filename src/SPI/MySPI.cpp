#include "MySPI.h"

#include <ESP32SPISlave.h>      // https://github.com/hideakitai/ESP32SPISlave


// ########### Public variables (in .h-file, too) #############

ESP32SPISlave slave;
constexpr uint32_t BUFFER_SIZE {32};

static uint8_t spi_tx[2][BUFFER_SIZE];
static uint8_t spi_rx[2][BUFFER_SIZE];

constexpr uint8_t CORE_TASK_SPI_SLAVE {1};

static TaskHandle_t task_handle_spi = 0;

uint32_t transactionNo = 0;

// ########### Private variables #############

#define pin_inv_SS     25   // Trovis CS input (active-HIGH)
#define pin_inv_SS_OUT 26   // Inverted CS output → wired to GPIO5 (ESP32 SPI CS, active-LOW)

// ########### ISR: CS inversion #############

IRAM_ATTR void slave_signal_from_master() {
    if (digitalRead(pin_inv_SS)) {
        digitalWrite(pin_inv_SS_OUT, LOW);  // Trovis CS high → ESP32 CS low (select)
    } else {
        digitalWrite(pin_inv_SS_OUT, HIGH); // Trovis CS low  → ESP32 CS high (idle)
    }
}

// ########### Protocol constants #############
//
// Trovis 5575 SPI protocol (discovered via reverse-engineering + Benvorth's work):
//
//  PASSIVE mode (MISO = 0x00):
//    Trovis sees 0xFF on MISO = "no command".
//    Transaction 1 (4B):  Trovis polls:      MOSI = 30 00 00 00
//    Transaction 2 (2B):  Trovis pushes:     MOSI = 57 <stellsignal>
//
//  COMMAND mode (MISO = inverted Modbus frame):
//    Same CS cycle carries command (bytes 0-13 on MISO) AND response (bytes 14+ on MOSI).
//    Trovis sees 0x06 header -> extends transaction to ~28-32 bytes.
//    MISO bytes 0-13:  inverted 14-byte Modbus request  (0xFF - cmd_byte)
//    MISO bytes 14-31: 0xFF (Trovis sees 0x00 = idle padding)
//    MOSI bytes 0-13:  30 00 00 00 ... (Trovis poll padding, ignore)
//    MOSI bytes 14+:   Modbus response (NOT inverted — read directly)
//    Register value:   rx_buf[19] (high) and rx_buf[20] (low)
//
//  Command frame structure (non-inverted, 14 bytes):
//    [0]     0x06        header
//    [1]     0x0C        length (12 bytes follow, excl. header+terminator)
//    [2-4]   00 FA 08    fixed preamble
//    [5]     0xFF        device selector
//    [6]     0x03        Modbus function code (read holding registers)
//    [7-8]   reg_hi/lo   register address (variable)
//    [9-10]  00 01       register count = 1
//    [11-12] crc_hi/lo   CRC-16 Modbus over bytes [5..10]
//    [13]    0x55        terminator
//
//  On MISO, each byte is inverted: miso_byte = 0xFF - cmd_byte

constexpr uint8_t CMD_LEN = 14;

// Registers to read (from Benvorth's register map)
static const uint16_t registersToRead[] = {
    0x0000, // Geraetekennung (device type, should be 5575)
    0x0009, // Aussenfuehler AF1 (outside temperature)
    0x000C, // Vorlauftemperatur VF1 (flow temperature)
    0x0010, // Ruecklauftemperatur RueF1 (return temperature)
    0x0016, // Speichertemperatur SF1 (storage temperature)
    0x001C, // WMZ Eingang Imp/h
    0x006A, // Stellsignal Rk1 [0..100%]
    0x03E7, // Vorlaufsollwert Rk1
};
const char* registerNames[] = {
    "Geraetekennung",
    "Aussenfuehler_AF1",
    "Vorlauftemp_VF1",
    "Ruecklauftemp_RueF1",
    "Speichertemp_SF1",
    "WMZ_Imp_h",
    "Stellsignal_Rk1",
    "Vorlaufsollwert_Rk1",
};
constexpr int NUM_REGISTERS = sizeof(registersToRead) / sizeof(registersToRead[0]);
uint16_t registerValues[NUM_REGISTERS + 1] = {}; // +1 for coils slot (MySPI.h compat)

// ########### Command helpers #############

// CRC input template (bytes [5]..[10] of the command frame, reg addr filled in at call time)
static uint8_t crcInput[6] = { 0xFF, 0x03, 0x00, 0x00, 0x00, 0x01 };

// Build the inverted 14-byte command for reading one register and store in buf[0..31].
// Bytes 14-31 are set to 0xFF (Trovis sees 0x00 = idle padding).
static void buildReadCmd(uint16_t regAddr, uint8_t* buf) {
    uint8_t cmd[CMD_LEN] = {
        0x06, 0x0C,
        0x00, 0xFA, 0x08,   // fixed preamble
        0xFF,               // device selector
        0x03,               // function: read holding registers
        (uint8_t)(regAddr >> 8), (uint8_t)(regAddr & 0xFF),
        0x00, 0x01,         // count = 1
        0x00, 0x00,         // CRC placeholder
        0x55                // terminator
    };

    // CRC over bytes [5..10]: {0xFF, 0x03, reg_hi, reg_lo, 0x00, 0x01}
    crcInput[2] = cmd[7];
    crcInput[3] = cmd[8];
    uint16_t crc = MODBUS_CRC16_v3(crcInput, sizeof(crcInput));
    cmd[11] = (uint8_t)(crc >> 8);
    cmd[12] = (uint8_t)(crc & 0xFF);

    // Fill buffer: 0xFF after command (Trovis sees 0x00 = idle padding)
    memset(buf, 0xFF, BUFFER_SIZE);
    for (uint8_t i = 0; i < CMD_LEN; i++) {
        buf[i] = 0xFF - cmd[i]; // invert
    }
}

// ########### Main SPI task #############

void spi_slave_main_task(void* pvParameters) {
    // Pattern: build+queue → wait → re-arm immediately → then log.
    // This ensures MISO is pre-loaded before Trovis fires the next CS,
    // regardless of how long Serial output takes.

    uint8_t  log_rx[BUFFER_SIZE];
    size_t   log_rxLen;
    uint32_t log_txNo;

    // State sequence (matches Benvorth's stage progression):
    //   ST_PASSIVE: MISO = all 0xFF  ("slave present" warm-up, Benvorth stage 0)
    //   ST_ANN:     MISO[0] = 0xF9   (announce "ready to send command", Benvorth stage 1)
    //   ST_CMD:     MISO = inverted Modbus CMD  (Benvorth stages 2..N+1)
    //   → back to ST_PASSIVE
    enum State { ST_PASSIVE, ST_ANN, ST_CMD } queued = ST_PASSIVE;
    int queued_reg = 0;

    // Pre-arm passive before first CS.
    memset(spi_tx[0], 0xFF, BUFFER_SIZE);
    slave.queue(spi_tx[0], spi_rx[0], BUFFER_SIZE);

    while (1) {
        // Wait for the queued transaction to complete.
        auto results = slave.wait();
        log_rxLen = results.size() > 0 ? results[0] : 0;
        memcpy(log_rx, spi_rx[0], BUFFER_SIZE);
        log_txNo = transactionNo++;

        State just_completed = queued;
        int   just_reg       = queued_reg;

        // Re-arm for next transaction immediately (before any Serial output).
        if (just_completed == ST_PASSIVE) {
            queued = ST_ANN;
            memset(spi_tx[0], 0xFF, BUFFER_SIZE);
            spi_tx[0][0] = 0xF9;
            slave.queue(spi_tx[0], spi_rx[0], BUFFER_SIZE);
        } else if (just_completed == ST_ANN) {
            queued = ST_CMD;
            queued_reg = 0;
            buildReadCmd(registersToRead[0], spi_tx[0]);
            slave.queue(spi_tx[0], spi_rx[0], BUFFER_SIZE);
        } else if (just_reg < NUM_REGISTERS - 1) {
            queued = ST_CMD;
            queued_reg = just_reg + 1;
            buildReadCmd(registersToRead[queued_reg], spi_tx[0]);
            slave.queue(spi_tx[0], spi_rx[0], BUFFER_SIZE);
        } else {
            // Last CMD done, back to passive.
            queued = ST_PASSIVE;
            queued_reg = 0;
            memset(spi_tx[0], 0xFF, BUFFER_SIZE);
            slave.queue(spi_tx[0], spi_rx[0], BUFFER_SIZE);
        }

        // Log completed transaction (MISO already re-armed above).
        if (just_completed == ST_PASSIVE) {
            Serial.printf("[%5lu] PSV  MOSI: %02X %02X %02X %02X\n",
                log_txNo, log_rx[0], log_rx[1], log_rx[2], log_rx[3]);
        } else if (just_completed == ST_ANN) {
            Serial.printf("[%5lu] ANN  MOSI: %02X %02X %02X %02X\n",
                log_txNo, log_rx[0], log_rx[1], log_rx[2], log_rx[3]);
        } else {
            Serial.printf("[%5lu] CMD reg=0x%04X (%s)  rxLen=%u  MOSI: ",
                log_txNo, registersToRead[just_reg], registerNames[just_reg], log_rxLen);
            for (size_t i = 0; i < log_rxLen && i < BUFFER_SIZE; i++)
                Serial.printf("%02X ", log_rx[i]);
            Serial.println();

            if (log_rxLen >= CMD_LEN + 7u) {
                uint16_t val = ((uint16_t)log_rx[CMD_LEN + 5] << 8) | log_rx[CMD_LEN + 6];
                registerValues[just_reg] = val;
                Serial.printf("         => %s = 0x%04X (%d)\n",
                    registerNames[just_reg], val, val);
            } else {
                Serial.printf("         => too short (%u bytes, need >=%u)\n",
                    log_rxLen, CMD_LEN + 7u);
            }
        }
    }
}


void setupSPISlave() {
    // VSPI pins: CS=5, CLK=18, MOSI=23, MISO=19
    // Trovis CS is active-HIGH (GPIO25). ISR inverts it to active-LOW on GPIO26 → GPIO5.
    // 52µs gap between CS↑ and first SCK↓ gives ISR enough time to invert before first bit.
    pinMode(pin_inv_SS, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(pin_inv_SS), slave_signal_from_master, CHANGE);
    pinMode(pin_inv_SS_OUT, OUTPUT);
    digitalWrite(pin_inv_SS_OUT, HIGH); // Idle = HIGH (CS deasserted)

    slave.setDataMode(SPI_MODE3); // CPOL=1, CPHA=1: clock idles HIGH, sample on rising edge.
                                  // Trovis drives MOSI on falling SCK edge (confirmed by PulseView).
    slave.setQueueSize(1);

    memset(spi_tx, 0x00, sizeof(spi_tx));

    slave.begin(VSPI, SCK, MISO, MOSI, SS);

    xTaskCreatePinnedToCore(
        spi_slave_main_task,
        "spi_slave_main",
        4096,
        NULL,
        2,
        &task_handle_spi,
        CORE_TASK_SPI_SLAVE
    );

    Serial.println("SPI Slave ready. Passive+command mode.");
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
