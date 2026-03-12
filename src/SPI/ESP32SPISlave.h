// ESP32SPISlave v0.3.0 by hideakitai — local copy with MODE3 register fix.
// Original: https://github.com/hideakitai/ESP32SPISlave/tree/v0.3.0
// Modification: post_setup_cb patches SPI registers for correct MODE3 timing.
// See: https://github.com/espressif/esp-idf/issues/7698

#pragma once
#ifndef ESP32SPI_SLAVE_H
#define ESP32SPI_SLAVE_H

#include <Arduino.h>
#include <SPI.h>
#include <driver/spi_slave.h>
#include <soc/spi_struct.h>
#include <deque>

#ifndef ARDUINO_ESP32_SPI_SLAVE_NAMESPACE_BEGIN
#define ARDUINO_ESP32_SPI_SLAVE_NAMESPACE_BEGIN \
    namespace arduino {                         \
    namespace esp32 {                           \
        namespace spi {                         \
            namespace slave {
#endif
#ifndef ARDUINO_ESP32_SPI_SLAVE_NAMESPACE_END
#define ARDUINO_ESP32_SPI_SLAVE_NAMESPACE_END \
    }                                         \
    }                                         \
    }                                         \
    }
#endif

ARDUINO_ESP32_SPI_SLAVE_NAMESPACE_BEGIN

void spi_slave_setup_done(spi_slave_transaction_t* trans);
void spi_slave_trans_done(spi_slave_transaction_t* trans);

class Slave {
    // callbacks
    friend void spi_slave_setup_done(spi_slave_transaction_t* trans);
    friend void spi_slave_trans_done(spi_slave_transaction_t* trans);

    spi_slave_interface_config_t if_cfg {
        .spics_io_num = SS,
        .flags = 0,
        .queue_size = 3,
        .mode = SPI_MODE0,
        .post_setup_cb = spi_slave_setup_done,
        .post_trans_cb = spi_slave_trans_done,
    };

    spi_bus_config_t bus_cfg {
        .mosi_io_num = MOSI,
        .miso_io_num = MISO,
        .sclk_io_num = SCK,
        .quadwp_io_num = 0,
        .quadhd_io_num = 0,
        .data4_io_num = 0,
        .data5_io_num = 0,
        .data6_io_num = 0,
        .data7_io_num = 0,
        .max_transfer_sz = SOC_SPI_MAXIMUM_BUFFER_SIZE,
        .flags = SPICOMMON_BUSFLAG_SLAVE,
        .intr_flags = 0,
    };

    spi_host_device_t host {SPI2_HOST};

    std::deque<spi_slave_transaction_t> transactions;
    std::deque<uint32_t> results;

public:
    // use SPI with the default pin assignment
    bool begin(const uint8_t spi_bus = HSPI) {
#ifdef CONFIG_IDF_TARGET_ESP32
        bus_cfg.mosi_io_num = (spi_bus == VSPI) ? MOSI : 13;
        bus_cfg.miso_io_num = (spi_bus == VSPI) ? MISO : 12;
        bus_cfg.sclk_io_num = (spi_bus == VSPI) ? SCK : 14;
        if_cfg.spics_io_num = (spi_bus == VSPI) ? SS : 15;
#endif
        return initialize(spi_bus);
    }

    // use SPI with your own pin assignment
    bool begin(const uint8_t spi_bus, const int8_t sck, const int8_t miso, const int8_t mosi, const int8_t ss) {
        bus_cfg.mosi_io_num = mosi;
        bus_cfg.miso_io_num = miso;
        bus_cfg.sclk_io_num = sck;
        if_cfg.spics_io_num = ss;
        return initialize(spi_bus);
    }

    bool end() {
        return (spi_slave_free(host) == ESP_OK);
    }

    // wait for transaction one by one
    bool wait(uint8_t* rx_buf, const size_t size) {
        return wait(rx_buf, NULL, size);
    }

    bool wait(uint8_t* rx_buf, const uint8_t* tx_buf, const size_t size) {
        if (!transactions.empty()) {
            printf(
                "[WARN] cannot execute transfer if queued transaction exists. queued transactions = %d\n",
                transactions.size());
            return 0;
        }

        addTransaction(rx_buf, tx_buf, size);

        esp_err_t e = spi_slave_transmit(host, &transactions.back(), portMAX_DELAY);
        if (e != ESP_OK) {
            printf("[ERROR] SPI device transmit failed : %d\n", e);
            transactions.pop_back();
            return 0;
        }

        return (e == ESP_OK);
    }

    // queueing transaction
    // wait (blocking) and timeout occurs if queue is full with transaction
    // (but designed not to queue transaction more than queue_size, so there is no timeout argument)
    bool queue(uint8_t* rx_buf, const size_t size) {
        return queue(rx_buf, NULL, size);
    }
    bool queue(uint8_t* rx_buf, const uint8_t* tx_buf, const size_t size) {
        if (transactions.size() >= if_cfg.queue_size) {
            printf("[WARNING] slave queue is full with transactions. discard new transaction request\n");
            return false;
        }

        addTransaction(rx_buf, tx_buf, size);
        esp_err_t e = spi_slave_queue_trans(host, &transactions.back(), portMAX_DELAY);

        return (e == ESP_OK);
    }

    // wait until all queued transaction will be done by master
    // if yield is finished, all the buffer is updated to latest
    void yield() {
        size_t n = transactions.size();
        for (uint8_t i = 0; i < n; ++i) {
            spi_slave_transaction_t* r_trans;
            esp_err_t e = spi_slave_get_trans_result(host, &r_trans, portMAX_DELAY);
            if (e != ESP_OK) {
                printf("[ERROR] SPI slave get trans result failed %d / %d : %d\n", i, n, e);
            }
        }
    }

    // transaction result info

    // size of completed (received) transaction results
    size_t available() const {
        return results.size();
    }

    // size of queued (not completed) transactions
    size_t remained() const {
        return transactions.size();
    }

    // size of the received bytes of the oldest queued transaction result
    uint32_t size() const {
        return results.front() / 8;
    }

    // pop the oldest transaction result
    void pop() {
        results.pop_front();
    }

    // ===== Main Configurations =====
    // set these optional parameters before begin() if you want

    void setDataMode(const uint8_t m) {
        setSpiMode(m);
    }

    // ===== Optional Configurations =====

    void setSlaveFlags(const uint32_t flags) {
        if_cfg.flags = flags;
    }

    void setQueueSize(const int n) {
        if_cfg.queue_size = n;
    }

    void setSpiMode(const uint8_t m) {
        if_cfg.mode = m;
    }

private:
    spi_host_device_t hostFromBusNumber(uint8_t spi_bus) const {
        switch (spi_bus) {
            case FSPI:
#ifdef CONFIG_IDF_TARGET_ESP32
                return SPI1_HOST;
#else
                return SPI2_HOST;
#endif
            case HSPI:
#if defined(CONFIG_IDF_TARGET_ESP32) || defined(CONFIG_IDF_TARGET_ESP32C3)
                return SPI2_HOST;
#else
                return SPI3_HOST;
#endif
#ifdef CONFIG_IDF_TARGET_ESP32
            case VSPI:
                return SPI3_HOST;
#endif
            default:
                return SPI2_HOST;
        }
    }

    bool initialize(const uint8_t spi_bus) {
        host = hostFromBusNumber(spi_bus);
        esp_err_t e = spi_slave_initialize(host, &bus_cfg, &if_cfg, SPI_DMA_DISABLED);

        return (e == ESP_OK);
    }

    void addTransaction(uint8_t* rx_buf, const uint8_t* tx_buf, const size_t size) {
        transactions.emplace_back(spi_slave_transaction_t());
        transactions.back().length = 8 * size;  // in bit size
        transactions.back().tx_buffer = tx_buf;
        transactions.back().rx_buffer = rx_buf;
        transactions.back().user = (void*)this;
    }

    void pushResult(const uint32_t s) {
        results.push_back(s);
    }
};

// Called by ESP-IDF after SPI registers are configured, before each transaction starts.
// Experiment 10: Try different miso_delay_mode values to fix MISO output timing.
// ESP-IDF slave MODE3 defaults: ck_idle_edge=0, ck_i_edge=0, miso_delay_mode=1
// The slave inverts CPOL perception, so ck_idle_edge=0 is correct for MODE3.
// See https://github.com/espressif/esp-idf/issues/7698
inline void spi_slave_setup_done(spi_slave_transaction_t* trans) {
    // Keep ESP-IDF defaults for ck_idle_edge and ck_i_edge (both 0 for slave MODE3)
    // Only change miso_delay_mode to fix MISO output timing
    SPI3.ctrl2.miso_delay_mode = 2;  // Try 0, 1, 2, 3 — default is 1
}

inline void spi_slave_trans_done(spi_slave_transaction_t* trans) {
    ((Slave*)trans->user)->results.push_back(trans->trans_len);
    ((Slave*)trans->user)->transactions.pop_front();
}

ARDUINO_ESP32_SPI_SLAVE_NAMESPACE_END

using ESP32SPISlave = arduino::esp32::spi::slave::Slave;

#endif  // ESP32SPI_SLAVE_H
