#include "src/Config/MyConfig.h"
#include "src/SPI/MySPI.h"

void setup() {
    Serial.begin(115200);
    while (!Serial)
        ; // wait for serial attach

    Serial.println("\n=== ESP32 Trovis 5575 Test Setup ===");
    Serial.printf("Build: %s %s\n", __DATE__, __TIME__);

    setupConfig();
    setupSPISlave();
}

void loop() {
    // Poll trigger and all SPI logic runs in spi_slave_main_task (MySPI.cpp)
    vTaskDelay(portMAX_DELAY);
}
