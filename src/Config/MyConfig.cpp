#include "MyConfig.h"
#include <Arduino.h>

char chipUUID[23];

void setupConfig() {
    uint64_t chipid = ESP.getEfuseMac();
    uint16_t chip = (uint16_t)(chipid >> 32);
    snprintf(chipUUID, 23, "DEVICE-%04X-%08X", chip, (uint32_t)chipid);
    Serial.print("Chip UUID: ");
    Serial.println(chipUUID);
}
