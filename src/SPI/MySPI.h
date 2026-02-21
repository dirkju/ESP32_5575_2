#ifndef MYSPI_H
#define MYSPI_H

#include <stdint.h>
#include <string.h>

extern uint32_t transactionNo;
extern volatile bool cs;

extern uint8_t rk1Stell[38];
extern int rkIdx;

extern uint8_t requestDataFromMaster_stage;
extern bool requestDataFromMaster;
extern uint32_t pollDataDelay_ms;
extern int64_t lastDataRequestTime;

extern const char* registerNames[];
extern uint16_t registerValues[];

void setupSPISlave();
uint16_t MODBUS_CRC16_v3(const unsigned char *buf, unsigned int len);

#endif
