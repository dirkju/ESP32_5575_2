#ifndef MYSPI_H
#define MYSPI_H

#include <stdint.h>
#include <string.h>

extern uint32_t transactionNo;

extern const char* registerNames[];
extern uint16_t registerValues[];

void setupSPISlave();
uint16_t MODBUS_CRC16_v3(const unsigned char *buf, unsigned int len);

#endif
