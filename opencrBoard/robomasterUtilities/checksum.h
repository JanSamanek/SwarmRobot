#ifndef CRC_H
#define CRC_H

#include <stdint.h>

void setCRC8(uint8_t *msg, uint32_t crc_index);
void appendCRC16CheckSum(uint8_t *msg, uint32_t msglen);

#endif
