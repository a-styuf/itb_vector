#ifndef __CRC16_H
#define __CRC16_H

# include <string.h>

typedef unsigned int uint32_t;
typedef int int32_t;
typedef unsigned short uint16_t;
typedef short int16_t;
typedef unsigned char uint8_t;
typedef signed char int8_t;

uint16_t crc16_ccitt(uint8_t *buf, uint8_t len);

#endif
