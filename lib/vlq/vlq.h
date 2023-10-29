#ifndef VLQ_H
#define VLQ_H

#include <stdint.h>

struct VarInt {
	uint32_t value;
	uint8_t length;
};

VarInt parse_vlq_int(uint8_t* bytes, uint8_t length);
uint8_t encode_vlq_int(uint8_t *p, uint32_t v);

#endif