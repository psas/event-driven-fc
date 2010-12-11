#ifndef BINARY_H
#define BINARY_H

#include <stdint.h>

/* For constant values of `bits`, GCC can unroll these loops into the
 * conventional straight-line unpacking we'd usually hand-code. */

#define readbe(type,bits) static type read##bits##be(const uint8_t *buf) \
{ \
	type ret = 0; \
	for(unsigned i = 0; i < bits / 8; ++i) \
		ret |= (type) buf[i] << (bits - 8 - i * 8); \
	return ret; \
}

#define readle(type,bits) static type read##bits##le(const uint8_t *buf) \
{ \
	type ret = 0; \
	for(unsigned i = 0; i < bits / 8; ++i) \
		ret |= (type) buf[i] << (i * 8); \
	return ret; \
}

readbe(uint16_t, 16)

readle(uint16_t, 16)
readle(uint32_t, 32)
readle(uint64_t, 48)

#endif /* BINARY_H */
