#include "chip.h"
#include <stddef.h>

#ifndef _WITH_LIB
__attribute__((used)) void *memset(void *s, int c, size_t n){
    for(size_t i = 0; i<n; i++){
        ((char*)s)[i] = c;
    }
    return s;
}

__attribute__((used)) void *memcpy(void *dest, const void *src, size_t n){
    for(size_t i = 0; i<n; i++){
        ((char*)dest)[i] = ((char*)src)[i];
    }
    return dest;
}
#endif

uint8_t equal(const void *s1, const void *s2, size_t n){
    for(size_t i = 0; i<n; i++){
        if(((char*)s1)[i] != ((char*)s2)[i]){
            return 0;
        }
    }

    return 1;
}

uint8_t safeCompare(uint8_t* buf1, uint8_t* buf2, uint8_t len){
	volatile uint8_t sum = 0;

	for(uint8_t i=0; i<len; i++){
		sum |= buf1[i] ^ buf2[i];
	}

	return sum > 0;
}

uint32_t replaceField32(uint32_t in, unsigned int offset, unsigned int length, unsigned int value){
	uint32_t mask = ((1<<length)-1) << offset;
	in &= ~mask;

	in |= (value << offset) & mask;
	return in;
}

void encodeBe32(uint8_t* buf, uint32_t val){
	buf[0] = val >> 24;
	buf[1] = val >> 16;
	buf[2] = val >> 8;
	buf[3] = val >> 0;
}

uint32_t decodeBe32(uint8_t* buf){
	uint32_t out = 0;

	out |= buf[0] << 24;
	out |= buf[1] << 16;
	out |= buf[2] << 8;
	out |= buf[3] << 0;

	return out;
}
