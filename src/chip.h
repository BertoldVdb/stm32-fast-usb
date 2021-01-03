#ifndef _SYSTEM_H
#define _SYSTEM_H

#include "stm32f0xx.h"
#include "usb_hal.h"
#include <stdint.h>

void halInitSystem(void);
void halInitTick(void);
uint32_t halGetTime(void);
void halDelay(uint32_t ms);
int main(void);
void halUartInit(uint8_t index, uint16_t baud);
void halUartSend(uint8_t b);
void halUartHex8(uint8_t b);
void halUartHexNibble(uint8_t b);
void halUartHex32(uint32_t b);

void halUartHex16(uint16_t b);
void halUartLf();
__attribute__((noreturn)) void halHalt();

enum Mode {
	MODE_ANALOG = 0x83,
	MODE_OUTPUT = 0x81,
	MODE_INPUT = 0x80,
	MODE_AF0 = 0,
	MODE_AF1 = 1,
	MODE_AF2 = 2,
	MODE_AF3 = 3,
	MODE_AF4 = 4,
	MODE_AF5 = 5,
	MODE_AF6 = 6,
	MODE_AF7 = 7,
};

enum Pull {
	PULL_NONE = 0,
	PULL_UP = 1,
	PULL_DOWN = 2
};

//void debugPrintBuffer(struct endpointBuffer buffer);
uint16_t readLE16(uint8_t* buf);


void halGpioConfigure(uint8_t port, uint8_t pin, enum Mode mode, enum Pull pull);
void halGpioSet(uint8_t port, uint8_t pin, uint8_t value);
uint8_t halGpioGet(uint8_t port, uint8_t pin);

uint32_t replaceField32(uint32_t in, unsigned int offset, unsigned int length, unsigned int value);

#endif
