#include "chip.h"

static GPIO_TypeDef* getPortAddress(uint8_t port) {
	uint32_t addr = AHB2PERIPH_BASE + ((uint32_t)port) * 0x400;

	return (GPIO_TypeDef*)addr;
}

void halGpioConfigure(uint8_t port, uint8_t pin, enum Mode mode, enum Pull pull){
	RCC->AHBENR |= 1<<(17+port);
	GPIO_TypeDef* pa = getPortAddress(port);
	uint8_t pinMode = 0;

	/* Alternate functions */
	if(mode <= 7){
		pinMode = 2;

		/* Configure alternate function register */
		if(pin < 8){
			pa->AFR[0] = replaceField32(pa->AFR[0], 4*pin, 4, mode);
		}else{
			pa->AFR[1] = replaceField32(pa->AFR[1], 4*(pin - 8), 4, mode);
		}

	}else{
		pinMode = mode & 0x3;
	}

	/* Write mode register */
	pa->MODER = replaceField32(pa->MODER, 2*pin, 2, pinMode);

	/* Set pullup */
	pa->PUPDR = replaceField32(pa->PUPDR, 2*pin, 2, pull);
}

void halGpioSet(uint8_t port, uint8_t pin, uint8_t value){
	if(value){
		getPortAddress(port)->BSRR = 1<<pin;
	}else{
		getPortAddress(port)->BSRR = 1<<(pin + 16);
	}
}

uint8_t halGpioGet(uint8_t port, uint8_t pin){
	return (getPortAddress(port)->IDR & (1<<pin)) > 0;
}
