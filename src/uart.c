#include "chip.h"

static USART_TypeDef* getUartAddress(uint8_t index){
	return USART2;
}

void halUartInit(uint8_t index, uint16_t baud){
	USART_TypeDef* usart = getUartAddress(index);

	if(index == 2){
		RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
	}

	usart->CR1 = 0;
	usart->BRR = baud;
	usart->CR3 = USART_CR3_OVRDIS;
	usart->CR2 = 0;
	usart->CR1 = USART_CR1_UE;
	usart->CR1 |= USART_CR1_TE | USART_CR1_RE;
}

void halUartSend(uint8_t b){
	USART_TypeDef* usart = getUartAddress(2);

	usart->TDR = b;
    while(!(usart->ISR & USART_ISR_TXE)){
    }
}

void halUartHexNibble(uint8_t b){
	if(b >= 10){
		halUartSend('A'+b-10);
		return;
	}
	halUartSend('0'+b);
}

void halUartHex8(uint8_t b){
	halUartHexNibble(b>>4);
	halUartHexNibble(b&0xF);
}

void halUartHex16(uint16_t b){
	halUartHex8(b>>8);
	halUartHex8(b&0xFF);
}

void halUartHex32(uint32_t b){
	halUartHex16(b>>16);
	halUartHex16(b&0xFFFF);
}

void halUartLf(){
	halUartSend('\r');
	halUartSend('\n');
}
