/*
 * debug.c
 *
 *  Created on: Nov 11, 2020
 *      Author: bertold
 */


#include "usb_hal.h"
#include "chip.h"

void debugPrintBuffer(USBEndpointBuffer buffer){

	for(uint16_t i=0; i<buffer.len; i++){
		halUartHex8(buffer.data[i]);
		halUartSend(' ');
		if ((i%16)==15){
			halUartLf();
		}
	}
	halUartLf();
}

uint16_t readLE16(uint8_t* buf){
	return ((uint16_t)buf[0]) | (((uint16_t)buf[1])<<8);
}
