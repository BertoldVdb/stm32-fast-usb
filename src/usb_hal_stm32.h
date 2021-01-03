/*
 * usb_stm32.h
 *
 *  Created on: Nov 12, 2020
 *      Author: bertold
 */

#ifndef SRC_USB_HAL_STM32_H_
#define SRC_USB_HAL_STM32_H_

#include "usb_hal.h"
#include "stm32f0xx.h"

#define CONFIG_USB_HAL_STM32_MAX_BUFFERS 6
#define CONFIG_USB_HAL_STM32_MAX_ENDPOINTS 2

typedef struct usbEndpoint{
	USBController* ctrl;

	uint8_t 		     index;
	USBEndpointType      type;
	uint8_t 		     addr;

	uint16_t 		     epRegBase;
	/*
	 * The controller supports hardware double buffering. This is
	 * not used as you still need to switch the buffer in the IRQ, so you
	 * can instead put another address, and use the endpoint bidirectionally.
	 */
	uint8_t 			 trxBufNum[2];
	uint16_t			 trxBufAddr[2][CONFIG_USB_HAL_STM32_MAX_BUFFERS];
	uint8_t	 			 trxBufIndex[2];

	uint8_t 	 		 txBufWrIndex;
	uint16_t    		 txBufLen[CONFIG_USB_HAL_STM32_MAX_BUFFERS];
	volatile uint8_t     txBufNumActive;

	uint8_t			 	 rxBufRdIndex;
	volatile uint8_t     rxBufNumActive;
	USBEndpointBuffer*   rxBuf[CONFIG_USB_HAL_STM32_MAX_BUFFERS];

//	volatile uint16_t*   trxBufUpdatePointer;
//	uint16_t             trxBufUpdateValue[2];
//	uint32_t             trxBufUpdateFastPath[2];
//	uint16_t		     txBufNextLen;

	USBTransmitCallback txCb;
	void* txCbParam;
} USBEndpoint;

typedef struct usbController {
	/* This data is used by the code in the interrupt handler to update the buffer ASAP */
	uint32_t trxFastPathArray[CONFIG_USB_HAL_STM32_MAX_ENDPOINTS*4];

	USB_TypeDef* usbDevice;

	volatile uint16_t* bufferRam;// = (uint16_t*)0x40006000;
	uint16_t  bufferRamIndex;// = 64;
	uint16_t  bufferRamSize;// = 1024;

	uint8_t endpointIndex;
	USBEndpoint endpoints[CONFIG_USB_HAL_STM32_MAX_ENDPOINTS];

	USBResetCallback resetCb;
} USBController;

void USBControllerIRQ(USBController* ctrl);

#endif /* SRC_USB_HAL_STM32_H_ */
