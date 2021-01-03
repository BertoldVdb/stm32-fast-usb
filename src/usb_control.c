/*
 * usb_control.c
 *
 *  Created on: Nov 11, 2020
 *      Author: bertold
 */


#include "usb_hal.h"
#include <stdlib.h>
#include "chip.h"
#include <string.h>

#include "usb_control.h"

static void usbControlHandlePacket(USBEndpointBuffer* buf, void* param);
static void usbControlReceive(USBControlHandler  *handler, enum usbControlState newState){
	handler->ctrlState = newState;
	USBEndpointReceive(handler->endpoint, &handler->rxBuf);
}

static void usbControlStatusTX(uint16_t len, void* param){
	USBControlHandler * handler = (USBControlHandler  *)param;

	if(!len){
		if(handler->newAddr){
			USBControllerSetAddress(USBEndpointGetController(handler->endpoint), handler->newAddr);
			handler->newAddr = 0;
		}
		usbControlReceive(handler, CTRL_WAIT_SETUP);
		return;
	}

	USBEndpointBuffer txBuf = {};
	USBEndpointTransmit(handler->endpoint, &txBuf, usbControlStatusTX, handler);
}

static void usbControlDataTX(uint16_t len, void* param){
	USBControlHandler  * handler = (USBControlHandler  *)param;

	uint32_t maxPktSize = 64;
	/* If last transmission was not max packet size, we are done, receive ACK */
	if(len < maxPktSize){
		usbControlReceive(handler, CTRL_WAIT_STATUS_RX);
		return;
	}

	uint32_t txLen = handler->req.dataLen - handler->reqDataIndex;
	if(txLen > maxPktSize){
		txLen = maxPktSize;
	}

	USBEndpointBuffer txBuf = {};
	txBuf.len = txLen;
	txBuf.data = (uint8_t*)(handler->req.data + handler->reqDataIndex);
	handler->reqDataIndex += txLen;

	USBEndpointTransmit(handler->endpoint, &txBuf, usbControlDataTX, handler);
}

static void usbControlCommandFailed(USBControlHandler  * handler){
	USBEndpointTransmitSetState(handler->endpoint, USBEP_STATE_STALL);
	usbControlReceive(handler, CTRL_WAIT_SETUP);
}

static void usbControlHandleCommandInput(USBControlHandler  * handler){
	handler->req.dataLen = 0;

	if (handler->inputBufferCb){
		handler->inputBufferCb(&handler->req, handler->reqCbParam);
	}
}


static uint8_t usbControlHandleCommand(USBControlHandler  * handler){
	if (handler->req.bmRequestType == 0 && handler->req.bRequest == 5){
		handler->newAddr = handler->req.wValue;
		return 1;
	}

	if (handler->reqCb){
		return handler->reqCb(&handler->req, handler->reqCbParam);
	}

	return 0;
}

static void usbControlHandlePacket(USBEndpointBuffer* buf, void* param){
	if (!buf){
		return;
	}

	USBControlHandler  * handler = (USBControlHandler  *)param;

	uint8_t isSetup = (buf->flags & USBEP_FLAGS_IS_SETUP) > 0;
	if (isSetup){
		handler->ctrlState = CTRL_WAIT_SETUP;
	}

	switch (handler->ctrlState){
	case CTRL_WAIT_SETUP:
		if (!isSetup || buf->len != 8){
			break;
		}

		handler->req.bmRequestType = buf->data[0];
		handler->req.bRequest      = buf->data[1];
		handler->req.wValue        = readLE16(&buf->data[2]);
		handler->req.wIndex        = readLE16(&buf->data[4]);
		handler->req.wLength       = readLE16(&buf->data[6]);
		handler->req.dataLen       = 0;

		uint8_t dataStageHostReceives = handler->req.bmRequestType >> 7;
		uint8_t dataStagePresent      = handler->req.wLength > 0;

		handler->reqDataIndex = 0;

		if(dataStagePresent && !dataStageHostReceives){
			usbControlHandleCommandInput(handler);

			usbControlReceive(handler, CTRL_WAIT_DATA_RX);
			return;
		}

		if(!usbControlHandleCommand(handler)){
			usbControlCommandFailed(handler);
			return;
		}

		if(!dataStagePresent){
			usbControlStatusTX(1, handler);
			return;
		}

		if(handler->req.dataLen > handler->req.wLength){
			handler->req.dataLen = handler->req.wLength;
		}

		usbControlDataTX(0xFFFF, handler);
		return;

	case CTRL_WAIT_DATA_RX:
		if((handler->reqDataIndex + (uint32_t)buf->len) <= handler->req.dataLen){
			memcpy(handler->req.data + handler->reqDataIndex, buf->data, buf->len);
		}
		handler->reqDataIndex += buf->len;

		if ((buf->len == 64) && (handler->reqDataIndex < handler->req.wLength)){
			usbControlReceive(handler, CTRL_WAIT_DATA_RX);
			return;
		}

		if (handler->reqDataIndex > handler->req.dataLen){
			usbControlCommandFailed(handler);
			return;
		}

		handler->req.dataLen = handler->reqDataIndex;

		if (!usbControlHandleCommand(handler)){
			usbControlCommandFailed(handler);
			return;
		}

		usbControlStatusTX(1, handler);
		return;

	default:
		break;
	}

	usbControlReceive(handler, CTRL_WAIT_SETUP);
}

void USBControlReset(USBControlHandler* handler, USBController* ctrl){
	USBEndpoint* endpoint = USBControllerEndpointAlloc(ctrl, 0, USBEP_TYPE_CONTROL, 64, 1, 64, 1);

	handler->newAddr = 0;
	handler->endpoint = endpoint;
	handler->rxBuf.cb = usbControlHandlePacket;
	handler->rxBuf.param = handler;
	usbControlReceive(handler, CTRL_WAIT_SETUP);
}

