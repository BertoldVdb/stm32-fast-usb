/*
 * usb.h
 *
 *  Created on: Nov 12, 2020
 *      Author: bertold
 */

#include <stdint.h>

#ifndef SRC_USB_HAL_H_
#define SRC_USB_HAL_H_

typedef struct usbController USBController;
typedef struct usbEndpoint USBEndpoint;

typedef enum {
	USBEP_FLAGS_IS_SETUP = 1,			/* This flag is set if the buffer contains a setup packet */
	USBEP_FLAGS_NATIVE_ACCESS = 2,      /* This flag indicates the buffer can be accessed safely with any word width */
	USBEP_FLAGS_IS_SAFE_CONCURRENT = 4, /* This flag indicates the buffer will still be valid after a new TX/RX request */
} USBEndpointBufferFlags;

typedef struct usbEndpointBuffer USBEndpointBuffer;
typedef void(*USBBufferCompleteCallback)(USBEndpointBuffer* buf, void* param);

typedef struct usbEndpointBuffer {
	USBEndpointBufferFlags  flags;
	uint8_t* data;
	uint16_t len;

	USBBufferCompleteCallback cb;
	void* param;
} USBEndpointBuffer;

typedef enum {
	USBEP_TYPE_BULK = 0,
	USBEP_TYPE_CONTROL = 1,
	USBEP_TYPE_ISO = 2,
	USBEP_TYPE_INTERRUPT = 3,
	USBEP_TYPE_INVALID = 0xFF
} USBEndpointType;

typedef enum {
	USBEP_STATE_INVALID = 0,
	USBEP_STATE_STALL = 1,
	USBEP_STATE_NAK = 2,
	USBEP_STATE_VALID = 3,

	USBEP_STATE_UNCHANGED
} USBEndpointState;

typedef void(*USBResetCallback)(USBController* ctrl);

int USBControllerInit(USBController* ctrl);
void USBControllerStart(USBController* ctrl, USBResetCallback resetCb);
void USBControllerStop(USBController* ctrl);

void USBControllerSetAddress(USBController* ctrl, uint8_t addr);


USBEndpoint* USBControllerEndpointAlloc(USBController* ctrl, uint8_t addr, USBEndpointType type, uint16_t txBufSize, uint8_t txNumBuffer, uint16_t rxBufSize, uint8_t rxNumBuffer);

typedef void(*USBReceiveCallback)(uint16_t len, void* param);
typedef void(*USBTransmitCallback)(uint16_t len, void* param);

USBController* USBEndpointGetController(USBEndpoint* ep);

uint8_t USBEndpointReceiveSetState(USBEndpoint* ep, USBEndpointState newState);
uint8_t USBEndpointReceive(USBEndpoint*    ep, USBEndpointBuffer* buf);
uint8_t USBEndpointReceiveGetData(USBEndpoint*  ep, USBEndpointBuffer* buf);


uint8_t USBEndpointTransmitSetState(USBEndpoint* ep, USBEndpointState newState);
uint8_t USBEndpointTransmitGetNativeBuffer(USBEndpoint*  ep, USBEndpointBuffer* buf);
uint8_t USBEndpointTransmit(USBEndpoint* ep, USBEndpointBuffer* buf, USBTransmitCallback cb, void*param);



#endif /* SRC_USB_HAL_H_ */
