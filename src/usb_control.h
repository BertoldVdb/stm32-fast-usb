#ifndef SRC_USB_CONTROL_H_
#define SRC_USB_CONTROL_H_

#include <stdint.h>
#include "usb_hal.h"

typedef struct {
	uint8_t  bmRequestType;
	uint8_t  bRequest;
	uint16_t wValue;
	uint16_t wIndex;
	uint16_t wLength;

	uint8_t* data;
	uint16_t dataLen;
} USBControlRequest;


enum usbControlState {
	CTRL_WAIT_SETUP,
	CTRL_WAIT_DATA_RX,
	CTRL_WAIT_STATUS_RX,
};

typedef void (*USBControlRequestGetRxBuffer)(USBControlRequest* req, void* param);
typedef uint8_t (*USBControlRequestReceived)(USBControlRequest* req, void* param);


typedef struct {
	uint8_t newAddr;
	USBEndpoint* endpoint;

	enum usbControlState ctrlState;

	USBControlRequest req;
	uint32_t reqDataIndex;

	USBControlRequestGetRxBuffer inputBufferCb;
	USBControlRequestReceived reqCb;
	void* reqCbParam;

	USBEndpointBuffer rxBuf;
} USBControlHandler;

void USBControlReset(USBControlHandler* handler, USBController* ctrl);




#endif /* SRC_USB_CONTROL_H_ */
