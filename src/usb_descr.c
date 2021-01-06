/*
 * usb_descr.c
 *
 *  Created on: Nov 18, 2020
 *      Author: bertold
 */

#include <stdint.h>
#include "usb_descr.h"
#include "usb_control.h"
#include "chip.h"

#define USBD1_DATA_REQUEST_EP           1
#define USBD1_DATA_AVAILABLE_EP         1
#define USBD1_INTERRUPT_REQUEST_EP      2

/*
 * USB Device Descriptor.
 */
//static const uint8_t vcom_device_descriptor_data[18] = {
//  USB_DESC_DEVICE       (0x0110,        /* bcdUSB (1.1).                    */
//                         0x02,          /* bDeviceClass (CDC).              */
//                         0x00,          /* bDeviceSubClass.                 */
//                         0x00,          /* bDeviceProtocol.                 */
//                         0x40,          /* bMaxPacketSize.                  */
//						 0x0403,        /* idVendor (ST).                   */
//						 0x0000,        /* idProduct.                       */
//                         0x0200,        /* bcdDevice.                       */
//                         1,             /* iManufacturer.                   */
//                         2,             /* iProduct.                        */
//                         3,             /* iSerialNumber.                   */
//                         1)             /* bNumConfigurations.              */
//};


static const uint8_t vcom_device_descriptor_data[18] = {
  USB_DESC_DEVICE       (0x0200,        /* bcdUSB (1.1).                    */
		  	  	  	  	  0xFF,0xFF,0xFF,
                         0x40,          /* bMaxPacketSize.                  */
						 0x1c40,        /* idVendor (ST).                   */
						 0x0534,        /* idProduct.                       */
                         0x0900,        /* bcdDevice.                       */
                         1,             /* iManufacturer.                   */
                         2,             /* iProduct.                        */
                         3,             /* iSerialNumber.                   */
                         1)             /* bNumConfigurations.              */
};

/* Configuration Descriptor tree for a CDC.*/
static const uint8_t vcom_configuration_descriptor_data[32] = {
  /* Configuration Descriptor.*/
  USB_DESC_CONFIGURATION(32,            /* wTotalLength.                    */
                         0x01,          /* bNumInterfaces.                  */
                         0x01,          /* bConfigurationValue.             */
                         0,             /* iConfiguration.                  */
                         0xC0,          /* bmAttributes (self powered).     */
                         50),           /* bMaxPower (100mA).               */
  /* Interface Descriptor.*/
  USB_DESC_INTERFACE    (0x00,          /* bInterfaceNumber.                */
                         0x00,          /* bAlternateSetting.               */
                         0x02,          /* bNumEndpoints.                   */
                         0x0A,          /* bInterfaceClass (Data Class
                                           Interface, CDC section 4.5).     */
                         0x00,          /* bInterfaceSubClass (CDC section
                                           4.6).                            */
                         0x00,          /* bInterfaceProtocol (CDC section
                                           4.7).                            */
                         0x00),         /* iInterface.                      */
  /* Endpoint 3 Descriptor.*/
  USB_DESC_ENDPOINT     (USBD1_DATA_AVAILABLE_EP,       /* bEndpointAddress.*/
                         0x02,          /* bmAttributes (Bulk).             */
                         0x0040,        /* wMaxPacketSize.                  */
                         0x00),         /* bInterval.                       */
  /* Endpoint 1 Descriptor.*/
  USB_DESC_ENDPOINT     (USBD1_DATA_REQUEST_EP|0x80,    /* bEndpointAddress.*/
                         0x02,          /* bmAttributes (Bulk).             */
                         0x0040,        /* wMaxPacketSize.                  */
                         0x00)          /* bInterval.                       */
};

/*
 * Configuration Descriptor wrapper.
 */
/*
 * U.S. English language identifier.
 */
//static const uint8_t vcom_string0[] = {
//  USB_DESC_BYTE(4),                     /* bLength.                         */
//  USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
//  USB_DESC_WORD(0x0409)                 /* wLANGID (U.S. English).          */
//};
//
///*
// * Vendor string.
// */
//static const uint8_t vcom_string1[] = {
//  USB_DESC_BYTE(38),                    /* bLength.                         */
//  USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
//  'S', 0, 'T', 0, 'M', 0, 'i', 0, 'c', 0, 'r', 0, 'o', 0, 'e', 0,
//  'l', 0, 'e', 0, 'c', 0, 't', 0, 'r', 0, 'o', 0, 'n', 0, 'i', 0,
//  'c', 0, 's', 0
//};

/*
 * Device Description string.
 */
static const uint8_t vcom_string2[] = {
  USB_DESC_BYTE(56),                    /* bLength.                         */
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
  'C', 0, 'h', 0, 'i', 0, 'b', 0, 'i', 0, 'O', 0, 'S', 0, '/', 0,
  'R', 0, 'T', 0, ' ', 0, 'V', 0, 'i', 0, 'r', 0, 't', 0, 'u', 0,
  'a', 0, 'l', 0, ' ', 0, 'C', 0, 'O', 0, 'M', 0, ' ', 0, 'P', 0,
  'o', 0, 'r', 0, 't', 0
};

uint8_t dataBuf[100];
void handleControlBuf(USBControlRequest* req, void* param){
	req->data = dataBuf;
	req->dataLen = sizeof(dataBuf);
}

#include <string.h>
uint8_t handleControl(USBControlRequest* req, void* param){

	if(0){
	halUartSend('C');
	halUartHex8(req->bmRequestType);
	halUartSend(' ');
	halUartHex8(req->bRequest);
	halUartSend(' ');
	halUartHex16(req->wValue);
	halUartSend(' ');
	halUartHex16(req->wIndex);
	halUartSend(' ');
	halUartHex16(req->wLength);
	halUartLf();


	for(uint16_t i=0;  i<req->dataLen; i++){
		halUartHex8(req->data[i]);
	}
	halUartLf();
	}

	if (req->bRequest == 6){
		if ((req->wValue>> 8) == 1){
			req->data = (uint8_t*)vcom_device_descriptor_data;
			req->dataLen = sizeof(vcom_device_descriptor_data);
		}else if((req->wValue>> 8) == 2){
			req->data = (uint8_t*)vcom_configuration_descriptor_data;
			req->dataLen = sizeof(vcom_configuration_descriptor_data);
		}else{
			req->data = (uint8_t*)vcom_string2;
			req->dataLen = sizeof(vcom_string2);
		}
	}else{

		req->data = dataBuf;

		memset(req->data, 0, req->wLength);
		if(req->wLength >= 2) req->data[1]=1<<6;
		req->dataLen=req->wLength;
	}


	return 1;
}




//
//const uint8_t devDesc[] = {18, 0x1, 0x00, 0x02, 0xff, 0xff, 0xff, 64, 0x34, 0x12, 0x78, 0x56, 0x00, 0x01, 0, 0, 0, 1};
//const uint8_t ctrlDesc[] = {    /* USB configuration descriptor */
//   9,          /* sizeof(usbDescriptorConfiguration): length of descriptor in bytes */
//   2,    /* descriptor type */
//   18, 0,
//               /* total length of data returned (including inlined descriptors) */
//   1,          /* number of interfaces in this configuration */
//   1,          /* index of this configuration */
//   0,          /* configuration name string index */
//   (1 << 7),       /* attributes */
//
//   500/2,            /* max USB current in 2mA units */
///* interface descriptor follows inline: */
//   9,          /* sizeof(usbDescrInterface): length of descriptor in bytes */
//   0x04, /* descriptor type */
//   0,          /* index of this interface */
//   0,          /* alternate setting for this interface */
//   0, /* endpoints excl 0: number of endpoint descriptors to follow */
//   0xFF,
//   0xFF,
//   0xFF,
//   0,          /* string index for interface */
//};

