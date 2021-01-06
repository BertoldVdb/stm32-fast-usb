#include <stdlib.h>

#include "usb_hal.h"
#include "usb_hal_stm32.h"
#include "chip.h"
#include "usb_control.h"


//static volatile uint16_t* bufferRam = (uint16_t*)0x40006000;
//static uint16_t  bufferRamIndex = 64;
//static uint16_t  bufferRamSize = 1024;
//
//
//
//struct endpoint {
//	uint8_t 		  flags;
//	enum endpointType type;
//	uint16_t		  bufSize[2];
//	uint8_t 		  addr;
//
//	void (*txDoneCb)(void*);
//	void  *txDoneParam;
//} endpoints[8];
//
//
//
//static void endpointBufferAllocCore(uint8_t ep, uint8_t slot, uint8_t size){
//	if (size & 1) abort();
//
//    uint16_t numBlocks;
//	uint16_t blockSize = 0;
//
//	if ((size & 0x1F) == 0) {
//		blockSize = 1;
//		numBlocks = (size >> 5) - 1;
//        } else {
//		numBlocks = size >> 1;
//	}
//
//	uint16_t count = (blockSize << 15) | (numBlocks << 10);
//
//	bufferRam[4*ep + 2*slot + 0] = bufferRamIndex;
//	bufferRam[4*ep + 2*slot + 1] = count;
//
//	bufferRamIndex += size;
//	if (bufferRamIndex > bufferRamSize) abort();
//}
//
//static void endpointBufferAlloc(uint8_t ep){
//	for(uint8_t i=0; i<2; i++){
//		if (endpoints[ep].bufSize[i]) endpointBufferAllocCore(ep, i, endpoints[ep].bufSize[i]);
//	}
//}
//
//
//static volatile uint16_t* endpointBufferGetAddress(uint8_t ep, uint8_t slot){
//	uint16_t start = bufferRam[4*ep + 2*slot + 0];
//	return bufferRam + (start>>1);
//}
//
//static struct endpointBuffer endpointBufferGet(uint8_t ep, uint8_t slot, uint8_t send){
//	struct endpointBuffer result;
//	result.data = (uint8_t*)endpointBufferGetAddress(ep, slot);
//
//	if (send){
//		result.len = endpoints[ep].bufSize[slot];
//	} else {
//		result.len = bufferRam[4*ep + 2*slot + 1] & 0x1FF;
//    }
//
//	return result;
//}
//
//static volatile uint16_t* endpointRegister(uint8_t ep){
//	return ((volatile uint16_t*)USB) + 2*ep;
//}
//
//void endpointStateUpdate(uint8_t ep, enum endpointState wantedRx, enum endpointState wantedTx){
//	volatile uint16_t* reg = endpointRegister(ep);
//
//	uint16_t base = endpoints[ep].addr;
//	base |= ((uint16_t)endpoints[ep].type) <<  9;
//
//	uint16_t current = *reg;
//	if (wantedRx != USB_EPSTATE_UNCHANGED){
//		uint16_t toggle = (current & USB_EPRX_STAT) >> 12;
//		toggle ^= wantedRx;
//		base |= toggle << 12;
//	}
//	if (wantedTx != USB_EPSTATE_UNCHANGED){
//		uint16_t toggle = (current & USB_EPTX_STAT) >> 4;
//		toggle ^= wantedTx;
//		base |= toggle << 4;
//	}
//
//	*reg=base;
//}
//
//static void setupControlEndpoint(){
//	endpoints[0].flags = 1;
//	endpoints[0].type = USB_EPTYPE_CONTROL;
//	endpoints[0].bufSize[0] = 64;
//	endpoints[0].bufSize[1] = 64;
//	endpoints[0].addr = 0;
//	endpointBufferAlloc(0);
//	endpointStateUpdate(0, 0x3, USB_EPSTATE_UNCHANGED);
//}
//
//
//void usbReset(){
//	bufferRamIndex = 64;
//	setupControlEndpoint();
//
//	/* Enable */
//	USB->DADDR = 1<<7;
//}
//
//struct endpointBuffer endpointGetTransmitBuffer(uint8_t ep){
//	return endpointBufferGet(ep, 0, 1);
//}
//
//void endpointTransmit(uint8_t ep, struct endpointBuffer txBuf, void (*txDoneCb)(void*), void* param){
//	endpoints[ep].txDoneCb = txDoneCb;
//	endpoints[ep].txDoneParam = param;
//	bufferRam[4*ep + 1] = txBuf.len;
//	debugPrintBuffer(txBuf);
//
//	volatile uint16_t* pktBuf = endpointBufferGetAddress(ep, 0);
//	uint16_t work;
//	for(uint8_t i=0; i<txBuf.len; i++){
//		work >>= 8;
//		work |= ((uint16_t)txBuf.data[i]) << 8;
//		pktBuf[i>>1]=work;
//	}
//
//	endpointStateUpdate(ep, USB_EPSTATE_UNCHANGED, USB_EPSTATE_VALID);
//}
//
//
//void usbHandleTransfer(uint8_t ep, uint8_t fromHost){
//	uint16_t epState = *endpointRegister(ep);
//
//	if (epState & USB_EP_CTR_RX){
//		uint8_t isSetup = (epState & USB_EP_SETUP) > 0;
//		endpointStateUpdate(ep, USB_EPSTATE_UNCHANGED, USB_EPSTATE_UNCHANGED);
//
//		struct endpointBuffer buffer = endpointBufferGet(ep, 1, 0);
//
//		if (endpoints[ep].type == USB_EPTYPE_CONTROL){
//			usbControlHandlePacket(ep, isSetup, buffer);
//		}
//
//
////		endpointStateUpdate(ep, 0x3, 0xFF);
//	}
//
//	if (epState & USB_EP_CTR_TX){
//		endpointStateUpdate(ep, USB_EPSTATE_UNCHANGED, USB_EPSTATE_UNCHANGED);
//
//		//endpointStateUpdate(ep, 0xFF, 0x2);
//
//		void (*cb)(void*) = endpoints[ep].txDoneCb;
//		if (cb){
//			endpoints[ep].txDoneCb = NULL;
//			cb(endpoints[ep].txDoneParam);
//		}
//	}
//
//
//	// *endpointRegister(0) |=
//
//}

USBController stm32usb;

 void USBHandler(){
	 USBControllerIRQ(&stm32usb);
	// void USBControllerIRQ(USBController* ctrl, uint32_t flags);
	 /*halUartSend('I');
	 halUartHex16(USB->ISTR);
	 halUartLf();*/

 }
 uint8_t handleControl(USBControlRequest* req, void* param);
 void handleControlBuf(USBControlRequest* req, void* param);

 USBControlHandler usbcontrol;
#include <string.h>
uint8_t txData[65] = {0,0,1<<6,'a','b',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ','\r','\n'};

volatile uint32_t canTx = 0;
volatile uint32_t canRx = 0;
volatile 	uint8_t work=10;
void testTx(uint16_t len, void* ep){
	canTx=1;

//	halUartHex8(canTx);


}

USBEndpointBuffer testBuf[6];
USBEndpoint* eptx;

 void testRx(USBEndpointBuffer* buf, void* ep){
	 if(!buf){
		 return;
	 }

	 canRx++;
	 //work=0;



	 return;

 }
	int rxIndex = 0;


 void resetHandler(USBController* ctrl){
	 USBControlReset(&usbcontrol, ctrl);
	 rxIndex=0;
	 eptx = USBControllerEndpointAlloc(ctrl, 1, USBEP_TYPE_BULK, 64, 6, 64, 6);
	// USBEndpoint* ep = USBControllerEndpointAlloc(ctrl, 1, USBEP_TYPE_BULK, 64, 0, 64, 3);

	 if(!eptx){
		 return;
	 }

	 for(int i=0; i<6; i++){
		 testBuf[i].cb = testRx;
		 testBuf[i].param = eptx;
	 }

	 for(int i=0; i<6; i++){
		 //USBEndpointReceive(eptx, &testBuf[i]);

		 //halUartHex32((uint32_t)&testBuf[i]);
		// halUartLf();

	 }
	 canRx = 6;
	 canTx=1;

 }

int main(){
	usbcontrol.reqCb=handleControl;
	usbcontrol.inputBufferCb=handleControlBuf;

	halGpioConfigure(0, 5, MODE_OUTPUT, PULL_NONE);
	halGpioConfigure(0, 2, MODE_AF1, PULL_NONE);
	halGpioConfigure(0, 3, MODE_AF1, PULL_NONE);

	halUartInit(2, 0x34);

	RCC->APB1ENR |= RCC_APB1ENR_USBEN;
	RCC->APB1ENR |= RCC_APB1ENR_CRSEN;

	NVIC_EnableIRQ(31);

	CRS->CR|=1<<6;
	CRS->CR|=1<<5;


	USBControllerInit(&stm32usb);
	halDelay(200);
	USBControllerStart(&stm32usb, resetHandler);

//	endpointPrintBuffer(0,1,0);}
	int i = 0;
	while(1){
		//continue;
		__disable_irq();
		if(canRx){
			canRx--;
			__enable_irq();
			//halUartHex32(canRx);
			//halUartHex32((uint32_t)&testBuf[rxIndex]);
			halUartHex8((canRx<<4)|rxIndex);
			halUartLf();
			USBEndpointReceive(eptx, &testBuf[rxIndex]);
			rxIndex++;
			if(rxIndex==6){
				rxIndex = 0;
			}
		}else{
			__enable_irq();
		}
		if(canTx&&(work<1000)){
		//	halUartHex8(work);
			//halUartLf();

		 USBEndpointBuffer tx = {};
		 tx.len = sizeof(txData)-1;
		 tx.data = (uint8_t*)txData+1;

		 memset(txData+4,' ', 11);
		 txData[4+work]='x';
		 work++;
		 if(work >= 10){
			 work = 0;
			 //while(1);
		 }
		 __disable_irq();
		 canTx=0;
		__enable_irq();
		 USBEndpointTransmit(eptx, &tx, testTx, eptx);


		 if(i>10000){
	//	 halUartHexNibble(eptx->txBufNumActive);
		// halUartLf();
		 }else{
			 i++;
		 }


		}else{
//				halDelay(100);
		//	asm("wfi");
		//	 halUartHex16(CRS->CR);
			// halUartSend(' ');
			 //halUartHex16(CRS->ISR);
			continue;
			 if(i>10000){
				 CRS->ICR = 0xF;
				 i=0;

					//if(ep->type ==USBEP_TYPE_BULK){
						halUartSend('S');
						halUartHex16(USB->EP0R);
						halUartSend(' ');
						halUartHex16(USB->EP1R);
						halUartLf();
					//}
			 }else{
				 i++;
			 }
		//	 halUartLf();

		}

		 continue;
		halGpioSet(0, 5, 1);
		halDelay(500);
		halGpioSet(0, 5, 0);
		halDelay(500);
		//halUartSend('b');
	//halUartHex16(USB->ISTR);
	//halUartLf();
	}

	return 0;
}
