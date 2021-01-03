/*
 * usb_stm32.c
 *
 *  Created on: Nov 12, 2020
 *      Author: bertold
 */

#include "usb_hal_stm32.h"

#include <stdlib.h>
#include <string.h>
#include "usb_hal.h"
#include "chip.h"

	/* This data is used by the assembler code in the interrupt handler to update the buffer ASAP
	 * 0: EP config ram addr (buftx, lentx, bufrx, lenrx)
	 * 1: EP reg ptr
	 * 2: 32 bit value that goes to buftx (bottom 16), lentx (top 16) -> 0 if no fastpath
	 * 3: EP reg update value for txack
	 * 4: value that goes to bufrx
	 * 5: EP reg update value for rxack */

#define USB_FASTPATH_EPCONFIGPTR 0
#define USB_FASTPATH_TX 1
#define USB_FASTPATH_RXTXACK 2
#define USB_FASTPATH_RX 3

static volatile uint16_t* endpointRegister(USBEndpoint* ep){
	return ((volatile uint16_t*)USB) + 2*ep->index;
}

static void usbAbort(){
	while(1);
}

static void endpointBufferAlloc(USBEndpoint* ep, uint8_t direction, uint16_t size){
	if (!size) return;
	if (size & 1) usbAbort();

    uint16_t numBlocks;
	uint16_t blockSize = 0;

	if ((size & 0x1F) == 0) {
		blockSize = 1;
		numBlocks = (size >> 5) - 1;
    } else {
		numBlocks = size >> 1;
	}

	uint16_t count = (blockSize << 15) | (numBlocks << 10);
	ep->ctrl->bufferRam[4*ep->index + 2*direction + 1] = count;

	for(uint8_t i=0; i<ep->trxBufNum[direction];i++){
		ep->trxBufAddr[direction][i] = ep->ctrl->bufferRamIndex;
		ep->ctrl->bufferRamIndex += size;
		if (ep->ctrl->bufferRamIndex > ep->ctrl->bufferRamSize) usbAbort();
	}

	ep->ctrl->bufferRam[4*ep->index + 2*direction + 0] = ep->trxBufAddr[direction][0];
}

static uint8_t endpointBufferIndexIncrement(USBEndpoint* ep, uint8_t direction){
	uint8_t tmp = ep->trxBufIndex[direction] + 1;
	if(tmp >= ep->trxBufNum[direction]){
		return 0;
	}
	return tmp;
}

static void endpointFastPathConfigureNextBuffer(USBEndpoint* ep, uint8_t direction, uint8_t doMore){
	uint32_t* fp = ep->ctrl->trxFastPathArray + 4*ep->index;

	uint8_t nextIndex = endpointBufferIndexIncrement(ep, direction);
	uint32_t newBuf = ep->trxBufAddr[direction][nextIndex];



	if(direction == 0){
		uint32_t newLen = ep->txBufLen[nextIndex];
		fp[USB_FASTPATH_TX] = newBuf | (newLen << 16);
		uint32_t txAck = ep->epRegBase & ~USB_EP_CTR_TX; /* Ack TX */

		if(doMore){
			txAck |= (1<<4); //Start a new transfer: NAK->VALID
		}

		fp[USB_FASTPATH_RXTXACK] = (fp[USB_FASTPATH_RXTXACK] & ~0xFFFF) | txAck;
	}else{
		fp[USB_FASTPATH_RX] = newBuf;
		uint32_t rxAck = ep->epRegBase & ~USB_EP_CTR_RX; /* Ack RX */

		if(doMore){
			rxAck |= (1<<12); //Start a new transfer: NAK->VALID
		}

		fp[USB_FASTPATH_RXTXACK] = (fp[USB_FASTPATH_RXTXACK] & 0xFFFF) | (rxAck << 16);
	}
}

//static void endpointBufferUpdatePrepare(USBEndpoint* ep, uint8_t direction){
//	uint8_t newIndex = endpointBufferIndexIncrement(ep, direction);
//	uint32_t newBuf = ep->trxBufAddr[direction][newIndex];
//
//	if(direction == 0){
//		uint32_t* fp = ep->ctrl->trxFastPathArray + 8*ep->index;
//		fp[USB_FASTPATH_TXACK] = ep->epRegBase | USB_EP_CTR_RX; /* Ack tx */
//
//		if(ep->txBufNumActive){
//			uint32_t newLen = ep->txBufLen[ep->trxBufIndex[0]];
//			fp[USB_FASTPATH_TX] = newBuf | (newLen << 16);
//			fp[USB_FASTPATH_TXACK] |= (1<<4); //Start a new transfer: NAK->VALID
//		}
//	}
//}


static void endpointBufferSetTXLen(USBEndpoint* ep, uint16_t len){
	ep->ctrl->bufferRam[4*ep->index+1] = len;
}

/*static void endpointBufferUpdate(USBEndpoint* ep, uint8_t direction){
	ep->trxBufIndex[direction]++;
	if(ep->trxBufIndex[direction] == ep->trxBufNum[direction]){
		ep->trxBufIndex[direction] = 0;
	}

	ep->ctrl->bufferRam[4*ep->index + 2*direction + 0] =
			ep->trxBufAddr[direction][ep->trxBufIndex[direction]];
}*/

static volatile uint16_t* endpointBufferGetAddress(USBEndpoint* ep, uint8_t direction, int8_t bufIndex){
	if(bufIndex < 0){
		bufIndex = ep->trxBufIndex[direction];
	}
	uint16_t start = ep->trxBufAddr[direction][bufIndex];
	return ep->ctrl->bufferRam + (start>>1);
}

//static uint16_t endpointBufferGetLen(USBEndpoint* ep, uint8_t direction){
//	return ep->ctrl->bufferRam[4*ep->index + 2*direction + 1] & 0x1FF;
//}


static void endpointStateSet(USBEndpoint* ep, USBEndpointState wantedRx, USBEndpointState wantedTx){
	volatile uint16_t* reg = endpointRegister(ep);

	uint16_t base = ep->epRegBase;

	uint16_t current = *reg;
	if (wantedRx != USBEP_STATE_UNCHANGED){
		uint16_t toggle = (current & USB_EPRX_STAT) >> 12;
		toggle ^= wantedRx;
		base |= toggle << 12;
	}
	if (wantedTx != USBEP_STATE_UNCHANGED){
		uint16_t toggle = (current & USB_EPTX_STAT) >> 4;
		toggle ^= wantedTx;
		base |= toggle << 4;
	}

	*reg=base;
}

static void endpointResetWork(USBEndpoint* ep){
	ep->type = USBEP_TYPE_INVALID;

	uint16_t epReg = *endpointRegister(ep);
	*endpointRegister(ep) = epReg;

	if (ep->txCb){
		ep->txCb(0, ep->txCbParam);
		ep->txCb = NULL;
	}

	for(uint8_t i=0; i<CONFIG_USB_HAL_STM32_MAX_BUFFERS; i++){
		if(ep->rxBuf[i]){
			if(ep->rxBuf[i]->cb){
				ep->rxBuf[i]->cb(NULL, ep->rxBuf[i]->param);
			}
			ep->rxBuf[i] = NULL;
		}
	}
}

static void endpointReset(USBController* ctrl, uint8_t makeNew){
	for(uint8_t i=0; i<ctrl->endpointIndex; i++){
		endpointResetWork(&ctrl->endpoints[i]);
	}

	ctrl->bufferRamIndex = 64;
	ctrl->endpointIndex = 0;

	if (makeNew && ctrl->resetCb){
		ctrl->resetCb(ctrl);
	}
}

static void usbReset(USBController* ctrl){
	endpointReset(ctrl, 1);

	USB->DADDR = 1<<7;
}

static void usbHandleTransferTX(USBEndpoint* ep){
	uint16_t len = ep->txBufLen[ep->trxBufIndex[0]];
	ep->trxBufIndex[0] = endpointBufferIndexIncrement(ep, 0);

	ep->txBufNumActive--;
	endpointFastPathConfigureNextBuffer(ep, 0, ep->txBufNumActive>=2);

	USBTransmitCallback txCb = ep->txCb;
	if(txCb){
		ep->txCb = NULL;
		txCb(len, ep->txCbParam);
	}
}
static void usbHandleTransferRX(USBEndpoint* ep, uint32_t epState){
	USBEndpointBuffer* buf = ep->rxBuf[ep->trxBufIndex[1]];
	ep->rxBuf[ep->trxBufIndex[1]] = NULL;

	buf->len = (epState >> 16) & 0x3FF;
	epState &= 0xFFFF;

	buf->flags = 0;

	ep->trxBufIndex[1] = endpointBufferIndexIncrement(ep, 1);

	ep->rxBufNumActive--;

	endpointFastPathConfigureNextBuffer(ep, 1, ep->rxBufNumActive>=2);

	if(buf->cb){
		//if (ep->trxBufNum[1] > 1){
		//	buffer.flags |= USBEP_FLAGS_IS_SAFE_CONCURRENT;
		//}

		if ((ep->type == USBEP_TYPE_CONTROL) && (epState & USB_EP_SETUP)){
			buf->flags |= USBEP_FLAGS_IS_SETUP;
		}

		buf->cb(buf, buf->param);
	}
}

int USBControllerInit(USBController* ctrl){
	memset(ctrl, 0, sizeof(*ctrl));
	ctrl->bufferRam = (uint16_t*)0x40006000;
	ctrl->bufferRamSize = 1024;
	ctrl->usbDevice = USB;

	USBControllerStop(ctrl);

	return 0;
}

void USBControllerStart(USBController* ctrl, USBResetCallback resetCb){
	ctrl->resetCb = resetCb;

	USB->BTABLE = 0;
	USB->CNTR = USB_CNTR_CTRM | USB_CNTR_RESETM;

	/* Plug in cable */
	USB->BCDR = 1<<15;
}

void USBControllerStop(USBController* ctrl){
	endpointReset(ctrl, 0);

	USB->BCDR = 0;
	USB->DADDR = 0;
	USB->CNTR = 3;
}
/* 0: EP config ram addr (buftx, lentx, bufrx, lenrx)
* 1: 32 bit value that goes to buftx (bottom 16), lentx (top 16) -> 0 if no fastpath
* 2: EP reg update value for txack/rxack
* 3: value that goes to bufrx
*/
void USBControllerIRQ(USBController* ctrl){
	uint16_t isr = ctrl->usbDevice->ISTR;

	//ASM version of fastpath code
	uint32_t fpAddr, epIndex, work1, work2, work3 = (uint32_t)ctrl->usbDevice, epState;
	asm volatile (
		//Testing CTR can be skipped, since we check the epState
		"mov %[epIndex], %[isr]\n"

	//	"mov %[work1], %[isr]\n" //work1=epState
	//	"lsr %[work1], #16\n" //work1>>=16 (CTR_RX goes to carry)
	//	"bcc done%=\n" //if c==0 -> done

		"lsl %[epIndex], #29\n" // Shift rightmost 3 bits to 31-30-29
		"lsr %[epIndex], #27\n" // Shift last 3 bits to 4-3-2
		"add %[work3], %[epIndex]\n" //Calculate epreg address
		"ldr %[epState], [%[work3], #0]\n" //Read epreg->epState

		"lsl %[epIndex], #2\n" // Shift last 3 bits to 6-5-4
		"mov %[fpAddr], %[fp]\n"
		"add %[fpAddr], %[epIndex]\n"

		//Check if we need to do RX (do this one first as a failure requires retransmission of 64 bytes)
		"mov %[work1], %[epState]\n" //work1=epState
		"lsr %[work1], #16\n" //work1>>=16 (CTR_RX goes to carry)
		"bcc checkTx%=\n" //if c==0 -> checkTx

		//RX Fastpath
		"ldr %[work1], [%[fpAddr], #0]\n"  //USB_FASTPATH_EPCONFIGPTR -> work1
		"ldr %[work2], [%[fpAddr], #12]\n" //USB_FASTPATH_RX -> work2
		"strh %[work2], [%[work1], #4]\n"  //work2 -> config[2]
		"ldrh %[work2], [%[work1], #6]\n"  //config[3] -> work2
		//"str %[work2], [%[fpAddr], #24]\n"
		"lsl %[work2], #16\n"              //work2 -> 16 high bits of epState
		"orr %[epState], %[work2] \n"
		"ldr %[work2], [%[fpAddr], #8]\n" //USB_FASTPATH_RXTXACK (high 16 bits)
		"lsr %[work2], #16\n"
		"strh %[work2], [%[work3], #0]\n"  // -> work3 (epreg)

		//Check if we need to do TX
		"checkTx%=:\n"
		"mov %[work1], #0x80\n" //work1=0x80 (CTR_TX)
		"tst %[epState], %[work1]\n" //v=epState & 0x80 (CTR_TX)
		"beq done%=\n" //if v==0 -> done

		//TX Fastpath
		"ldr %[work1], [%[fpAddr], #0]\n"  //USB_FASTPATH_EPCONFIGPTR -> work1
		"ldr %[work2], [%[fpAddr], #4]\n"  //USB_FASTPATH_TX -> work2
		"strh %[work2], [%[work1], #0]\n"
		"lsr %[work2], #16\n"
		"strh %[work2], [%[work1], #2]\n"
		"ldr %[work2], [%[fpAddr], #8]\n" //USB_FASTPATH_RXTXACK (low 16 bits)
		"strh %[work2], [%[work3], #0]\n"  // -> work3 (epreg)

		"done%=:\n"
		: [fpAddr]"=&l"(fpAddr), [epIndex]"=&l"(epIndex), [work1]"=&l"(work1), [work2]"=&l"(work2), [work3]"+&l"(work3), [epState]"=&l"(epState)
		: [isr]"l"(isr), [fp]"l"(ctrl->trxFastPathArray)
		: "memory");

	//ctrl->usbDevice->ISTR = ~isr;

	//if (isr & USB_ISTR_CTR){
	//	halUartHex16(isr);
		epIndex = isr&7;
		USBEndpoint* ep = &ctrl->endpoints[epIndex];

		if(epState & USB_EP_CTR_RX){
			usbHandleTransferRX(ep, epState);
		}
		if(epState & USB_EP_CTR_TX){
			usbHandleTransferTX(ep);
		}
	//}

	if (isr & USB_ISTR_RESET){
		ctrl->usbDevice->ISTR &= ~USB_ISTR_RESET;
		usbReset(ctrl);
	}
}

void USBControllerSetAddress(USBController* ctrl, uint8_t addr){
	USB->DADDR = addr | (1<<7);
}

USBEndpoint* USBControllerEndpointAlloc(USBController* ctrl,
										uint8_t addr,
										USBEndpointType type,
										uint16_t txBufSize,
										uint8_t txNumBuffer,
										uint16_t rxBufSize,
										uint8_t rxNumBuffer){

	if (ctrl->endpointIndex == CONFIG_USB_HAL_STM32_MAX_ENDPOINTS){
		return NULL;
	}

	USBEndpoint* ep = &ctrl->endpoints[ctrl->endpointIndex];
	memset(ep, 0, sizeof(*ep));

	ep->index = ctrl->endpointIndex;
	ep->ctrl = ctrl;
	ep->addr = addr;
	ep->trxBufNum[0]   = txNumBuffer;
	ep->trxBufNum[1]   = rxNumBuffer;
	ep->type = type;

	uint16_t base = ep->addr;
	base |= ((uint16_t)ep->type) <<  9;
	base |= USB_EP_CTR_RX | USB_EP_CTR_TX;

	ep->epRegBase = base;

	/* Cache this to handle the IRQ faster */
	uint32_t* fp = ep->ctrl->trxFastPathArray + 4*ep->index;
	fp[USB_FASTPATH_EPCONFIGPTR] = (uint32_t)&ep->ctrl->bufferRam[4*ep->index];

	endpointBufferAlloc(ep, 0, txBufSize);
	endpointBufferAlloc(ep, 1, rxBufSize);

	endpointFastPathConfigureNextBuffer(ep, 0, 0);
	endpointFastPathConfigureNextBuffer(ep, 1, 0);
	ctrl->endpointIndex++;

	endpointStateSet(ep, USBEP_STATE_NAK, USBEP_STATE_NAK);

	return ep;
}

uint8_t USBEndpointReceiveSetState(USBEndpoint* ep, USBEndpointState newState){
	endpointStateSet(ep, newState, USBEP_STATE_UNCHANGED);
	return 0;
}

uint8_t USBEndpointTransmitSetState(USBEndpoint* ep, USBEndpointState newState){
	endpointStateSet(ep, USBEP_STATE_UNCHANGED, newState);
	return 0;
}

uint8_t USBEndpointReceive(USBEndpoint* ep, USBEndpointBuffer* buf){
	if (ep->type == USBEP_TYPE_INVALID){
		return 1;
	}

	if(ep->rxBufNumActive == ep->trxBufNum[1]){
		return 2;
	}

	buf->data = (uint8_t*)endpointBufferGetAddress(ep, 1, ep->rxBufRdIndex);
	ep->rxBuf[ep->rxBufRdIndex] = buf;

	ep->rxBufRdIndex++;
	if(ep->rxBufRdIndex >= ep->trxBufNum[1]){
		ep->rxBufRdIndex = 0;
	}

	__disable_irq();
	ep->rxBufNumActive++;

	endpointFastPathConfigureNextBuffer(ep, 1, ep->rxBufNumActive >= 2);

	if(ep->rxBufNumActive == 1){
		USBEndpointReceiveSetState(ep, USBEP_STATE_VALID);
	}
	__enable_irq();

	return 0;
}

uint8_t USBEndpointReceiveGetData(USBEndpoint* ep, USBEndpointBuffer* buf){
	//*buf = ep->rxBuf[ep->rxBufRdIndex];
	//buf->data = (uint8_t*)endpointBufferGetAddress(ep, 1, ep->rxBufRdIndex);

	//ep->rxBufRdIndex++;
	//if(ep->rxBufRdIndex >= ep->trxBufNum[1]){
//		ep->rxBufRdIndex = 0;
//	}

	return 0;
}


/*uint8_t USBEndpointReceive(USBEndpoint*  ep, USBReceiveCallback cb, void* param){
	if (ep->type == USBEP_TYPE_INVALID){
		return 1;
	}

	__disable_irq();
	if(ep->rxBufNumActive == ep->trxBufNum[1]){
		ep->rxCbParam = param;
		ep->rxCb = cb;
	}else{
		ep->rxBufNumActive++;

		endpointFastPathConfigureNextBuffer(ep, 1, ep->rxBufNumActive >= 2);

		if(ep->rxBufNumActive == 1){
			USBEndpointReceiveSetState(ep, USBEP_STATE_VALID);
		}

		//TODO: call CB somehow
		//cb(buf->len, param);
	}
	__enable_irq();

	ep->rxCbParam = param;
	ep->rxCb = cb;

	endpointStateSet(ep, USBEP_STATE_VALID, USBEP_STATE_UNCHANGED);*/

/*	return 0;
}*/

uint8_t USBEndpointTransmit(USBEndpoint* ep, USBEndpointBuffer* buf, USBTransmitCallback cb, void* param){
	if (ep->type == USBEP_TYPE_INVALID){
		return 1;
	}

	volatile uint16_t* pktBuf = endpointBufferGetAddress(ep, 0, ep->txBufWrIndex);
	ep->txBufLen[ep->txBufWrIndex] = buf->len;

    /* Written in ASM as the memory only supports 16-bit access, and this is performance critical */
	if(buf->len){
		if(((uint32_t)buf->data) & 1){
			/* Unaligned copy, if buf->len&1 we copied one byte too many, but this is ok as the memory
			 * is half-word aligned and the length is correct */

			uint32_t work, work2;
			uint32_t cnt = 0;
			asm volatile ("loop%=: \n"
				 "ldrb %[work], [%[src],  %[cnt]] \n"
				 "add %[cnt], #1 \n"
				 "ldrb %[work2], [%[src], %[cnt]] \n"
				 "lsl %[work2], #8 \n"
			     "orr %[work], %[work2] \n"
				 "add %[cnt], #1 \n"
				 "strh %[work], [%[dest], %[cnt]] \n"
				 "cmp %[cnt], %[len] \n"
				 "blt loop%=     \n"
					: [work]"=&l"(work), [work2]"=&l"(work2), [cnt]"+l"(cnt)
					: [len]"l"(buf->len), [src]"l"(buf->data), [dest]"l"(pktBuf-1)
					: "memory");

		}else{
			/* Aligned copy (also copies one byte too many for buf->len&1, but this is ok) */
			uint32_t work;
			uint32_t cnt = 0;
			asm volatile ("loop%=: \n"
				 "ldrh %[work], [%[src],  %[cnt]] \n"
				 "strh %[work], [%[dest], %[cnt]] \n"
				 "add %[cnt], #2 \n"
				 "cmp %[cnt], %[len] \n"
				 "blt loop%=     \n"
					: [work]"=&l"(work), [cnt]"+l"(cnt)
					: [len]"l"(buf->len), [src]"l"(buf->data), [dest]"l"(pktBuf)
					: "memory");
		}
	}

	ep->txBufWrIndex++;
	if(ep->txBufWrIndex >= ep->trxBufNum[0]){
		ep->txBufWrIndex = 0;
	}

	__disable_irq();
	ep->txBufNumActive++;

	endpointFastPathConfigureNextBuffer(ep, 0, ep->txBufNumActive >= 2);

	if(ep->txBufNumActive == 1){
		endpointBufferSetTXLen(ep, buf->len);
		USBEndpointTransmitSetState(ep, USBEP_STATE_VALID);
	}

	if(ep->txBufNumActive == ep->trxBufNum[0]){
		ep->txCbParam = param;
		ep->txCb = cb;
	}else{
		cb(buf->len, param);
	}
	__enable_irq();

	return 0;
}

USBController* USBEndpointGetController(USBEndpoint* ep){
	return ep->ctrl;
}
