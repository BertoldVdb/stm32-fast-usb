#include <stdint.h>
#include "stm32f0xx.h"
#include "chip.h"

extern uint8_t __data_ram_begin;
extern uint8_t __data_ram_end;
extern uint8_t __data_flash_begin;

extern uint8_t __bss_ram_begin;
extern uint8_t __bss_ram_end;

__attribute__((noreturn)) void halHalt(){
    __disable_irq();
    while(1);
}

__attribute__((noreturn)) void halReset(){
	SCB->AIRCR = (0x5FA << 16) | SCB_AIRCR_SYSRESETREQ_Msk;
	while(1);
}

__attribute__((noreturn)) void ResetHandler(){
    __disable_irq();

    /* Zero .BSS section */
    uint8_t* bssPtr = &__bss_ram_begin;
    while(bssPtr < &__bss_ram_end){
        *bssPtr = 0;
        bssPtr++;
    }

    /* Copy .DATA section from FLASH to RAM */
    uint8_t* dataRomPtr = &__data_flash_begin;
    uint8_t* dataRamPtr = &__data_ram_begin;
    while(dataRamPtr < &__data_ram_end){
        *dataRamPtr = *dataRomPtr;
        dataRamPtr++;
        dataRomPtr++;
    }

    halInitSystem();
    halInitTick();

    /* Run main */
    __enable_irq();
    main();

    /* Do nothing */
    halHalt();
}

static void UnhandledVector(){
    halHalt();
}

__attribute__((alias("UnhandledVector"), weak)) void NMIHandler();
__attribute__((alias("UnhandledVector"), weak)) void HardFaultHandler();
__attribute__((alias("UnhandledVector"), weak)) void SVCallHandler();
__attribute__((alias("UnhandledVector"), weak)) void PendSVHandler();
__attribute__((alias("UnhandledVector"), weak)) void SysTickHandler();
__attribute__((alias("UnhandledVector"), weak)) void WWDGHandler();
__attribute__((alias("UnhandledVector"), weak)) void PVDVDDIO2Handler();
__attribute__((alias("UnhandledVector"), weak)) void RTCHandler();
__attribute__((alias("UnhandledVector"), weak)) void FLASHHandler();
__attribute__((alias("UnhandledVector"), weak)) void RCCCRSHandler();
__attribute__((alias("UnhandledVector"), weak)) void EXTI01Handler();
__attribute__((alias("UnhandledVector"), weak)) void EXTI23Handler();
__attribute__((alias("UnhandledVector"), weak)) void EXTI415Handler();
__attribute__((alias("UnhandledVector"), weak)) void TSCHandler();
__attribute__((alias("UnhandledVector"), weak)) void DMA1Channel1Handler();
__attribute__((alias("UnhandledVector"), weak)) void DMA1Channel23Handler();
__attribute__((alias("UnhandledVector"), weak)) void DMA1Channel4567Handler();
__attribute__((alias("UnhandledVector"), weak)) void ADC1COMPHandler();
__attribute__((alias("UnhandledVector"), weak)) void TIM1BRKUPTRGCOMHandler();
__attribute__((alias("UnhandledVector"), weak)) void TIM1CCHandler();
__attribute__((alias("UnhandledVector"), weak)) void TIM2Handler();
__attribute__((alias("UnhandledVector"), weak)) void TIM3Handler();
__attribute__((alias("UnhandledVector"), weak)) void TIM6DACHandler();
__attribute__((alias("UnhandledVector"), weak)) void TIM7Handler();
__attribute__((alias("UnhandledVector"), weak)) void TIM14Handler();
__attribute__((alias("UnhandledVector"), weak)) void TIM15Handler();
__attribute__((alias("UnhandledVector"), weak)) void TIM16Handler();
__attribute__((alias("UnhandledVector"), weak)) void TIM17Handler();
__attribute__((alias("UnhandledVector"), weak)) void I2C1Handler();
__attribute__((alias("UnhandledVector"), weak)) void I2C2Handler();
__attribute__((alias("UnhandledVector"), weak)) void SPI1Handler();
__attribute__((alias("UnhandledVector"), weak)) void SPI2Handler();
__attribute__((alias("UnhandledVector"), weak)) void USART1Handler();
__attribute__((alias("UnhandledVector"), weak)) void USART2Handler();
__attribute__((alias("UnhandledVector"), weak)) void USART34Handler();
__attribute__((alias("UnhandledVector"), weak)) void CECCANHandler();
__attribute__((alias("UnhandledVector"), weak)) void USBHandler();

typedef void(*VectorEntry)();

__attribute__((used, section(".vectors"))) const VectorEntry vectors[] = {
	    ResetHandler,
	    NMIHandler,
	    HardFaultHandler,
	    UnhandledVector,
	    UnhandledVector,
	    UnhandledVector,
	    UnhandledVector,
	    UnhandledVector,
	    UnhandledVector,
	    UnhandledVector,
	    SVCallHandler,
	    UnhandledVector,
	    UnhandledVector,
	    PendSVHandler,
	    SysTickHandler,
		WWDGHandler,
		PVDVDDIO2Handler,
		RTCHandler,
		FLASHHandler,
		RCCCRSHandler,
		EXTI01Handler,
		EXTI23Handler,
		EXTI415Handler,
		TSCHandler,
		DMA1Channel1Handler,
		DMA1Channel23Handler,
		DMA1Channel4567Handler,
		ADC1COMPHandler,
		TIM1BRKUPTRGCOMHandler,
		TIM1CCHandler,
		TIM2Handler,
		TIM3Handler,
		TIM6DACHandler,
		TIM7Handler,
		TIM14Handler,
		TIM15Handler,
		TIM16Handler,
		TIM17Handler,
		I2C1Handler,
		I2C2Handler,
		SPI1Handler,
		SPI2Handler,
		USART1Handler,
		USART2Handler,
		USART34Handler,
		CECCANHandler,
		USBHandler,
};

