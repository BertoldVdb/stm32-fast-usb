#include "chip.h"

static inline void enableHSI(uint8_t on){
    if(on){
        RCC->CR |= RCC_CR_HSION;
        while(!(RCC->CR & RCC_CR_HSIRDY));
    }else{
        RCC->CR &=~ RCC_CR_HSION;
    }
}

static inline void enableHSI48(uint8_t on){
    if(on){
        RCC->CR2 |= RCC_CR2_HSI48ON;
        while(!(RCC->CR2 & RCC_CR2_HSI48RDY));
    }else{
        RCC->CR2 &=~ RCC_CR2_HSI48ON;
    }
}

static inline void setSystemClock(uint8_t sysClock){
    sysClock &= 0x3;

    /* Request clock change */
    RCC->CFGR &=~ RCC_CFGR_SW;
    RCC->CFGR |= sysClock;

    /* Wait for clock change */
    for(;;){
        uint8_t selected = (RCC->CFGR >> 2) & RCC_CFGR_SW;
        if(selected == sysClock) break;
    }
}

static inline void setSystemNVMLatency(uint8_t latency){
    if(latency >= 1){
        FLASH->ACR |= FLASH_ACR_LATENCY;
    }else{
        FLASH->ACR &=~ FLASH_ACR_LATENCY;
    }
}

void halInitSystem(){
    /* Reset and enable power */
    RCC->APB1RSTR = RCC_APB1RSTR_PWRRST;
    RCC->APB1RSTR = 0;
    RCC->APB1ENR  = RCC_APB1ENR_PWREN;

    setSystemNVMLatency(1);

    enableHSI48(1);
    setSystemClock(RCC_CFGR_SW_HSI48);

    /* Enable flash prefetch */
    FLASH->ACR |= FLASH_ACR_PRFTBE;
}



