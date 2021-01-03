#include "chip.h"

static volatile uint32_t systime;

uint32_t halGetTime(){
    return systime;
}

void halInitTick(){
    SysTick->LOAD=F_CPU/1000;
    SysTick->VAL=0;
    SysTick->CTRL=SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk;
}

void SysTickHandler(){
    systime++;
}

void halDelay(uint32_t ms){
    uint32_t target = halGetTime() + ms;
    while(halGetTime() < target);
}
