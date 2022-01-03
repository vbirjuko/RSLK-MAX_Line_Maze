/*
 * Timer32.c
 *
 *  Created on: 6 янв. 2021 г.
 *      Author: wl
 */
#include "msp.h"
#include "CortexM.h"
#include "resources.h"

volatile unsigned int timer_busy = 0;
void Timer32_Init(void) {
    TIMER32_1->CONTROL = TIMER32_CONTROL_MODE | TIMER32_CONTROL_ONESHOT |
            TIMER32_CONTROL_PRESCALE_1 | TIMER32_CONTROL_SIZE | TIMER32_CONTROL_IE;
    NVIC_SetPriority(T32_INT1_IRQn, T32_INT1_Priority);
    NVIC_EnableIRQ(T32_INT1_IRQn);
}

void delay_us(uint32_t delay) {

    timer_busy = 1;
    TIMER32_1->LOAD = delay * 3;
    TIMER32_1->CONTROL |= TIMER32_CONTROL_ENABLE;
    while (timer_busy) WaitForInterrupt();
}

void T32_INT1_IRQHandler(void) {
    TIMER32_1->INTCLR = 0;
    timer_busy = 0;
}


void benchmark_start(void) {
    TIMER32_2->CONTROL = TIMER32_CONTROL_MODE | TIMER32_CONTROL_ONESHOT |
            TIMER32_CONTROL_PRESCALE_1 | TIMER32_CONTROL_SIZE;
    TIMER32_2->LOAD = 0xFFFFFFFF;
    TIMER32_2->CONTROL |= TIMER32_CONTROL_ENABLE;
}

unsigned int benchmark_stop(void) {
    TIMER32_2->CONTROL &= ~TIMER32_CONTROL_ENABLE;
    return 0xFFFFFFFF - TIMER32_2->VALUE;
}
