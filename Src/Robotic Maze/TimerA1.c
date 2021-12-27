// TimerA1.c
// Runs on MSP432
// Use Timer A1 in periodic mode to request interrupts at a particular
// period.
// Daniel Valvano
// July 5, 2017

/* This example accompanies the books
   "Embedded Systems: Introduction to the MSP432 Microcontroller",
       ISBN: 978-1512185676, Jonathan Valvano, copyright (c) 2017
   "Embedded Systems: Real-Time Interfacing to the MSP432 Microcontroller",
       ISBN: 978-1514676585, Jonathan Valvano, copyright (c) 2017
   "Embedded Systems: Real-Time Operating Systems for ARM Cortex-M Microcontrollers",
       ISBN: 978-1466468863, , Jonathan Valvano, copyright (c) 2017
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/

Simplified BSD License (FreeBSD License)
Copyright (c) 2017, Jonathan Valvano, All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are
those of the authors and should not be interpreted as representing official
policies, either expressed or implied, of the FreeBSD Project.
*/

#include <stdint.h>
#include "msp.h"

void dummy(void) {
    return;
}

void (*TimerA1Task0)(void) = dummy;   // user function
void (*TimerA1Task1)(void) = dummy;
void (*TimerA1Task2)(void) = dummy;
// ***************** TimerA1_Init ****************
// Activate Timer A1 interrupts to run user task periodically
// Inputs:  task is a pointer to a user function
//          period in units (24/SMCLK), 16 bits
// Outputs: none
void TimerA1_Init(void(*task0)(void), uint16_t period, 
									void(*task1)(void), uint16_t delay1,
									void(*task2)(void), uint16_t delay2){
  TimerA1Task0 = task0;             // user function
  TimerA1Task1 = task1;             // user function
  TimerA1Task2 = task2;             // user function
  // bits15-10=XXXXXX, reserved
  // bits9-8=10,       clock source to SMCLK
  // bits7-6=10,       input clock divider /4
  // bits5-4=00,       stop mode
  // bit3=X,           reserved
  // bit2=0,           set this bit to clear
  // bit1=0,           no interrupt on timer
  TIMER_A1->CTL = TIMER_A_CTL_SSEL__SMCLK | TIMER_A_CTL_MC__STOP | TIMER_A_CTL_CLR;
  TIMER_A2->CTL = TIMER_A_CTL_SSEL__SMCLK | TIMER_A_CTL_MC__STOP | TIMER_A_CTL_CLR;
  // bits15-14=00,     no capture mode
  // bits13-12=XX,     capture/compare input select
  // bit11=X,          synchronize capture source
  // bit10=X,          synchronized capture/compare input
  // bit9=X,           reserved
  // bit8=0,           compare mode
  // bits7-5=XXX,      output mode
  // bit4=1,           enable capture/compare interrupt on CCIFG
  // bit3=X,           read capture/compare input from here
  // bit2=0,           output this value in output mode 0
  // bit1=X,           capture overflow status
  // bit0=0,           clear capture/compare interrupt pending
  //  TIMER_A1->EX0 = 0x0005;    // configure for input clock divider /6
  TIMER_A1->EX0 = 0;
//  TIMER_A1->CCR[0] = (period - 1);   // период опроса.
  TIMER_A2->EX0 = 0;
  TIMER_A2->CCR[0] = (period - 1);   // период опроса.

	TIMER_A1->CCTL[0] = TIMER_A_CCTLN_CCIE;  
	TIMER_A2->CCTL[0] = TIMER_A_CCTLN_CCIE;  
  TIMER_A2->CCR[1] = delay1;						// момент отсечки
  TIMER_A2->CCR[2] = period - 100*12;	// это запуск АЦП для изм.напр.батареи
  TIMER_A1->CCR[0] = delay2;						// момент отсечки
  TIMER_A2->CCTL[1] = TIMER_A_CCTLN_CCIE;
  TIMER_A2->CCTL[2] = TIMER_A_CCTLN_OUTMOD_7;
	
  TIMER_A1->CCTL[1] = TIMER_A_CCTLN_CM__FALLING | TIMER_A_CCTLN_CCIS__CCIA | TIMER_A_CCTLN_SCS | TIMER_A_CCTLN_CAP;
  TIMER_A1->CCTL[2] = TIMER_A_CCTLN_CM__FALLING | TIMER_A_CCTLN_CCIS__CCIA | TIMER_A_CCTLN_SCS | TIMER_A_CCTLN_CAP;
  TIMER_A1->CCTL[3] = TIMER_A_CCTLN_CM__FALLING | TIMER_A_CCTLN_CCIS__CCIA | TIMER_A_CCTLN_SCS | TIMER_A_CCTLN_CAP;
  TIMER_A1->CCTL[4] = TIMER_A_CCTLN_CM__FALLING | TIMER_A_CCTLN_CCIS__CCIA | TIMER_A_CCTLN_SCS | TIMER_A_CCTLN_CAP;

// interrupts enabled in the main program after all devices initialized
//  NVIC->IP[3] = (NVIC->IP[3]&0xFFFFFF00)|0x00000040; // priority 2
//  NVIC->ISER[0] = 0x00001000;   // enable interrupt 12 in NVIC
  NVIC_SetPriority(TA1_0_IRQn, 2);
  NVIC_SetPriority(TA2_0_IRQn, 2);
  NVIC_SetPriority(TA2_N_IRQn, 2);
  NVIC_EnableIRQ(TA1_0_IRQn);
  NVIC_EnableIRQ(TA2_0_IRQn);
  NVIC_EnableIRQ(TA2_N_IRQn);
  TIMER_A1->CTL = TIMER_A_CTL_SSEL__SMCLK | TIMER_A_CTL_MC__CONTINUOUS; // start Timer A1 in continuous mode
  TIMER_A2->CTL = TIMER_A_CTL_SSEL__SMCLK | TIMER_A_CTL_MC__UP;      // start Timer A2 in up mode
	BITBAND_PERI(TIMER_A2->CTL, TIMER_A_CTL_CLR_OFS) = 1;
}


// ------------TimerA1_Stop------------
// Deactivate the interrupt running a user task periodically.
// Input: none
// Output: none
void TimerA1_Stop(void){
  TIMER_A1->CTL = TIMER_A_CTL_SSEL__SMCLK | TIMER_A_CTL_MC__STOP | TIMER_A_CTL_CLR;
  TIMER_A2->CTL = TIMER_A_CTL_SSEL__SMCLK | TIMER_A_CTL_MC__STOP | TIMER_A_CTL_CLR;
//  NVIC->ICER[0] = 0x00001000;     // disable interrupt 12 in NVIC
  NVIC_DisableIRQ(TA2_0_IRQn);
  NVIC_DisableIRQ(TA2_N_IRQn);
}

void TA1_0_IRQHandler(void){
  TIMER_A1->CCTL[0] &= ~0x0001; // acknowledge capture/compare interrupt 0
  (*TimerA1Task2)();             // execute user task
}

void TA2_0_IRQHandler(void){
  TIMER_A2->CCTL[0] &= ~0x0001; // acknowledge capture/compare interrupt 0
  (*TimerA1Task0)();             // execute user task
}

void TA2_N_IRQHandler(void) {
    switch(TIMER_A2->IV) {
    case 2:
        TIMER_A2->CCTL[1] &= ~TIMER_A_CCTLN_CCIFG;
        (*TimerA1Task1)();
        break;

    case 4:
        TIMER_A2->CCTL[2] &= ~TIMER_A_CCTLN_CCIFG;
 //       (*TimerA1Task2)();
        break;

    case 6:
        TIMER_A2->CCTL[3] &= ~TIMER_A_CCTLN_CCIFG;
        break;

    case 8:
        TIMER_A2->CCTL[4] &= ~TIMER_A_CCTLN_CCIFG;
        break;

    case 0x0e:
        TIMER_A2->CTL &= ~TIMER_A_CTL_IFG;
        (*TimerA1Task0)();
        break;

    default:
        break;
//        TIMER_A2->CCTL[5] &= ~TIMER_A_CCTLN_CCIFG;
//        TIMER_A2->CCTL[6] &= ~TIMER_A_CCTLN_CCIFG;
    }
}
