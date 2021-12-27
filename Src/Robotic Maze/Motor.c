// Motor.c
// Runs on MSP432
// Provide mid-level functions that initialize ports and
// set motor speeds to move the robot. Lab 13 starter
// Daniel Valvano
// July 8, 2017

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

// Sever VCCMD=VREG jumper on Motor Driver and Power Distribution Board and connect VCCMD to 3.3V.
//   This makes P3.7 and P3.6 low power disables for motor drivers.  0 to sleep/stop.
// Sever nSLPL=nSLPR jumper.
//   This separates P3.7 and P3.6 allowing for independent control
// Left motor direction connected to P1.7 (J2.14)
// Left motor PWM connected to P2.7/TA0CCP4 (J4.40)
// Left motor enable connected to P3.7 (J4.31)
// Right motor direction connected to P1.6 (J2.15)
// Right motor PWM connected to P2.6/TA0CCP3 (J4.39)
// Right motor enable connected to P3.6 (J2.11)

#include <stdint.h>
#include "msp.h"
#include "CortexM.h"
//#include "PWM.h"

// *******Lab 13 solution*******
#ifdef RSLK_MAX
#define LEFT    7
#define RIGHT   6
#define ENABLE_PORT     P2
#define SLEEP_PORT      P3
#define DIRECTION_PORT  P5
#define DIRECTION_LEFT	4
#define DIRECTION_RIGHT	5
#else
#define LEFT    7
#define RIGHT   6
#define DIRECTION_PORT  P1
#define ENABLE_PORT     P2
#define SLEEP_PORT      P3
#define DIRECTION_LEFT	LEFT
#define DIRECTION_RIGHT	RIGHT
#endif
// ------------Motor_Init------------
// Initialize GPIO pins for output, which will be
// used to control the direction of the motors and
// to enable or disable the drivers.
// The motors are initially stopped, the drivers
// are initially powered down, and the PWM speed
// control is uninitialized.
// Input: none
// Output: none
void Motor_Init(void){
  // write this as part of Lab 13
    // Direction
  DIRECTION_PORT->SEL0 &= ~((1u << DIRECTION_LEFT)|(1u << DIRECTION_RIGHT));
  DIRECTION_PORT->SEL1 &= ~((1u << DIRECTION_LEFT)|(1u << DIRECTION_RIGHT));
  DIRECTION_PORT->DIR  |=   (1u << DIRECTION_LEFT)|(1u << DIRECTION_RIGHT);
  DIRECTION_PORT->OUT  &= ~((1u << DIRECTION_LEFT)|(1u << DIRECTION_RIGHT));
      // nSleep
  SLEEP_PORT->SEL0 &= ~((1u << LEFT)|(1u << RIGHT));
  SLEEP_PORT->SEL1 &= ~((1u << LEFT)|(1u << RIGHT));
  SLEEP_PORT->DIR  |=   (1u << LEFT)|(1u << RIGHT);
  SLEEP_PORT->OUT  &= ~((1u << LEFT)|(1u << RIGHT));
      // PWM
  ENABLE_PORT->DIR   |=  (1u << LEFT)|(1u << RIGHT); // P2.6, P2.7 output
  ENABLE_PORT->SEL0  |=  (1u << LEFT)|(1u << RIGHT);   // P2.6, P2.7 Timer0A functions
  ENABLE_PORT->SEL1  &= ~((1u << LEFT)|(1u << RIGHT)); // P2.6, P2.7 Timer0A functions
  
  TIMER_A0->CCTL[0] = 0x0080; // CCI0 toggle
  TIMER_A0->CCR[0] = 15000; // Period is 2*period*8*83.33ns is 1.333*period -> 20ms
  TIMER_A0->EX0 = 0x0000; // divide by 1
  TIMER_A0->CCTL[3] = 0x0040; // CCR1 toggle/reset
  TIMER_A0->CCR[3] = 0; // CCR1 duty cycle is duty1/period
  TIMER_A0->CCTL[4] = 0x0040; // CCR2 toggle/reset
  TIMER_A0->CCR[4] = 0; // CCR2 duty cycle is duty2/period
  TIMER_A0->CTL = TIMER_A_CTL_SSEL__SMCLK | TIMER_A_CTL_ID__1 | TIMER_A_CTL_MC__UPDOWN;// 0x02F0; // SMCLK=12MHz, divide by 8, up-down mode
  // bit mode
  // 9-8 10 TASSEL, SMCLK=12MHz
  // 7-6 11 ID, divide by 8
  // 5-4 11 MC, up-down mode
  // 2 0 TACLR, no clear
  // 1 0 TAIE, no interrupt
  // 0 TAIFG
}

// ------------Motor_Stop------------
// Stop the motors, power down the drivers, and
// set the PWM speed control to 0% duty cycle.
// Input: none
// Output: none
void Motor_Stop(void){
  // write this as part of Lab 13
    TIMER_A0->CCR[3] = 0;
    TIMER_A0->CCR[4] = 0;
//    SLEEP_PORT->OUT &= ~((1u << LEFT)|(1u << RIGHT));
}

void Motor_Speed (int16_t left, int16_t right) {
    SLEEP_PORT->OUT |=  (1u << LEFT)|(1u << RIGHT);
    BITBAND_PERI(DIRECTION_PORT->OUT, DIRECTION_LEFT) = (left < 0) ? 1 : 0;
    BITBAND_PERI(DIRECTION_PORT->OUT, DIRECTION_RIGHT) = (right < 0) ? 1 : 0;
    if (left < 0) left = -left;
    if (right < 0) right = -right;
    TIMER_A0->CCR[3] = (uint16_t)right;
    TIMER_A0->CCR[4] = (uint16_t)left;
}


// ------------Motor_Forward------------
// Drive the robot forward by running left and
// right wheels forward with the given duty
// cycles.
// Input: leftDuty  duty cycle of left wheel (0 to 14,998)
//        rightDuty duty cycle of right wheel (0 to 14,998)
// Output: none
// Assumes: Motor_Init() has been called
void Motor_Forward(uint16_t leftDuty, uint16_t rightDuty){ 
  // write this as part of Lab 13
    SLEEP_PORT->OUT |=  (1u << LEFT)|(1u << RIGHT);
    DIRECTION_PORT->OUT &=  ~((1u << DIRECTION_LEFT)|(1u << DIRECTION_RIGHT));
    TIMER_A0->CCR[3] = rightDuty;
    TIMER_A0->CCR[4] = leftDuty;
}

// ------------Motor_Right------------
// Turn the robot to the right by running the
// left wheel forward and the right wheel
// backward with the given duty cycles.
// Input: leftDuty  duty cycle of left wheel (0 to 14,998)
//        rightDuty duty cycle of right wheel (0 to 14,998)
// Output: none
// Assumes: Motor_Init() has been called
void Motor_Right(uint16_t leftDuty, uint16_t rightDuty){ 
  // write this as part of Lab 13
    DIRECTION_PORT->OUT |=  (1u << DIRECTION_RIGHT); // backward
    DIRECTION_PORT->OUT &= ~(1u << DIRECTION_LEFT);  // forward
    SLEEP_PORT->OUT |=  (1u << LEFT)|(1u << RIGHT);
    TIMER_A0->CCR[3] = rightDuty;
    TIMER_A0->CCR[4] = leftDuty;
}

// ------------Motor_Left------------
// Turn the robot to the left by running the
// left wheel backward and the right wheel
// forward with the given duty cycles.
// Input: leftDuty  duty cycle of left wheel (0 to 14,998)
//        rightDuty duty cycle of right wheel (0 to 14,998)
// Output: none
// Assumes: Motor_Init() has been called
void Motor_Left(uint16_t leftDuty, uint16_t rightDuty){ 
  // write this as part of Lab 13
    DIRECTION_PORT->OUT |=  (1u << DIRECTION_LEFT);   // backward
    DIRECTION_PORT->OUT &= ~(1u << DIRECTION_RIGHT);  // forward
    SLEEP_PORT->OUT |=  (1u << LEFT)|(1u << RIGHT);
    TIMER_A0->CCR[3] = rightDuty;
    TIMER_A0->CCR[4] = leftDuty;
}

// ------------Motor_Backward------------
// Drive the robot backward by running left and
// right wheels backward with the given duty
// cycles.
// Input: leftDuty  duty cycle of left wheel (0 to 14,998)
//        rightDuty duty cycle of right wheel (0 to 14,998)
// Output: none
// Assumes: Motor_Init() has been called
void Motor_Backward(uint16_t leftDuty, uint16_t rightDuty){ 
  // write this as part of Lab 13
    DIRECTION_PORT->OUT |= (1u << DIRECTION_LEFT)|(1u << DIRECTION_RIGHT);
    SLEEP_PORT->OUT     |= (1u << LEFT)|(1u << RIGHT);
    TIMER_A0->CCR[3] = rightDuty;
    TIMER_A0->CCR[4] = leftDuty;
}
