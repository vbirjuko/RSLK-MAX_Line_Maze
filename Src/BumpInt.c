// BumpInt.c
// Runs on MSP432, interrupt version
// Provide low-level functions that interface bump switches the robot.
// Daniel Valvano and Jonathan Valvano
// July 2, 2017

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

// Negative logic bump sensors
// P4.7 Bump5, left side of robot
// P4.6 Bump4
// P4.5 Bump3
// P4.3 Bump2
// P4.2 Bump1
// P4.0 Bump0, right side of robot

#ifdef RSLK_MAX
#define BUMP0   (1u << 7)
#define BUMP1   (1u << 6)
#define BUMP2   (1u << 5)
#define BUMP3   (1u << 3)
#define BUMP4   (1u << 2)
#define BUMP5   (1u << 0)
#else
#define BUMP0   (1u << 5)
#define BUMP1   (1u << 6)
#define BUMP2   (1u << 7)
#define BUMP3   (1u << 4)
#define BUMP4   (1u << 3)
#define BUMP5   (1u << 0)
#endif

#define BUMP_PORT	P4

#define BUMP_MASK (BUMP0 | BUMP1 | BUMP2 | BUMP3 | BUMP4 | BUMP5)
const unsigned int sequence[] = {BUMP5, BUMP4, BUMP3, BUMP2, BUMP1, BUMP0};

#include <stdint.h>
#include "msp.h"

void Bump_Init(void){
    // write this as part of Lab 10
  BUMP_PORT->SEL0 &= ~BUMP_MASK;
  BUMP_PORT->SEL1 &= ~BUMP_MASK;
  BUMP_PORT->DIR  &= ~BUMP_MASK;
  BUMP_PORT->OUT  |=  BUMP_MASK;
  BUMP_PORT->REN  |=  BUMP_MASK;
}


// Initialize Bump sensors
// Make six Port 4 pins inputs
// Activate interface pullup
// pins 7,6,5,3,2,0
// Interrupt on falling edge (on touch)

void dummy_bump(uint8_t data) { };

void (*BumpIntTask)(uint8_t)  = dummy_bump;   // user function
void BumpInt_Init(void(*task)(uint8_t)){
    // write this as part of Lab 14
    BumpIntTask = task;
    P4->SEL0 &= ~BUMP_MASK;
    P4->SEL1 &= ~BUMP_MASK;
    P4->DIR  &= ~BUMP_MASK;
    P4->OUT  |=  BUMP_MASK;
    P4->REN  |=  BUMP_MASK;
    P4->IES  |=  BUMP_MASK;
    P4->IFG  &= ~BUMP_MASK;   // clear flags
    P4->IE   |=  BUMP_MASK;     // arm interrupt
//    NVIC->IP[9] = (NVIC->IP[9]&0xFF00FFFF)|0x00400000; // priority 2
//    NVIC->ISER[1] = 0x00000040;     // enable interrupt 38 in NVIC
    NVIC_SetPriority(PORT4_IRQn, 2);
    NVIC_EnableIRQ(PORT4_IRQn);
}

// Read current state of 6 switches
// Returns a 6-bit positive logic result (0 to 63)
// bit 5 Bump5
// bit 4 Bump4
// bit 3 Bump3
// bit 2 Bump2
// bit 1 Bump1
// bit 0 Bump0
uint8_t Bump_Read(void){
    // write this as part of Lab 14
//    unsigned int bumps;
//    bumps = ((P4->IN & KEY_MASK) ^ KEY_MASK);
//    return ((bumps & 0xE0) >> 2) | ((bumps & 0xC0) >> 1) | (bumps & 0x01); // DEPENDED ON KEY_MASK

    uint32_t result = 0, sensors, i, mask = 0x01;

    sensors = ((P4->IN & BUMP_MASK) ^ BUMP_MASK);
    for (i = 0; i < 6; i++ ) {
        if (sensors & sequence[i]) {
            result |= mask;
        }
        mask <<= 1;
    }
    return result;
}
// we do not care about critical section/race conditions
// triggered on touch, falling edge
void PORT4_IRQHandler(void){
    // write this as part of Lab 14
    uint32_t result = 0, i, mask = 0x01;

    for (i = 0; i < 6; i++ ) {
        if (P4->IFG & sequence[i]) result |= mask;
        mask <<= 1;
		}
    P4->IFG = 0;
    (*BumpIntTask)(result);
}

void BumpInt_Stop(void) {
	NVIC_DisableIRQ(PORT4_IRQn);
	BumpIntTask = dummy_bump;
}

int Bump_angle(uint8_t bump_data) {
	const int angle[] = {-45, -27, -9, 9, 27, 45};
	unsigned int first_sensor = 256, last_sensor = 256, i, mask = (1 << 0);
	for (i = 0; i < 6; i++) {
		if (bump_data & mask) {
			last_sensor = i;
			if (first_sensor == 256) first_sensor = i;
		}
		mask <<= 1;
	}

	if (first_sensor == 256) return 90;
  else 	return ((angle[first_sensor] + angle[last_sensor])/2 );
}
