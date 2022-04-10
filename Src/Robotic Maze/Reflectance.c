// Reflectance.c
// Provide functions to take measurements using a QTR-8RC reflectance
// sensor array (Pololu part number 961).  This works by outputting to
// the sensor, waiting, then reading the digital value of each of the
// eight phototransistors.  The more reflective the target surface is,
// the faster the voltage decays.
// Student version of GPIO lab
// Daniel and Jonathan Valvano
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


// reflectance LED illuminate connected to P5.3
// reflectance sensor 1 connected to P7.0 (robot's right, robot off road to left)
// reflectance sensor 2 connected to P7.1
// reflectance sensor 3 connected to P7.2
// reflectance sensor 4 connected to P7.3 center
// reflectance sensor 5 connected to P7.4 center
// reflectance sensor 6 connected to P7.5
// reflectance sensor 7 connected to P7.6
// reflectance sensor 8 connected to P7.7 (robot's left, robot off road to right)

#include <stdint.h>
#include "msp432.h"
#include "resources.h"
#include "Clock.h"
#include "TimerA1.h"
#include "main.h"
#include "Maze.h"
#include "Timer32.h"

volatile uint8_t photo_array[8];
volatile uint8_t current_sensor, photo_data_ready;

// ------------Reflectance_Init------------
// Initialize the GPIO pins associated with the QTR-8RC
// reflectance sensor.  Infrared illumination LEDs are
// initially off.
// Input: none
// Output: none
#ifdef RSLK_MAX
	#define IRLED(x)		BITBAND_PERI(P5->OUT, 3) = x; BITBAND_PERI(P9->OUT, 2) = x /* port 5 pin 3 */
	#define IRSENSOR P7
#else
	#define IRLED(x)   BITBAND_PERI(P5->OUT, 3) = x /* port 5 pin 3 */
	#define IRSENSOR P7
#endif


void Timer1_CCR1_Handler(void) {
//		current_sensor = Reflectance_End();
	current_sensor = IRSENSOR->IN;
	if (TIMER_A1->CCTL[2] & TIMER_A_CCTLN_CCI) current_sensor |= (1<<2);
	if (TIMER_A1->CCTL[3] & TIMER_A_CCTLN_CCI) current_sensor |= (1<<3);
	if (TIMER_A1->CCTL[4] & TIMER_A_CCTLN_CCI) current_sensor |= (1<<4);
	if (TIMER_A1->CCTL[1] & TIMER_A_CCTLN_CCI) current_sensor |= (1<<5);
	
//	current_bumper = Bump_Read();
}

void Timer1_reflectance(void) {
	unsigned int captured_val[8];
//	IRLED(0);   //  1) Set P5.3 low (turn off IR LED)
  if (TIMER_A1->CCTL[2] & TIMER_A_CCTLN_CCIFG) {captured_val[2] = TIMER_A1->CCR[2]; TIMER_A1->CCTL[2] &= ~TIMER_A_CCTLN_CCIFG;} else captured_val[2] = 0xFFFF;
  if (TIMER_A1->CCTL[3] & TIMER_A_CCTLN_CCIFG) {captured_val[3] = TIMER_A1->CCR[3]; TIMER_A1->CCTL[3] &= ~TIMER_A_CCTLN_CCIFG;} else captured_val[3] = 0xFFFF;
  if (TIMER_A1->CCTL[4] & TIMER_A_CCTLN_CCIFG) {captured_val[4] = TIMER_A1->CCR[4]; TIMER_A1->CCTL[4] &= ~TIMER_A_CCTLN_CCIFG;} else captured_val[4] = 0xFFFF;
  if (TIMER_A1->CCTL[1] & TIMER_A_CCTLN_CCIFG) {captured_val[5] = TIMER_A1->CCR[1]; TIMER_A1->CCTL[1] &= ~TIMER_A_CCTLN_CCIFG;} else captured_val[5] = 0xFFFF;
//    photo_data_ready  = 1;
//    Reflectance_Start();
  IRSENSOR->SEL0 = 0;   // GPIO
  IRSENSOR->OUT =  0xff;
  IRSENSOR->DIR =  0xff;    //  2) Make P7.0 an output, and set it high (charging the capacitor)
  photo_array[0] = (current_sensor & (1 << 0)) ? 255 : 0;
  photo_array[1] = (current_sensor & (1 << 1)) ? 255 : 0;
//  photo_array[2] = (current_sensor & (1 << 2)) ? 255 : 0;
//  photo_array[3] = (current_sensor & (1 << 3)) ? 255 : 0;
  photo_array[2] = (captured_val[2] < 2048*12) ? captured_val[2] / (12*8) : 255;
  photo_array[3] = (captured_val[3] < 2048*12) ? captured_val[3] / (12*8) : 255;
  photo_array[4] = (captured_val[4] < 2048*12) ? captured_val[4] / (12*8) : 255;
  photo_array[5] = (captured_val[5] < 2048*12) ? captured_val[5] / (12*8) : 255;
//  photo_array[4] = (current_sensor & (1 << 4)) ? 255 : 0;
//  photo_array[5] = (current_sensor & (1 << 5)) ? 255 : 0;
  photo_array[6] = (current_sensor & (1 << 6)) ? 255 : 0;
  photo_array[7] = (current_sensor & (1 << 7)) ? 255 : 0;
  photo_data_ready   = 1;
}

//  Clock_Delay1us(10); //  3) Wait 10 us, Clock_Delay1us(10);
void Reflectance_Stop_charge(void) {
  IRSENSOR->DIR =  0x00;    //  4) Make P7.0 an input
  IRSENSOR->SEL0 = ((1 << 2) | (1 << 3) | (1 << 4) | (1 << 5)); // switch to port mapped function.// Timer capture
  IRSENSOR->SEL1 = 0;
  BITBAND_PERI(TIMER_A1->CTL, TIMER_A_CTL_CLR_OFS) = 1; 
	IRLED(1);   //  1) Set P5.3 high (turn on IR LED)
}

void Reflectance_Init(void){
    // write this as part of Lab 6
    P5->SEL0 &= ~(1 << 3);
    P5->SEL1 &= ~(1 << 3);
    P5->DIR |= (1 << 3);
    P5->OUT &= ~(1 << 3);
#ifdef RSLK_MAX
    P9->SEL0 &= ~(1 << 2);
    P9->SEL1 &= ~(1 << 2);
    P9->DIR |= (1 << 2);
    P9->OUT &= ~(1 << 2);
#endif
	

    IRSENSOR->SEL0 = 0x00;
    IRSENSOR->SEL1 = 0x00;
    IRSENSOR->DIR = 0xff;
    IRSENSOR->OUT = 0xff;
}

void Reflectance_Init_with_Timer(unsigned int threshold){
	unsigned int i;
		TimerA1_Stop();
    // write this as part of Lab 6
    P5->SEL0 &= ~(1 << 3);
    P5->SEL1 &= ~(1 << 3);
    P5->DIR |= (1 << 3);
    P5->OUT |= (1 << 3);
#ifdef RSLK_MAX
    P9->SEL0 &= ~(1 << 2);
    P9->SEL1 &= ~(1 << 2);
    P9->DIR |= (1 << 2);
    P9->OUT |= (1 << 2);
#endif
		// установка уровня инфракрасной подсветки
		IRLED(0);		// выключить на 3 мс
//		Clock_Delay1ms(3);
		delay_us(3000);
		IRLED(1);
//      Clock_Delay1ms(3);
        delay_us(3000);
		for (i = 32; i > data.ir_led_level; i--) {
			IRLED(0);
			delay_us(10);
			IRLED(1);
			delay_us(10);
		}

//    IRSENSOR->SEL0 = 0x00;
    IRSENSOR->SEL0 = ((1 << 2) | (1 << 3) | (1 << 4) | (1 << 5)); // switch to port mapped function.
    IRSENSOR->SEL1 = 0x00;
    IRSENSOR->DIR = 0xff;
    IRSENSOR->OUT = 0xff;

		TimerA1_Init(	&Reflectance_Stop_charge, 		2500*12,
									&Timer1_reflectance, 		 (2500-20)*12, 
									&Timer1_CCR1_Handler, 	 threshold*12);
}

/*
// ------------Reflectance_Read------------
// Read the eight sensors
// Turn on the 8 IR LEDs
// Pulse the 8 sensors high for 10 us
// Make the sensor pins input
// wait t us
// Read sensors
// Turn off the 8 IR LEDs
// Input: time to wait in usec
// Output: sensor readings
// Assumes: Reflectance_Init() has been called
uint8_t Reflectance_Read(uint32_t time){
    uint8_t result;
		IRLED(1);   //  1) Set P5.3 high (turn on IR LED)
    IRSENSOR->DIR =  0xff;      //  2) Make P7.0 an output, and set it high (charging the capacitor)
    IRSENSOR->OUT =  0xff;
    Clock_Delay1us(10);         //  3) Wait 10 us, Clock_Delay1us(10);
    IRSENSOR->DIR =  0x00;      //  4) Make P7.0 an input
    Clock_Delay1us(time);
    result = IRSENSOR->IN;

		IRLED(0);									//  6) Set P5.3 low (turn off IR LED, saving power)
    return result;
}

// ------------Reflectance_Center------------
// Read the two center sensors
// Turn on the 8 IR LEDs
// Pulse the 8 sensors high for 10 us
// Make the sensor pins input
// wait t us
// Read sensors
// Turn off the 8 IR LEDs
// Input: time to wait in usec
// Output: 0 (off road), 1 off to left, 2 off to right, 3 on road
// (Left,Right) Sensors
// 1,1          both sensors   on line
// 0,1          just right     off to left
// 1,0          left left      off to right
// 0,0          neither        lost
// Assumes: Reflectance_Init() has been called
uint8_t Reflectance_Center(uint32_t time){
    uint8_t result;

		IRLED(1);									//  1) Set P5.3 high (turn on IR LED)
    
    IRSENSOR->DIR =  0xff;      //  2) Make P7.0 an output, and set it high (charging the capacitor)
    IRSENSOR->OUT =  0xff;
    Clock_Delay1us(10);         //  3) Wait 10 us, Clock_Delay1us(10);
    IRSENSOR->DIR =  0x00;      //  4) Make P7.0 an input
    Clock_Delay1us(time);
    result = IRSENSOR->IN;
		IRLED(0);									//  6) Set P5.3 low (turn off IR LED, saving power)
    result &= (1u << 4) | (1u << 3);
    result >>= 3;
    return result; // replace this line
}


// Perform sensor integration
// Input: data is 8-bit result from line sensor
// Output: position in 0.1mm relative to center of line
int32_t Reflectance_Position(uint8_t data){
    const int W[] = {329, 235, 141, 47, -47, -141, -235, -329};
    unsigned int i;
    int sigma, result, count;
    sigma = 0; count = 0;
    for (i=0; i<8; i++) {
        if (data & 0x01) {
            sigma += W[i];
            count++;
        }
        data >>= 1;
    }
    if (count)  result = sigma/count;
    else result = 0;
    // write this as part of Lab 6
 return result; // replace this line
}


// ------------Reflectance_Start------------
// Begin the process of reading the eight sensors
// Turn on the 8 IR LEDs
// Pulse the 8 sensors high for 10 us
// Make the sensor pins input
// Input: none
// Output: none
// Assumes: Reflectance_Init() has been called
void Reflectance_Start(void){
    // write this as part of Lab 10
		IRLED(1);									//  1) Set P5.3 high (turn on IR LED)
    IRSENSOR->DIR =  0xff;    //  2) Make P7.0 an output, and set it high (charging the capacitor)
    IRSENSOR->OUT =  0xff;
    Clock_Delay1us(10); //  3) Wait 10 us, Clock_Delay1us(10);
    IRSENSOR->DIR =  0x00;    //  4) Make P7.0 an input
}


// ------------Reflectance_End------------
// Finish reading the eight sensors
// Read sensors
// Turn off the 8 IR LEDs
// Input: none
// Output: sensor readings
// Assumes: Reflectance_Init() has been called
// Assumes: Reflectance_Start() was called 1 ms ago
uint8_t Reflectance_End(void){
    // write this as part of Lab 10
    uint8_t result;
    result = IRSENSOR->IN;
		IRLED(0);									//  6) Set P5.3 low (turn off IR LED, saving power)
    return result;
}
*/
int line_error(unsigned int first_sensor, unsigned int last_sensor) {
    const int W[] = {329, 235, 141, 47, -47, -141, -235, -329};
    if ((first_sensor == 256) || (last_sensor == 256)) return 0;
    if (last_sensor == first_sensor) {
        return W[last_sensor];
    } else {
        return ((W[first_sensor] + W[last_sensor])/2 - 47) + ((photo_array[first_sensor]-(data.threshold >> 3))*94/(photo_array[first_sensor]+photo_array[last_sensor]-((2 * data.threshold) >> 3))) ;
    }
}

void set_threshold(unsigned int threshold) {
    Reflectance_Init_with_Timer(threshold);
}
