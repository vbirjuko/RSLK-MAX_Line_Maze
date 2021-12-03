#include <msp.h>
#include "configure.h"
#include "Tachometer.h"
#include "Blinker.h"

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

void Motor_PWM (int16_t left, int16_t right) {
//    SLEEP_PORT->OUT |=  ((1u << LEFT)|(1u << RIGHT));
//    BITBAND_PERI(DIRECTION_PORT->OUT, DIRECTION_LEFT) = (left < 0) ? 1 : 0;
//    BITBAND_PERI(DIRECTION_PORT->OUT, DIRECTION_RIGHT) = (right < 0) ? 1 : 0;
    if (left < 0) {
        BITBAND_PERI(DIRECTION_PORT->OUT, DIRECTION_LEFT) = 1;
        TIMER_A0->CCR[4] = -left;
    } else {
        BITBAND_PERI(DIRECTION_PORT->OUT, DIRECTION_LEFT) = 0;
        TIMER_A0->CCR[4] = left;
    }
    if (right < 0) {
        BITBAND_PERI(DIRECTION_PORT->OUT, DIRECTION_RIGHT) = 1;
        TIMER_A0->CCR[3] = -right;
    } else {
        BITBAND_PERI(DIRECTION_PORT->OUT, DIRECTION_RIGHT) = 0;
        TIMER_A0->CCR[3] = right;
    }
//
//    TIMER_A0->CCR[3] = (uint16_t)right;
//    TIMER_A0->CCR[4] = (uint16_t)left;
}

Tach_stru_t Left, Right;
int XstartL  = 0, XstartR = 0;

void Controller(void){

// write this as part of Lab 17
    Tach_stru_t Left, Right;
    static int XprimeL, ErrorL, UIL, UPR, UR;
    static int XprimeR, ErrorR, UIR, UPL, UL;
		unsigned int backlight = 0;

		Tachometer_Get(&Left, &Right);
		XprimeL = (Left.Period)  ? 200000000/Left.Period  : 200000000/65536;
		XprimeR = (Right.Period) ? 200000000/Right.Period : 200000000/65536;
    if (Left.Dir == REVERSE) XprimeL = -XprimeL;
    if (Right.Dir == REVERSE) XprimeR = -XprimeR;

    ErrorL = XstartL - XprimeL;
    ErrorR = XstartR - XprimeR;

    UIL += (data.motor_Kint*ErrorL) >> 10;
    UIR += (data.motor_Kint*ErrorR) >> 10;

    if (UIL > 14999) UIL = 14999;
    else if (UIL < -14999) UIL = -14999;

    if (UIR > 14999) UIR = 14999;
    else if (UIR < -14999) UIR = -14999;

    UPL = (data.motor_Kprop*ErrorL) >> 10;
    UPR = (data.motor_Kprop*ErrorR) >> 10;

    UL = UIL + UPL;
    UR = UIR + UPR;

    if (UL > 14999) UL = 14999;
    else if (UL < -14999) UL = -14999;

    if (UR > 14999) UR = 14999;
    else if (UR < -14999) UR = -14999;

    if (XstartL == 0 && Left.Dir  == STOPPED) UIL = UL = 0, backlight |= FR_LEFT;
    if (XstartR == 0 && Right.Dir == STOPPED) UIR = UR = 0, backlight |= FR_RGHT;

		if (UL < 0) backlight |= BK_LEFT;
		if (UR < 0) backlight |= BK_RGHT;
#ifdef BLINKER_MOTOR
		Blinker_Output(backlight);
#endif

		Motor_PWM(UL, UR);
}

void Motor_Init(void){
	tachometer_init();
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

  TIMER_A0->CCTL[0] = 0x0080 | TIMER_A_CCTLN_CCIE; // CCI0 toggle, interrupt enable
  TIMER_A0->CCR[0] = 15000; // Period is 2*period*4*83.33ns is 0.666*period -> 10ms
  TIMER_A0->EX0 = 0x0000; // divide by 1
  TIMER_A0->CCTL[3] = 0x0040; // CCR1 toggle/reset
  TIMER_A0->CCR[3] = 0; // CCR1 duty cycle is duty1/period
  TIMER_A0->CCTL[4] = 0x0040; // CCR2 toggle/reset
  TIMER_A0->CCR[4] = 0; // CCR2 duty cycle is duty2/period
												// SMCLK=12MHz, divide by 4, up-down mode, interrupt enable 
  TIMER_A0->CTL = TIMER_A_CTL_TASSEL_2 | TIMER_A_CTL_ID__4 | TIMER_A_CTL_MC__UPDOWN | TIMER_A_CTL_IE;
  // bit mode
  // 9-8 10 TASSEL, SMCLK=12MHz
  // 7-6 11 ID, divide by 8
  // 5-4 11 MC, up-down mode
  // 2 0 TACLR, no clear
  // 1 0 TAIE, no interrupt
  // 0 TAIFG
  NVIC_SetPriority(TA0_0_IRQn, 3);
  NVIC_SetPriority(TA0_N_IRQn, 3);
  NVIC_EnableIRQ(TA0_0_IRQn);
  NVIC_EnableIRQ(TA0_N_IRQn);
}

void TA0_0_IRQHandler(void){
  // write this as part of lab 16
    TIMER_A0->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG; //0x0001;             // acknowledge capture/compare interrupt 0
    Controller();         // execute user task
}

void TA0_N_IRQHandler(void){
  // write this as part of lab 16
    switch(TIMER_A0->IV) {
    case 2:
        TIMER_A0->CCTL[1] &= ~TIMER_A_CCTLN_CCIFG;
        break;

    case 4:
        TIMER_A0->CCTL[2] &= ~TIMER_A_CCTLN_CCIFG;
        break;

    case 6:
        TIMER_A0->CCTL[3] &= ~TIMER_A_CCTLN_CCIFG;
        break;

    case 8:
        TIMER_A0->CCTL[4] &= ~TIMER_A_CCTLN_CCIFG;
        break;

    case 0x0e:
        TIMER_A0->CTL &= ~TIMER_A_CTL_IFG;
        Controller();         // execute user task
        break;

    default:
        break;
//        TIMER_A0->CCTL[5] &= ~TIMER_A_CCTLN_CCIFG;
//        TIMER_A0->CCTL[6] &= ~TIMER_A_CCTLN_CCIFG;
    }
}

void Motor_Speed(int16_t left, int16_t right) {
	XstartL = left;
	XstartR = right;
}

// ------------Motor_Stop------------
void Motor_Stop(void){
	XstartL = 0;
	XstartR = 0;
}

void Motor_Enable(void) {
  SLEEP_PORT->OUT  |=  (1u << LEFT)|(1u << RIGHT);
}

void Motor_Disable(void) {
	SLEEP_PORT->OUT  &= ~((1u << LEFT)|(1u << RIGHT));
}
