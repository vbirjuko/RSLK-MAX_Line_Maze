#include "msp.h"
#include "TA3InputCapture.h"
#include "Tachometer.h"

#ifdef RSLK_MAX
#define ENC_Left_A 	(BITBAND_PERI(P10->IN, 5))
#define ENC_Left_B	(BITBAND_PERI(P5->IN, 2))
#define ENC_Right_A	(BITBAND_PERI(P10->IN, 4))
#define ENC_Right_B	(BITBAND_PERI(P5->IN, 0))
#else
#define ENC_Left_A 	(BITBAND_PERI(P8->IN, 2))
#define ENC_Left_B	(BITBAND_PERI(P9->IN, 2))
#define ENC_Right_A	(BITBAND_PERI(P10->IN, 4))
#define ENC_Right_B	(BITBAND_PERI(P10->IN, 5))
#endif

volatile Tach_stru_t TachLeft, TachRight;

volatile unsigned int RollOverLeft = 0, RollOverRight = 0;
volatile uint32_t LeftPeriodInc = 0, RightPeriodInc = 0, EventCountLeft = 0, EventCountRight = 0;


volatile int LeftSteps = 0, RightSteps = 0;

void tachometerRightInt(uint16_t currenttime){
    static uint32_t Tachometer_FirstRightTime, Tachometer_SecondRightTime = 0;
    Tachometer_FirstRightTime = Tachometer_SecondRightTime & 0xFFFF;
    Tachometer_SecondRightTime = currenttime + ((RollOverRight) << 16);
    if(ENC_Right_B){
// Encoder B is high, so this is a step forward
        RightSteps++;
        TachRight.Dir = FORWARD;
    } else {
// Encoder B is low, so this is a step backward
        RightSteps--;
        TachRight.Dir = REVERSE;
    }
    if (RollOverRight < 2) {
        TachRight.Period = Tachometer_SecondRightTime - Tachometer_FirstRightTime;
    } else {
        TachRight.Dir = STOPPED;
        TachRight.Period = 0x20000; //65535;
    }
    RollOverRight = 0;
#ifdef VARIANT1
    if (EventCountRight++ == 0) RightPeriodInc = TachRight.Period;
    else {
        RightPeriodInc += TachRight.Period;
        TachRight.Period = RightPeriodInc / EventCountRight;
    }
#else
#define LPF_FACTOR  3
    static unsigned int PeriodBuff[1 << LPF_FACTOR] = {0}, PeriodSum = 0, PeriodIndex=0;
    PeriodSum -= PeriodBuff[PeriodIndex];
    PeriodBuff[PeriodIndex] =TachRight.Period;
    PeriodSum +=  PeriodBuff[PeriodIndex++];
    PeriodIndex &= (1 << LPF_FACTOR) - 1;
    TachRight.Period = PeriodSum >> LPF_FACTOR;
#endif
}

void tachometerLeftInt(uint16_t currenttime){
    static uint32_t Tachometer_FirstLeftTime, Tachometer_SecondLeftTime = 0;
    Tachometer_FirstLeftTime = Tachometer_SecondLeftTime & 0xFFFF;
    Tachometer_SecondLeftTime = currenttime + ((RollOverLeft) << 16);
    if(ENC_Left_B){
// Encoder B is high, so this is a step forward
        LeftSteps++;
        TachLeft.Dir = FORWARD;
    } else {
// Encoder B is low, so this is a step backward
        LeftSteps--;
        TachLeft.Dir = REVERSE;
    }
    if (RollOverLeft < 2) {
        TachLeft.Period = Tachometer_SecondLeftTime - Tachometer_FirstLeftTime;
    } else {
        TachLeft.Dir = STOPPED;
        TachLeft.Period = 0x20000; //65535;
    }
    RollOverLeft = 0;
#ifdef VARIANT1
    if (EventCountLeft++ == 0) LeftPeriodInc = TachLeft.Period;
    else {
        LeftPeriodInc += TachLeft.Period;
        TachLeft.Period = LeftPeriodInc / EventCountLeft;
    }
#else
    static unsigned int PeriodBuff[1 << LPF_FACTOR] = {0}, PeriodSum = 0, PeriodIndex=0;
    PeriodSum -= PeriodBuff[PeriodIndex];
    PeriodBuff[PeriodIndex] = TachLeft.Period;
    PeriodSum +=  PeriodBuff[PeriodIndex++];
    PeriodIndex &= (1 << LPF_FACTOR) - 1;
    TachLeft.Period = PeriodSum >> LPF_FACTOR;
#endif
}

void tachometrInt(void) {
    if (++RollOverLeft  > 3) TachLeft.Dir  = STOPPED;
    if (++RollOverRight > 3) TachRight.Dir = STOPPED;
}


void tachometer_init() {
	TachLeft.Dir = TachRight.Dir = STOPPED;
	TachLeft.Period = TachRight.Period = 0x20000;
#ifdef RSLK_MAX
	// initialize P5.2 & P5.0 and make it GPIO
  P5->SEL0 	&= ~((1 << 2) | (1 << 0));				// ELB, ERB
  P5->SEL1 	&= ~((1 << 2) | (1 << 0));        // configure P5 as GPIO
  P5->DIR 	&= ~((1 << 2) | (1 << 0));        // make P5 in
	
	P10->SEL0 |=  ((1u << 4) | (1 << 5));				// ELA, ERA
	P10->SEL1 &= ~((1u << 4) | (1 << 5));       // configure P10.4 as TA3.CCI0A, P10.5 as TA3.CCI1A
	P10->DIR  &= ~((1u << 4) | (1 << 5));       // make P10.4 in
	LeftSteps = 0, RightSteps = 0;
	TimerA3Capture_Init(&tachometerRightInt, &tachometerLeftInt, &tachometrInt);
#else
  // initialize P9.2 and make it GPIO
  P9->SEL0 	&= ~(1 << 2);				// ELB
  P9->SEL1 	&= ~(1 << 2);               // configure P9.2 as GPIO
  P9->DIR 	&= ~(1 << 2);                // make P9.2 in
  // initialize P10.5 and make it GPIO
  P10->SEL0 &= ~(1 << 5);				// ERB
  P10->SEL1 &= ~(1 << 5);              // configure P10.5 as GPIO
  P10->DIR 	&= ~(1 << 5);               // make P10.5 in

	P10->SEL0 |= (1u << 4);				// ERA
	P10->SEL1 &= ~(1u << 4);                 // configure P10.4 as TA3CCP0
	P10->DIR  &= ~(1u << 4);                  // make P10.4 in

	P8->SEL0 |= (1u << 2);				// ELA
	P8->SEL1 &= ~(1u << 2);                 // configure 8.2 as TA3CCP2
	P8->DIR  &= ~(1u << 2);                  // make P8.2 in
	LeftSteps = 0, RightSteps = 0;
	TimerA3Capture_Init(&PeriodMeasure0, &PeriodMeasure2, &tachometrInt);
#endif
}

// ------------Tachometer_Get------------
// Get the most recent tachometer measurements.
// Input: leftTach   is pointer to store last measured tachometer period of left wheel (units of 0.083 usec)
//        leftDir    is pointer to store enumerated direction of last movement of left wheel
//        leftSteps  is pointer to store total number of forward steps measured for left wheel (360 steps per ~220 mm circumference)
//        rightTach  is pointer to store last measured tachometer period of right wheel (units of 0.083 usec)
//        rightDir   is pointer to store enumerated direction of last movement of right wheel
//        rightSteps is pointer to store total number of forward steps measured for right wheel (360 steps per ~220 mm circumference)
// Output: none
// Assumes: Tachometer_Init() has been called
// Assumes: Clock_Init48MHz() has been called
void Tachometer_Get(Tach_stru_t *leftTach, Tach_stru_t *rightTach) {
    __disable_irq();
    *rightTach = TachRight;
    *leftTach  = TachLeft;
    EventCountRight = 0;
    EventCountLeft  = 0;
    __enable_irq();
}
