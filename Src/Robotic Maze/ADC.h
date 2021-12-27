#ifndef ADC_H
#define ADC_H

#include "stdint.h"

//**************Low pass Digital filter**************
typedef struct {
	uint32_t Size;      // Size-point average, Size=1 to 512
	uint32_t x[256];   // two copies of MACQ
	uint32_t I1;        // index to oldest
//	volatile uint32_t start;
	volatile uint32_t Sum, data_ready;    // sum of the last Size samples
} LPF_t;

extern LPF_t LFP_center, LPF_left, LPF_right, LPF_battery;

//extern uint32_t LPFSize;      // Size-point average, Size=1 to 512
//extern volatile uint32_t LPFSum, ADC_data_ready;    // sum of the last Size samples

void ADC0_InitSWTriggerCh21(void);
void ADC0_Stop(void);

#endif
