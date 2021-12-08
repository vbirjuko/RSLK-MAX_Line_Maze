#include "msp.h"
#include "configure.h"
#include "ADC.h"
#include "driverlib.h"

volatile uint32_t LPF_start = 0;
LPF_t LPF_center, LPF_left, LPF_right, LPF_battery;

void LPF__Init(LPF_t * instance, uint32_t initial, uint32_t size){ 
	int i;
  if(size>256) size=256; // max
  instance->Size = size;
  instance->I1 = size-1;
  instance->Sum = size*initial; // prime MACQ with initial data
  for(i=0; i<size; i++){
    instance->x[i] = initial;
  }
}

// calculate one filter output, called at sampling rate
// Input: new ADC data   Output: filter output
// y(n) = (x(n)+x(n-1)+...+x(n-Size-1)/Size
uint32_t LPF__Calc(LPF_t * instance, uint32_t newdata){
  if(instance->I1 == 0){
    instance->I1 = instance->Size-1;              // wrap
  } else{
    instance->I1--;                     // make room for data
  }
  instance->Sum = instance->Sum+newdata-instance->x[instance->I1];   // subtract oldest, add newest
  instance->x[instance->I1] = newdata;     // save new data
	LPF_battery.data_ready = 1;
  return instance->Sum/instance->Size;
}

void ADC0_InitSWTriggerCh21(void){
    // write this for Lab 15

    // Включим опорное напряжение 2,5в
    /* Setting reference voltage to 2.5  and enabling reference */
    MAP_REF_A_setReferenceVoltage(REF_A_VREF2_5V);
    MAP_REF_A_enableReferenceVoltage();
//
//    while (REF_A->CTL0 & REF_A_CTL0_GENBUSY) continue;
//    REF_A->CTL0 = REF_A_CTL0_VSEL_3 | REF_A_CTL0_ON;  // Vref = 2.5v
	
    ADC14->CTL0 &= ~ADC14_CTL0_ENC;                     // 2) ADC14ENC = 0 to allow programming
    while(ADC14->CTL0 & ADC14_CTL0_BUSY) continue;      // 3) wait for BUSY to be zero
    ADC14->CTL0 = ADC14_CTL0_PDIV_0  /* 31-30 ADC14PDIV  predivider,            00b = Predivide by 1 */
            | ADC14_CTL0_SHS_0      /* 29-27 ADC14SHSx  SHM source            000b = ADC14SC bit */
//          | ADC14_CTL0_SHS_6      /* 29-27 ADC14SHSx  TA2_C2 source            000b = ADC14SC bit */
            | ADC14_CTL0_SHP        /* 26    ADC14SHP   SHM pulse-mode          1b = SAMPCON the sampling timer */
            | ADC14_CTL0_DIV_0      /* 24-22 ADC14DIVx  clock divider         000b = /1 */
            | ADC14_CTL0_SSEL__SMCLK/* 21-19 ADC14SSELx clock source select   100b = SMCLK */
            | ADC14_CTL0_CONSEQ_3   /* 18-17 ADC14CONSEQx mode select          00b = Single-channel, single-conversion */
            | ADC14_CTL0_SHT0__32 | ADC14_CTL0_SHT1__32 /* ADC14SHTxx sample-and-hold time 0011b = 32 clocks*/
            | ADC14_CTL0_MSC         // multiple channel on one Rising edge
            | ADC14_CTL0_ON;        /* 4     ADC14ON    ADC14 on                1b = powered up */

    ADC14->CTL1 = ADC14_CTL1_RES__14BIT;    // 5) ADC14MEM0, 14-bit, ref on, regular power

    if (data.low_battery_level * data.volt_calibr < 0x1ffff) {
        ADC14->LO0 = data.low_battery_level * data.volt_calibr * 0x3fff / 10000;  // 6.3в - нижний порог батареи
        ADC14->HI0 = 0x3fff; // 10в - верхний порог батареи (недостижим)
    } else {
//        assert("ERROR: incorrect values in low_battery_level or/and volt_calibr\r\n");
    }
		
// battery (FB2 P8.4) A21
    P8->SEL1 |= (1u << 4);
    P8->SEL0 |= (1u << 4);
// IR Right (P9.0) A17;  Left (9.1) A16
    P9->SEL0 |= (1u << 0) | (1u << 1);
    P9->SEL1 |= (1u << 0) | (1u << 1);
//IR Center (P6.1) A14
    P6->SEL0 |= (1u << 1);
    P6->SEL1 |= (1u << 1);


    ADC14->MCTL[0] = ADC14_MCTLN_INCH_21 | ADC14_MCTLN_VRSEL_1 | ADC14_MCTLN_WINC; // питание
    ADC14->MCTL[1] = ADC14_MCTLN_INCH_14 ; 							// дальномер центральный
    ADC14->MCTL[2] = ADC14_MCTLN_INCH_16 ; 							// левый
    ADC14->MCTL[3] = ADC14_MCTLN_INCH_17 | ADC14_MCTLN_EOS 		 ; 	// правый

    ADC14->IER0 = (1u << 3); // 7) interrupts on MEM[3] write
    ADC14->IER1 = 0; // no interrupts on error
		
    ADC14->CTL0 |= ADC14_CTL0_ENC;         // 9) enable

    LPF_start = 1;
	
    NVIC_SetPriority(ADC14_IRQn, 7);
    NVIC_EnableIRQ(ADC14_IRQn);
}

void ADC0_Stop(void){
    ADC14->CTL0 &= ~ADC14_CTL0_ENC;                     // 2) ADC14ENC = 0 to allow programming
    NVIC_DisableIRQ(ADC14_IRQn);
}

// ADC14IFGR0 bit 0 is set when P4.1 = A12 conversion done
//                  cleared on read ADC14MEM0
// ADC14CLRIFGR0 bit 0, write 1 to clear flag
// ADC14IVx is 0x0C when ADC14MEM0 interrupt flag; Interrupt Flag: ADC14IFG0
// ADC14MEM0 14-bit conversion in bits 13-0 (31-16 undefined, 15-14 zero)
void ADC14_IRQHandler(void){
    // write this for Lab 15
	if (LPF_start) {
		LPF_start = 0;
		LPF__Init(&LPF_battery, ADC14->MEM[0], 256);
		LPF__Init(&LPF_center, 	ADC14->MEM[1], 256);
		LPF__Init(&LPF_left, 		ADC14->MEM[2], 256);
		LPF__Init(&LPF_right, 	ADC14->MEM[3], 256);
	} else {		
		LPF__Calc(&LPF_battery, ADC14->MEM[0]);      // 4) return result 0 to 16383
		LPF__Calc(&LPF_center, 	ADC14->MEM[1]);      // 4) return result 0 to 16383
		LPF__Calc(&LPF_left, 		ADC14->MEM[2]);      // 4) return result 0 to 16383
		LPF__Calc(&LPF_right, 	ADC14->MEM[3]);      // 4) return result 0 to 16383
	}
}
