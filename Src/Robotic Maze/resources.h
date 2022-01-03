/*
 * resources.h
 *
 *  Created on: 2 янв. 2022 г.
 *      Author: wl
 */

#ifndef RESOURCES_H_
#define RESOURCES_H_


// IRQ priorities

// ********** TIMER_A ****************
// Motor
#define TA0_0_Priority      3
#define TA0_N_Priority      3

// Reflectance
#define TA1_0_Priority      2
#define TA2_0_Priority      2
#define TA2_N_Priority      2

// Tachometer
#define TA3_0_Priority      1
#define TA3_N_Priority      1

// ********** TIMER_32 ****************
#define T32_INT1_Priority   4

// ********** EUSCI
// EUSCI_A0 UART0 (USB_CDC)
#define EUSCIA0_Priority    4
// EUSCI_A2 UART1 (BGX)
#define EUSCIA2_Priority    4
// EUSCI_B0 (SPI FRAM)
#define EUSCIB0_Priority    3
// EUSCI_B1 (i2c)
#define I2C_Priority        4


// ********** PORT *******************
#define PORT4_Priority      2


// ********** ADC ********************
#define ADC14_Priority      7
#endif /* RESOURCES_H_ */
