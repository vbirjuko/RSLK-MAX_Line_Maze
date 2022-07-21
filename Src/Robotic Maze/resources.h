/*
 * resources.h
 *
 *  Created on: 2 янв. 2022 г.
 *      Author: wl
 */

#ifndef RESOURCES_H_
#define RESOURCES_H_

// Define robot configuration
// RSLK_MAX - mandatory. If not defined use classic RSLK configuration
// which may conflict with other modules
#define RSLK_MAX

// Used display.  Alter to SH1106 if necessary
#define SSD1306

// display orientation. Define if display show picture in wrong orientation
#define UPSIDEDOWN

// Looks mandatory too. Defines keyboard connection thru PCA9536. If not defined
// assume keys connected to MC port, what may conflict with other modules
#define PCA

// define if you have SPI FRAM connected to port P1.5-1.7 and P3.0 to make logging
// If not defined (or size equal 0) logging will make to SPI EEPROM which hold configuration
#define FRAM_SIZE       (256*1024)

// define if color sensor is placed on robots back.
// Defines when check dead end color: before turn (if not defined) or after (if defined).
#define COLOR_SENSOR_ON_BACK

//  what should show blinker LEDs.
// BLINKER_MOTOR - front yellow signalize motor STOP state - rear red if motor powered backward.
// BLINKER_SEGMENT - what speed is selected running on maze segment. Just for debug purposes.
#define BLINKER_SEGMENT

// SCANPERSECOND задаёт все временные задержки, а так же частоту запуска АЦП.
#define SCANPERSECOND 2000

// Ширина колеи робота для вычисления поворотов
#define TRACK_WIDE      (143)

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

// ********** EUSCI *******************
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
