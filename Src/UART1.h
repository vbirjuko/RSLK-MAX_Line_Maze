/*
 * UART1.h
 *
 *  Created on: 30 нояб. 2021 г.
 *      Author: wl
 */

#ifndef UART1_H_
#define UART1_H_


void UART1_Init(void);
char UART1_InChar(void);
//------------UART1_OutChar------------
// Output 8-bit to serial port
// Input: letter is an 8-bit ASCII character to be transferred
// Output: none
void UART1_OutChar(uint8_t letter);
//------------UART1_OutString------------
// Output String (NULL termination)
// Input: pointer to a NULL-terminated string to be transferred
// Output: none
void UART1_OutString(char *pt);

//-----------------------UART1_OutUDec-----------------------
// Output a 32-bit number in unsigned decimal format
// Input: 32-bit number to be transferred
// Output: none
// Variable format 1-10 digits with no space before or after
void UART1_OutUDec(uint32_t n);

#include "UART_drv.h"

extern uart_handle_t bgx_uart;

#define STREAM_PORT     P6
#define STREAM_PIN      0

#define STREAM_BIT  (BITBAND_PERI(STREAM_PORT->IN, STREAM_PIN))


#endif /* UART1_H_ */
