/*
 * UART_drv.h
 *
 *  Created on: 1 дек. 2020 г.
 *      Author: wl
 */

#ifndef UART_DRV_H_
#define UART_DRV_H_
#include "msp.h"

#define FIFO_BUFFER_SIZE    256

typedef struct {
    EUSCI_A_Type * EUSCI_BASE;
    volatile uint8_t RxFifoBuffer[FIFO_BUFFER_SIZE];
    volatile uint8_t TxFifoBuffer[FIFO_BUFFER_SIZE];
    volatile unsigned int RxReadIdx;
    volatile unsigned int RxWriteIdx;
    volatile unsigned int TxReadIdx;
    volatile unsigned int TxWriteIdx;
} uart_handle_t;

/**
 * @details   Initialize EUSCI_A for UART operation
 * @details   115,200 baud rate (assuming 12 MHz SMCLK clock),
 * @details   8 bit word length, no parity bits, one stop bit
 * @details   You must configure ports pins and define IRQ handler
 * @details   which calls this driver handler and enable it.
 * @param  huart pointer to handler structure
 * @return none
 * @brief  Initialize EUSCI Ax
 */
void UART_drv_Init(uart_handle_t * huart);

void UART_IRQHandler(uart_handle_t * huart);

/**
 * @details   Receive a character from EUSCI_A2 UART
 * @details   Interrupt synchronization,
 * @details   blocking, spin if RxFifo is empty
 * @param  huart pointer to handler structure
 * @return ASCII code of received data
 * @note   UART_Init must be called once prior
 * @brief  Receive byte into MSP432
 */
uint8_t UART_InChar(uart_handle_t * huart);

/**
 * @details   Transmit a character to EUSCI_A2 UART
 * @details   Busy-wait synchronization,
 * @details   blocking, wait for UART to be ready
 * @param  huart pointer to handler structure
 * @param  data is the ASCII code for data to send
 * @return none
 * @note   UART_Init must be called once prior
 * @brief  Transmit byte out of MSP432
 */
void UART_OutChar(uart_handle_t * huart, uint8_t data);

/**
 * @details   Transmit a string to EUSCI_A2 UART
 * @param  huart pointer to handler structure
 * @param  pt is pointer to null-terminated ASCII string to be transferred
 * @return none
 * @note   UART_Init must be called once prior
 * @brief  Transmit string out of MSP432
 */
void UART_OutString(uart_handle_t * huart, char *pt);


/**
 * @details   Check the receive FIFO from EUSCI_A2 UART
 * @details   non-blocking
 * @param  huart pointer to handler structure
 * @return number of characters in FIFO available for reading
 * @brief  Check status of receive FIFO
 */
uint32_t UART_InStatus(uart_handle_t * huart);

void UART_OutUDec(uart_handle_t * huart, uint32_t n);

#endif /* UART_DRV_H_ */
