/*
 * UART_drv.c
 *
 *  Created on: 1 дек. 2020 г.
 *      Author: wl
 */
#include "msp.h"
#include "UART_drv.h"


void UART_drv_Init(uart_handle_t * huart) {
      huart->EUSCI_BASE->CTLW0 = EUSCI_A_CTLW0_SWRST;// 0x0001;                   // hold the USCI module in reset mode
      huart->EUSCI_BASE->CTLW0 = EUSCI_A_CTLW0_SSEL__SMCLK | EUSCI_A_CTLW0_SWRST;
     // set the baud rate
     // N = clock/baud rate = 12,000,000/115,200 = 104.1667
      huart->EUSCI_BASE->BRW = 104;           // UCBR = baud rate = int(N) = 104
      huart->EUSCI_BASE->MCTLW &= ~0xFFF1;    // clear first and second modulation stage bit fields
//      P1->SEL0 |= 0x0C;
//      P1->SEL1 &= ~0x0C;             // configure P1.3 and P1.2 as primary module function
      huart->RxReadIdx = 0;
      huart->RxWriteIdx = 0;
      huart->TxReadIdx = 0;
      huart->TxWriteIdx = 0;
      huart->EUSCI_BASE->CTLW0 &= ~EUSCI_A_CTLW0_SWRST;    // enable the USCI module
      huart->EUSCI_BASE->IE = EUSCI_A_IE_RXIE;       // disable interrupts (transmit ready, start received, transmit empty, receive full)
}

void RxFifoPut(uart_handle_t * huart, uint8_t data){
  if(((huart->RxWriteIdx+1)&(FIFO_BUFFER_SIZE - 1)) != huart->RxReadIdx) {
      huart->RxFifoBuffer[huart->RxWriteIdx] = data; // save in FIFO
      huart->RxWriteIdx = (huart->RxWriteIdx + 1)&(FIFO_BUFFER_SIZE - 1);         // next place to put
  }
}

int TxFifoPut(uart_handle_t * huart, uint8_t data){
  if(((huart->TxWriteIdx+1)&(FIFO_BUFFER_SIZE - 1)) != huart->TxReadIdx) {
      huart->TxFifoBuffer[huart->TxWriteIdx] = data; // save in FIFO
      huart->TxWriteIdx = (huart->TxWriteIdx + 1)&(FIFO_BUFFER_SIZE - 1);         // next place to put
      return 1;
  } else return 0;
}

int RxFifoGet(uart_handle_t * huart, uint8_t *datapt){
  if(huart->RxWriteIdx == huart->RxReadIdx) return 0;            // fail if empty
  *datapt = huart->RxFifoBuffer[huart->RxReadIdx];                 // retrieve data
  huart->RxReadIdx = (huart->RxReadIdx + 1) & (FIFO_BUFFER_SIZE - 1);         // next place to get
  return 1;
}

uint8_t TxFifoGet(uart_handle_t * huart){
  uint8_t result;
  if(huart->TxWriteIdx == huart->TxReadIdx) return 0;            // fail if empty
  result = huart->TxFifoBuffer[huart->TxReadIdx];                 // retrieve data
  huart->TxReadIdx = (huart->TxReadIdx + 1) & (FIFO_BUFFER_SIZE - 1);         // next place to get
  return result;
}

void UART_IRQHandler(uart_handle_t * huart){
    switch (huart->EUSCI_BASE->IV) {
        case 02:
            RxFifoPut(huart, (uint8_t)huart->EUSCI_BASE->RXBUF);// clears UCRXIFG
            break;

        case 04:
            huart->EUSCI_BASE->TXBUF = TxFifoGet(huart);
            if (huart->TxReadIdx == huart->TxWriteIdx) huart->EUSCI_BASE->IE &= ~EUSCI_A_IE_TXIE;
            break;
        case 0x06: // Vector 6: UCSTTIFG
            break;
        case 0x08: // Vector 8: UCTXCPTIFG
            break;
        default:
            break;
    }
}

//------------UART_InStatus------------
// Returns how much data available for reading
// Input: none
// Output: number of bytes in receive FIFO
unsigned int UART_InStatus(uart_handle_t * huart){
    return ((huart->RxReadIdx - huart->RxWriteIdx) & (FIFO_BUFFER_SIZE - 1));
}

//------------UART_InChar------------
// Wait for new serial port input, interrupt synchronization
// Input: none
// Output: an 8-bit byte received
// spin if RxFifo is empty
uint8_t UART_InChar(uart_handle_t * huart){
  uint32_t letter;
  while(RxFifoGet(huart, (uint8_t *)&letter) == 0) continue;
  return(letter);
}

///------------UART_OutChar------------
// Output 8-bit to serial port
// Input: letter is an 8-bit data to be transferred
// Output: none
void UART_OutChar(uart_handle_t * huart, uint8_t data){
    while (TxFifoPut(huart, data) == 0) continue;
    huart->EUSCI_BASE->IE |= EUSCI_A_IE_TXIE;
}

//------------UART_OutString------------
// Output String (NULL termination)
// Input: pointer to a NULL-terminated string to be transferred
// Output: none
void UART_OutString(uart_handle_t * huart, char *pt){
  while(*pt){
    UART_OutChar(huart, *pt++);
  }
}

//-----------------------UART_OutUDec-----------------------
// Output a 32-bit number in unsigned decimal format
// Input: 32-bit number to be transferred
// Output: none
// Variable format 1-10 digits with no space before or after
void UART_OutUDec(uart_handle_t * huart, uint32_t n){
// This function uses recursion to convert decimal number
//   of unspecified length as an ASCII string
  if(n >= 10){
    UART_OutUDec(huart, n/10);
    n = n%10;
  }
  UART_OutChar(huart, n+'0'); /* n is between 0 and 9 */
}
