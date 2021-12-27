// UART1.c
// Runs on MSP432
// Use UCA2 to implement bidirectional data transfer to and from a
// CC2650 BLE module, uses interrupts for receive and busy-wait for transmit

// Daniel Valvano
// May 24, 2016

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


// UCA2RXD (VCP receive) connected to P3.2
// UCA2TXD (VCP transmit) connected to P3.3
// J1.3  from Bluetooth (DIO3_TXD) to LaunchPad (UART RxD){MSP432 P3.2}
// J1.4  from LaunchPad to Bluetooth (DIO2_RXD) (UART TxD){MSP432 P3.3}

#include <stdint.h>
#include "UART1.h"
#include "msp.h"
#include "UART_drv.h"

uart_handle_t bgx_uart = {
      EUSCI_A2,
      {0}, {0},
      0, 0, 0, 0
};

//------------UART1_Init------------
// Initialize the UART for 115,200 baud rate (assuming 12 MHz SMCLK clock),
// 8 bit word length, no parity bits, one stop bit
// Input: none
// Output: none
void UART1_Init(void){
    STREAM_PORT->SEL0   &= ~(1ul << STREAM_PIN);
    STREAM_PORT->SEL1   &= ~(1ul << STREAM_PIN);
    STREAM_PORT->DIR    &= ~(1ul << STREAM_PIN);
    UART_drv_Init(&bgx_uart);
    P3->SEL0 |= 0x0C;
    P3->SEL1 &= ~0x0C;          // configure P3.3 and P3.2 as primary module function
    NVIC_EnableIRQ(EUSCIA2_IRQn);
}

void EUSCIA2_IRQHandler(void){
    UART_IRQHandler(&bgx_uart);
}

char UART1_InChar(void){
  return(UART_InChar(&bgx_uart));
}

//------------UART1_OutChar------------
// Output 8-bit to serial port
// Input: letter is an 8-bit ASCII character to be transferred
// Output: none
void UART1_OutChar(uint8_t letter){
    UART_OutChar(&bgx_uart, letter);
}

//------------UART1_OutString------------
// Output String (NULL termination)
// Input: pointer to a NULL-terminated string to be transferred
// Output: none
void UART1_OutString(char *pt){
  while(*pt){
    UART1_OutChar(*pt);
    pt++;
  }
}

//-----------------------UART1_OutUDec-----------------------
// Output a 32-bit number in unsigned decimal format
// Input: 32-bit number to be transferred
// Output: none
// Variable format 1-10 digits with no space before or after
void UART1_OutUDec(uint32_t n){
// This function uses recursion to convert decimal number
//   of unspecified length as an ASCII string
  if(n >= 10){
    UART1_OutUDec(n/10);
    n = n%10;
  }
  UART1_OutChar(n+'0'); /* n is between 0 and 9 */
}

#ifdef VALVANOCODE
#define FIFOSIZE   256       // size of the FIFOs (must be power of 2)
#define FIFOSUCCESS 1        // return value on success
#define FIFOFAIL    0        // return value on failure
volatile uint32_t RxPutI, TxPutI;      // should be 0 to SIZE-1
volatile uint32_t RxGetI, TxGetI;      // should be 0 to SIZE-1 
uint32_t RxFifoLost, TxFifoLost;  // should be 0 
volatile uint8_t RxFIFO[FIFOSIZE], TxFIFO[FIFOSIZE];
void RxFifo_Init(void){
  RxPutI = RxGetI = 0;                      // empty
  RxFifoLost = 0; // occurs on overflow
	TxPutI = TxGetI = 0;
	TxFifoLost = 0;
}

static uint32_t Messageindexb;
static char Messageb[8];

int RxFifo_Put(uint8_t data){
  if(((RxPutI+1)&(FIFOSIZE-1)) == RxGetI){
    RxFifoLost++;
    return FIFOFAIL; // fail if full  
  }    
  RxFIFO[RxPutI] = data;                    // save in FIFO
  RxPutI = (RxPutI+1)&(FIFOSIZE-1);         // next place to put
  return FIFOSUCCESS;
}

int TxFifo_Put(uint8_t data){
  if(((TxPutI+1)&(FIFOSIZE-1)) == TxGetI){
    TxFifoLost++;
    return FIFOFAIL; // fail if full  
  }    
  TxFIFO[TxPutI] = data;                    // save in FIFO
  TxPutI = (TxPutI+1)&(FIFOSIZE-1);         // next place to put
  return FIFOSUCCESS;
}

int RxFifo_Get(uint8_t *datapt){ 
  if(RxPutI == RxGetI) return 0;            // fail if empty
  *datapt = RxFIFO[RxGetI];                 // retrieve data
  RxGetI = (RxGetI+1)&(FIFOSIZE-1);         // next place to get
  return FIFOSUCCESS; 
}

uint8_t TxFifo_Get(void){
	uint8_t result;
  if(TxPutI == TxGetI) return 0;            // fail if empty
  result = TxFIFO[TxGetI];                 // retrieve data
  TxGetI = (TxGetI+1)&(FIFOSIZE-1);         // next place to get
  return result; 
}

//------------UART1_InStatus------------
// Returns how much data available for reading
// Input: none
// Output: number of bytes in receive FIFO
uint32_t UART1_InStatus(void){  
 return ((RxPutI - RxGetI)&(FIFOSIZE-1));  
}
//------------UART1_Init------------
// Initialize the UART for 115,200 baud rate (assuming 12 MHz SMCLK clock),
// 8 bit word length, no parity bits, one stop bit
// Input: none
// Output: none
void UART1_Init(void){
  RxFifo_Init();              // initialize FIFOs
  P6->SEL0 &= ~(STREAM_COMMAND_MASK);
  P6->SEL1 &= ~(STREAM_COMMAND_MASK);
  P6->DIR  &= ~(STREAM_COMMAND_MASK);

	
  EUSCI_A2->CTLW0 = 0x0001;         // hold the USCI module in reset mode
  // bit15=0,      no parity bits
  // bit14=x,      not used when parity is disabled
  // bit13=0,      LSB first
  // bit12=0,      8-bit data length
  // bit11=0,      1 stop bit
  // bits10-8=000, asynchronous UART mode
  // bits7-6=11,   clock source to SMCLK
  // bit5=0,       reject erroneous characters and do not set flag
  // bit4=0,       do not set flag for break characters
  // bit3=0,       not dormant
  // bit2=0,       transmit data, not address (not used here)
  // bit1=0,       do not transmit break (not used here)
  // bit0=1,       hold logic in reset state while configuring
  EUSCI_A2->CTLW0 = 0x00C1;
                              // set the baud rate
                              // N = clock/baud rate = 12,000,000/115,200 = 104.1667
  EUSCI_A2->BRW = 104;        // UCBR = baud rate = int(N) = 104

  EUSCI_A2->MCTLW = 0x0000;   // clear first and second modulation stage bit fields
// since TxFifo is empty, we initially disarm interrupts on UCTXIFG, but arm it on OutChar
  P3->SEL0 |= 0x0C;
  P3->SEL1 &= ~0x0C;          // configure P3.3 and P3.2 as primary module function
  NVIC->IP[4] = (NVIC->IP[4]&0xFF00FFFF)|0x00400000; // priority 2
  NVIC->ISER[0] = 0x00040000; // enable interrupt 18 in NVIC
  EUSCI_A2->CTLW0 &= ~0x0001; // enable the USCI module
                              // enable interrupts on receive full
  EUSCI_A2->IE = 0x0001;      // disable interrupts on transmit empty, start, complete
//	NVIC_EnableIRQ();
}


//------------UART1_InChar------------
// Wait for new serial port input, interrupt synchronization
// Input: none
// Output: an 8-bit byte received
// spin if RxFifo is empty
uint8_t UART1_InChar(void){
  uint32_t letter;
  while(RxFifo_Get((uint8_t *)&letter) == FIFOFAIL) continue;
  return(letter);
}

///------------UART1_OutChar------------
// Output 8-bit to serial port, busy-wait
// Input: letter is an 8-bit data to be transferred
// Output: none
/*
void UART1_OutChar(uint8_t data){
  while((EUSCI_A2->IFG&0x02) == 0);
  EUSCI_A2->TXBUF = data;
}
*/
void UART1_OutChar(uint8_t data){
  while (TxFifo_Put(data) != FIFOSUCCESS) continue;
	EUSCI_A2->IE |= EUSCI_A_IE_TXIE;
}

// interrupt 18 occurs on :
// UCRXIFG RX data register is full
// vector at 0x00000088 in startup_msp432.s
/*
void EUSCIA2_IRQHandler(void){
  if(EUSCI_A2->IFG&0x01){             // RX data register full
    RxFifo_Put((uint8_t)EUSCI_A2->RXBUF);// clears UCRXIFG
  } 
}
*/
void EUSCIA2_IRQHandler(void){
	switch (EUSCI_A2->IV) {
		case 02:
			RxFifo_Put((uint8_t)EUSCI_A2->RXBUF);// clears UCRXIFG
			break;
		
		case 04:
			EUSCI_A2->TXBUF = TxFifo_Get();
			if (TxGetI == TxPutI) EUSCI_A2->IE &= ~EUSCI_A_IE_TXIE;
			break;
		case 0x06: // Vector 6: UCSTTIFG
			break;
		case 0x08: // Vector 8: UCTXCPTIFG
			break;
		default: 
			break;
	}
}

//------------UART1_OutString------------
// Output String (NULL termination)
// Input: pointer to a NULL-terminated string to be transferred
// Output: none
void UART1_OutString(char *pt){
  while(*pt){
    UART1_OutChar(*pt);
    pt++;
  }
}
//------------UART1_FinishOutput------------
// Wait for all transmission to finish
// Input: none
// Output: none
void UART1_FinishOutput(void){
  // Wait for entire tx message to be sent
  while((EUSCI_A2->IFG&0x02) == 0) continue;
}


//-----------------------UART0_OutUDec-----------------------
// Output a 32-bit number in unsigned decimal format
// Input: 32-bit number to be transferred
// Output: none
// Variable format 1-10 digits with no space before or after
void UART1_OutUDec(uint32_t n){
// This function uses recursion to convert decimal number
//   of unspecified length as an ASCII string
  if(n >= 10){
    UART1_OutUDec(n/10);
    n = n%10;
  }
  UART1_OutChar(n+'0'); /* n is between 0 and 9 */
}

void static fillmessageb(uint32_t n){
// This function uses recursion to convert decimal number
//   of unspecified length as an ASCII string
  if(n >= 10){
    fillmessageb(n/10);
    n = n%10;
  }
  Messageb[Messageindexb] = (n+'0'); /* n is between 0 and 9 */
  if(Messageindexb<7)Messageindexb++;
}

static void fillmessage4b(uint32_t n){
  if(n>=1000){  // 1000 to 9999
    Messageindexb = 0;
  } else if(n>=100){  // 100 to 999
    Messageb[0] = ' ';
    Messageindexb = 1;
  }else if(n>=10){ //
    Messageb[0] = ' '; /* n is between 10 and 99 */
    Messageb[1] = ' ';
    Messageindexb = 2;
  }else{
    Messageb[0] = ' '; /* n is between 0 and 9 */
    Messageb[1] = ' ';
    Messageb[2] = ' ';
    Messageindexb = 3;
  }
  fillmessageb(n);
}
static void fillmessage5b(uint32_t n){
  if(n>99999)n=99999;
  if(n>=10000){  // 10000 to 99999
    Messageindexb = 0;
  } else if(n>=1000){  // 1000 to 9999
    Messageb[0] = ' ';
    Messageindexb = 1;
  }else if(n>=100){  // 100 to 999
    Messageb[0] = ' ';
    Messageb[1] = ' ';
    Messageindexb = 2;
  }else if(n>=10){ //
    Messageb[0] = ' '; /* n is between 10 and 99 */
    Messageb[1] = ' ';
    Messageb[2] = ' ';
    Messageindexb = 3;
  }else{
    Messageb[0] = ' '; /* n is between 0 and 9 */
    Messageb[1] = ' ';
    Messageb[2] = ' ';
    Messageb[3] = ' ';
    Messageindexb = 4;
  }
  fillmessageb(n);
}

//-----------------------UART0_OutUDec4-----------------------
// Output a 32-bit number in unsigned decimal format
// Input: 32-bit number to be transferred
// Output: none
// Fixed format 4 digits with no space before or after
void UART1_OutUDec4(uint32_t n){
  if(n>9999){
    UART1_OutString("****");
  }else{
    Messageindexb = 0;
    fillmessage4b(n);
    Messageb[Messageindexb] = 0; // terminate
    UART1_OutString(Messageb);
  }
}
//-----------------------UART0_OutUDec5-----------------------
// Output a 32-bit number in unsigned decimal format
// Input: 32-bit number to be transferred
// Output: none
// Fixed format 5 digits with no space before or after
void UART1_OutUDec5(uint32_t n){
  if(n>99999){
    UART1_OutString("*****");
  }else{
    Messageindexb = 0;
    fillmessage5b(n);
    Messageb[Messageindexb] = 0; // terminate
    UART1_OutString(Messageb);
  }
}
#endif
