/*
 * FRAM_Logging.c
 *
 *  Created on: 5 мая 2021 г.
 *      Author: wl
 */

#include "msp.h"
#include "driverlib.h"

typedef struct  {
	uint8_t 	command[4];
	uint16_t	data[128];
} spi_buffer_t;

#define FRAM_CS_PORT	P3
#define FRAM_CS_PIN		0

#define USE_DMA_FOR_SPI
 //configure P10.0 - P10.3 as primary module function
#define UCB0SOMI		(1u << 3)
#define UCB0SIMO    (1u << 2)
#define UCB0CLK     (1u << 1)
#define UCB0STE     (1u << 0)
#define NSS					(BITBAND_PERI(P1->OUT, 0))

static volatile unsigned int eeprom_write_busy = 0;
//static volatile uint8_t exchange_buffer[256+4];
//static volatile uint8_t *spi_buffer_tx_ptr, *spi_buffer_rx_ptr;

void FRAM_Logging_Init(void) {
	  EUSCI_B0->CTLW0 = 0x0001;             // hold the eUSCI module in reset mode
	      // configure UCA3CTLW0 for:
	      // bit15      UCCKPH = 1; clock data out on fall (clock into SSD1306 on rise)
	      // bit14      UCCKPL = 0; clock is low when inactive
	      // bit13      UCMSB = 1; MSB first
	      // bit12      UC7BIT = 0; 8-bit data
	      // bit11      UCMST = 1; master mode
	      // bits10-9   UCMODEx = 2; UCSTE active low
	      // bit8       UCSYNC = 1; synchronous mode
	      // bits7-6    UCSSELx = 2; eUSCI clock SMCLK
	      // bits5-2    reserved
	      // bit1       UCSTEM = 1; UCSTE pin enables slave
	      // bit0       UCSWRST = 1; reset enabled
//	  EUSCI_A3->CTLW0 = 0xAD83;
	  EUSCI_B0->CTLW0 = EUSCI_B_CTLW0_CKPH | EUSCI_B_CTLW0_MSB | EUSCI_B_CTLW0_MST |
			  	  	  	EUSCI_B_CTLW0_SYNC | EUSCI_B_CTLW0_SSEL__SMCLK |
						EUSCI_B_CTLW0_MODE_2 | EUSCI_B_CTLW0_STEM | EUSCI_B_CTLW0_SWRST;
	      // set the baud rate for the eUSCI which gets its clock from SMCLK
	      // Clock_Init48MHz() from ClockSystem.c sets SMCLK = HFXTCLK/4 = 12 MHz
	      // if the SMCLK is set to 12 MHz, divide by 3 for 4 MHz baud clock
	  EUSCI_A3->BRW = 0x1;
	      // modulation is not used in SPI mode, so clear UCA3MCTLW
	  EUSCI_A3->MCTLW = 0;
	  P1->SEL0 |=  ((1ul << 5)|(1ul << 6)|(1ul << 7)); 	//
	  P1->SEL1 &= ~((1ul << 5)|(1ul << 6)|(1ul << 7));	// configure P1.7, P1.6, and P1.5 as primary module function
	  FRAM_CS_PORT->SEL0 &= ~(1ul << FRAM_CS_PIN);
	  FRAM_CS_PORT->SEL1 &= ~(1ul << FRAM_CS_PIN);     			 			// configure P3.0 as GPIO (CS)
	  FRAM_CS_PORT->DIR  |=  (1ul << FRAM_CS_PIN);
	  FRAM_CS_PORT->OUT	 |=  (1ul << FRAM_CS_PIN);
	  EUSCI_B0->CTLW0 &= ~0x0001;           // enable eUSCI module
	  EUSCI_B0->IE &= ~0x0003;              // disable interrupts

	  NVIC_SetPriority(EUSCIB0_IRQn, 3);
	  NVIC_EnableIRQ(EUSCIB0_IRQn);

	  EUSCI_B0->IE = EUSCI_B_IE_RXIE;
}

#define NULL ((void* )0 )

volatile uint8_t *FRAM_send_ptr, *FRAM_recv_ptr;
volatile unsigned int write_in_progress = 0, FRAM_send_count = 0, FRAM_overrun = 0;

void EUSCIB0_IRQHandler(void) {
	switch (EUSCI_B0->IV) {
	case 2:	// data received
		if (FRAM_recv_ptr == NULL) {
		    EUSCI_B0->RXBUF;
		} else {
		    if (EUSCI_B0->STATW & EUSCI_B_STATW_OE) FRAM_overrun = 1;
		    *FRAM_recv_ptr++ = EUSCI_B0->RXBUF;
		}
		break;
	case 4: // data send empty
		EUSCI_B0->TXBUF = *FRAM_send_ptr++;
		if (--FRAM_send_count == 0) {
			EUSCI_B0->IE &= ~EUSCI_B_IE_TXIE;
		}
		write_in_progress = 1;
		break;
	}
}

unsigned int FRAM_log_Start(uint32_t FRAM_Addr) {
	static uint8_t FRAM_wr_buffer[] = {0x02, 0x00, 0x00, 0x00};

	if (BITBAND_PERI(FRAM_CS_PORT->OUT, FRAM_CS_PIN) == 0) return 1;
	write_in_progress = 0;
	FRAM_overrun = 0;
	BITBAND_PERI(FRAM_CS_PORT->OUT, FRAM_CS_PIN) = 0;// cs = 0
	FRAM_wr_buffer[0] = 0x06;
	FRAM_send_ptr = FRAM_wr_buffer;
	FRAM_recv_ptr = NULL;
	FRAM_send_count = 1;
	EUSCI_B0->IE |= EUSCI_B_IE_TXIE;
	while (FRAM_send_count || (EUSCI_B0->STATW & EUSCI_B_STATW_SPI_BUSY)) continue;
	write_in_progress = 0;
	BITBAND_PERI(FRAM_CS_PORT->OUT, FRAM_CS_PIN) = 1;// cs = 1

	FRAM_send_ptr = FRAM_wr_buffer;
	FRAM_recv_ptr = NULL;
	FRAM_wr_buffer[3] = (FRAM_Addr >> 0) & 0xFF;
	FRAM_wr_buffer[2] = (FRAM_Addr >> 8) & 0xFF;
	FRAM_wr_buffer[1] = (FRAM_Addr >> 16) & 0xFF;
	FRAM_wr_buffer[0] = 0x02;
	FRAM_send_count = sizeof(FRAM_wr_buffer);

	BITBAND_PERI(FRAM_CS_PORT->OUT, FRAM_CS_PIN) = 0;// cs = 0
	EUSCI_B0->IE |= EUSCI_B_IE_TXIE;
	while (FRAM_send_count || (EUSCI_B0->STATW & EUSCI_B_STATW_SPI_BUSY)) continue;

	return 0;
}

unsigned int FRAM_log_Stop(void) {
	if (write_in_progress == 0) return 1;
	while (FRAM_send_count || (EUSCI_B0->STATW & EUSCI_B_STATW_SPI_BUSY)) continue;
	write_in_progress = 0;
	BITBAND_PERI(FRAM_CS_PORT->OUT, FRAM_CS_PIN) = 1;// cs = 1
	return FRAM_overrun;
}

unsigned int FRAM_log_write(uint8_t *wr_data_ptr, uint8_t *rd_data_ptr, unsigned int wr_data_size) {
	if (write_in_progress == 0) return 1;
	while (FRAM_send_count || (EUSCI_B0->STATW & EUSCI_B_STATW_SPI_BUSY)) continue;
	write_in_progress = 0;
	FRAM_send_ptr = wr_data_ptr;
	FRAM_recv_ptr = rd_data_ptr;
	FRAM_send_count = wr_data_size;

	EUSCI_B0->IE |= EUSCI_B_IE_TXIE;
	while (write_in_progress == 0) continue;
	return 0;
}

unsigned int FRAM_wrsr(uint8_t block_protection) {
	static uint8_t FRAM_wr_buffer[] = {0x01, 0x00}, FRAM_rd_buffer[2];

	if (BITBAND_PERI(FRAM_CS_PORT->OUT, FRAM_CS_PIN) == 0) return 1;
	write_in_progress = 0;
	BITBAND_PERI(FRAM_CS_PORT->OUT, FRAM_CS_PIN) = 0;// cs = 0
	FRAM_wr_buffer[0] = 0x06;
	FRAM_send_ptr = FRAM_wr_buffer;
	FRAM_recv_ptr = FRAM_rd_buffer;
	FRAM_send_count = 1;
	EUSCI_B0->IE |= EUSCI_B_IE_TXIE;
	while (FRAM_send_count || (EUSCI_B0->STATW & EUSCI_B_STATW_SPI_BUSY)) continue;
	write_in_progress = 0;
	BITBAND_PERI(FRAM_CS_PORT->OUT, FRAM_CS_PIN) = 1;// cs = 1


	FRAM_send_ptr = FRAM_wr_buffer;
	FRAM_recv_ptr = FRAM_rd_buffer;
	FRAM_wr_buffer[1] = block_protection;
	FRAM_wr_buffer[0] = 0x01;
	FRAM_send_count = sizeof(FRAM_wr_buffer);

	BITBAND_PERI(FRAM_CS_PORT->OUT, FRAM_CS_PIN) = 0;// cs = 0
	EUSCI_B0->IE |= EUSCI_B_IE_TXIE;
	while (FRAM_send_count || (EUSCI_B0->STATW & EUSCI_B_STATW_SPI_BUSY)) continue;
	write_in_progress = 0;
	BITBAND_PERI(FRAM_CS_PORT->OUT, FRAM_CS_PIN) = 1;// cs = 1
	return 0;
}

unsigned int FRAM_rdsr(uint8_t *block_protection) {
	static uint8_t FRAM_wr_bp[] = {0x05, 0x00};

	if (BITBAND_PERI(FRAM_CS_PORT->OUT, FRAM_CS_PIN) == 0) return 1;
	write_in_progress = 0;
	BITBAND_PERI(FRAM_CS_PORT->OUT, FRAM_CS_PIN) = 0;// cs = 0
	FRAM_send_ptr = FRAM_wr_bp;
	FRAM_recv_ptr = FRAM_wr_bp;

	FRAM_wr_bp[0] = 0x05;
	FRAM_send_count = sizeof(FRAM_wr_bp);

	EUSCI_B0->IE |= EUSCI_B_IE_TXIE;
	while (FRAM_send_count || (EUSCI_B0->STATW & EUSCI_B_STATW_SPI_BUSY)) continue;
	write_in_progress = 0;
	BITBAND_PERI(FRAM_CS_PORT->OUT, FRAM_CS_PIN) = 1;// cs = 1
	*block_protection = FRAM_wr_bp[1];
	return 0;
}

unsigned int FRAM_read_Start(uint32_t FRAM_Addr) {
	static uint8_t FRAM_wr_buffer[] = {0x02, 0x00, 0x00, 0x00};

	if (BITBAND_PERI(FRAM_CS_PORT->OUT, FRAM_CS_PIN) == 0) return 1;
	write_in_progress = 0;

	FRAM_send_ptr = FRAM_wr_buffer;
	FRAM_recv_ptr = NULL;
	FRAM_wr_buffer[3] = (FRAM_Addr >> 0) & 0xFF;
	FRAM_wr_buffer[2] = (FRAM_Addr >> 8) & 0xFF;
	FRAM_wr_buffer[1] = (FRAM_Addr >> 16) & 0xFF;
	FRAM_wr_buffer[0] = 0x03;
	FRAM_send_count = sizeof(FRAM_wr_buffer);

	BITBAND_PERI(FRAM_CS_PORT->OUT, FRAM_CS_PIN) = 0;// cs = 0
	EUSCI_B0->IE |= EUSCI_B_IE_TXIE;
	return 0;
}


#ifdef UNUSED
void  spi_exchange(unsigned char * send_data_ptr, uint16_t count) {

    while (DMA_getChannelMode(1) != UDMA_MODE_STOP) continue;

    MAP_DMA_assignChannel(DMA_CH0_EUSCIB0TX0);
    MAP_DMA_assignChannel(DMA_CH1_EUSCIB0RX0);

    MAP_DMA_setChannelControl(DMA_CH1_EUSCIB0RX0 | UDMA_PRI_SELECT,
    UDMA_SIZE_8 | UDMA_SRC_INC_NONE | UDMA_DST_INC_8 | UDMA_ARB_1);

    MAP_DMA_setChannelControl(DMA_CH0_EUSCIB0TX0 | UDMA_PRI_SELECT,
    UDMA_SIZE_8 | UDMA_SRC_INC_8 | UDMA_DST_INC_NONE | UDMA_ARB_1);


    /* Setup the RX transfer characteristics & buffers */
    MAP_DMA_setChannelTransfer(DMA_CH1_EUSCIB0RX0 | UDMA_PRI_SELECT,
    UDMA_MODE_BASIC,
            (void *) MAP_SPI_getReceiveBufferAddressForDMA(EUSCI_B0_BASE),
            send_data_ptr,
            count);

    /* Setup the TX transfer characteristics & buffers */
    MAP_DMA_setChannelTransfer(DMA_CH0_EUSCIB0TX0 | UDMA_PRI_SELECT,
    UDMA_MODE_BASIC, (void *) send_data_ptr,
            (void *) MAP_SPI_getTransmitBufferAddressForDMA(EUSCI_B3_BASE),
            count);

    /* Assigning/Enabling Interrupts */
//    MAP_Interrupt_enableInterrupt(INT_DMA_INT2);
//    MAP_DMA_enableInterrupt(INT_DMA_INT2);

    MAP_DMA_enableChannel(1);
    MAP_DMA_enableChannel(0);


    while (DMA_getChannelMode(1) != UDMA_MODE_STOP) continue;
}
#endif
/*

#include "LaunchPad.h"
#include "UART0.h"

#define MEMORY_START    (0x30000)
#define ISALPHA(X)  (((X) >= ' ') && ((X) <= 0x7f))

uint8_t scratchpad[1024];
volatile unsigned int next = 0;
void FRAM_Read_test(void) {
    unsigned int ii, jj;

    LaunchPad_LED(0);
    FRAM_rdsr(scratchpad);
    UART0_OutString("StatusWord: ");
    UART0_OutUHex2(scratchpad[0]);
    UART0_OutString("\r\n");


    if (FRAM_read_Start(MEMORY_START)) LaunchPad_LED(1);
    for (ii = 0; ii < 256; ii++) {
            LaunchPad_Output(GREEN);
            if (FRAM_log_write(scratchpad, scratchpad, 16)) LaunchPad_LED(1);
            while (next == 0) continue;
            UART0_OutUHex(MEMORY_START+ii*16);
            UART0_OutString(" ");
            for (jj = 0; jj < 16; jj++) {
                UART0_OutUHex2(scratchpad[jj]);
                UART0_OutString(" ");
            }
            UART0_OutString(" ");
            for (jj = 0; jj < 16; jj++) {
                if (ISALPHA(scratchpad[jj])) UART0_OutChar(scratchpad[jj]);
                else                        UART0_OutChar('.');
            }
            UART0_OutString("\r\n");
            next = 0;
    }
    while (next == 0) continue;
    if (FRAM_log_Stop()) LaunchPad_Output(RED);
    else LaunchPad_Output(BLUE);
}
*/
