/*
 * FRAM_Logging.c
 *
 *  Created on: 5 мая 2021 г.
 *      Author: wl
 */

#include "msp.h"
#include "driverlib.h"
#include "FRAM_Logging.h"
#include "resources.h"

#define USE_FRAM_DMA

#define FRAM_CS_PORT	P3
#define FRAM_CS_PIN		0

//#define USE_DMA_FOR_SPI
 //configure P1.5 - P1.7 as primary module function
#define UCB0SOMI	(1u << 5)
#define UCB0SIMO    (1u << 6)
#define UCB0CLK     (1u << 7)

#define UCB0NSS					(BITBAND_PERI(FRAM_CS_PORT->OUT, FRAM_CS_PIN))

static volatile unsigned int fram_write_busy = 0;
//static volatile uint8_t exchange_buffer[256+4];
//static uint8_t *volatile spi_buffer_tx_ptr, *volatile spi_buffer_rx_ptr;

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
	  P1->SEL0 |=  (UCB0SOMI|UCB0SIMO|UCB0CLK); 	//
	  P1->SEL1 &= ~(UCB0SOMI|UCB0SIMO|UCB0CLK);	// configure P1.7, P1.6, and P1.5 as primary module function
	  FRAM_CS_PORT->SEL0 &= ~(1ul << FRAM_CS_PIN);
	  FRAM_CS_PORT->SEL1 &= ~(1ul << FRAM_CS_PIN);     			 			// configure P3.0 as GPIO (CS)
	  FRAM_CS_PORT->DIR  |=  (1ul << FRAM_CS_PIN);
	  FRAM_CS_PORT->OUT	 |=  (1ul << FRAM_CS_PIN);
	  EUSCI_B0->CTLW0 &= ~0x0001;           // enable eUSCI module
	  EUSCI_B0->IE &= ~0x0003;              // disable interrupts

	  NVIC_SetPriority(EUSCIB0_IRQn, EUSCIB0_Priority);
	  NVIC_EnableIRQ(EUSCIB0_IRQn);

//	  EUSCI_B0->IE = EUSCI_B_IE_RXIE;
}

#ifndef USE_FRAM_DMA
#define NULL ((void* )0 )

uint8_t * volatile FRAM_send_ptr, * volatile FRAM_recv_ptr;
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
	UCB0NSS = 0;// cs = 0
	FRAM_wr_buffer[0] = 0x06;
	FRAM_send_ptr = FRAM_wr_buffer;
	FRAM_recv_ptr = NULL;
	FRAM_send_count = 1;
	EUSCI_B0->IE |= EUSCI_B_IE_TXIE;
	while (FRAM_send_count || (EUSCI_B0->STATW & EUSCI_B_STATW_SPI_BUSY)) continue;
	write_in_progress = 0;
	UCB0NSS = 1;// cs = 1

	FRAM_send_ptr = FRAM_wr_buffer;
	FRAM_recv_ptr = NULL;
	FRAM_wr_buffer[3] = (FRAM_Addr >> 0) & 0xFF;
	FRAM_wr_buffer[2] = (FRAM_Addr >> 8) & 0xFF;
	FRAM_wr_buffer[1] = (FRAM_Addr >> 16) & 0xFF;
	FRAM_wr_buffer[0] = 0x02;
	FRAM_send_count = sizeof(FRAM_wr_buffer);

	UCB0NSS = 0;// cs = 0
	EUSCI_B0->IE |= EUSCI_B_IE_TXIE;
	while (FRAM_send_count || (EUSCI_B0->STATW & EUSCI_B_STATW_SPI_BUSY)) continue;

	return 0;
}

unsigned int FRAM_log_Stop(void) {
	if (write_in_progress == 0) return 1;
	while (FRAM_send_count || (EUSCI_B0->STATW & EUSCI_B_STATW_SPI_BUSY)) continue;
	write_in_progress = 0;
	UCB0NSS = 1;// cs = 1
    EUSCI_B0->IE &= ~(EUSCI_B_IE_RXIE | EUSCI_B_IE_TXIE);

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

void FRAM_wait_EOT(void) {
    while (FRAM_send_count || (EUSCI_B0->STATW & EUSCI_B_STATW_SPI_BUSY)) continue;
}

unsigned int FRAM_wrsr(uint8_t block_protection) {
	static uint8_t FRAM_wr_buffer[] = {0x01, 0x00};

	if (UCB0NSS == 0) return 1;
	write_in_progress = 0;
	UCB0NSS = 0;// cs = 0
	FRAM_wr_buffer[0] = 0x06;
	FRAM_send_ptr = FRAM_wr_buffer;
	FRAM_recv_ptr = NULL;
	FRAM_send_count = 1;
	EUSCI_B0->IE |= EUSCI_B_IE_TXIE;
	while (FRAM_send_count || (EUSCI_B0->STATW & EUSCI_B_STATW_SPI_BUSY)) continue;
	write_in_progress = 0;
	UCB0NSS = 1;// cs = 1


	FRAM_send_ptr = FRAM_wr_buffer;
	FRAM_recv_ptr = NULL;
	FRAM_wr_buffer[1] = block_protection;
	FRAM_wr_buffer[0] = 0x01;
	FRAM_send_count = sizeof(FRAM_wr_buffer);

	UCB0NSS = 0;// cs = 0
	EUSCI_B0->IE |= EUSCI_B_IE_TXIE;
	while (FRAM_send_count || (EUSCI_B0->STATW & EUSCI_B_STATW_SPI_BUSY)) continue;
	write_in_progress = 0;
	UCB0NSS = 1;// cs = 1
	return 0;
}

unsigned int FRAM_rdsr(uint8_t *block_protection) {
	static uint8_t FRAM_wr_bp[] = {0x05, 0x00};

	if (UCB0NSS == 0) return 1;
	write_in_progress = 0;
	UCB0NSS = 0;// cs = 0
	FRAM_send_ptr = FRAM_wr_bp;
	FRAM_recv_ptr = FRAM_wr_bp;

	FRAM_wr_bp[0] = 0x05;
	FRAM_send_count = sizeof(FRAM_wr_bp);

	EUSCI_B0->IE |= EUSCI_B_IE_TXIE;
	while (FRAM_send_count || (EUSCI_B0->STATW & EUSCI_B_STATW_SPI_BUSY)) continue;
	write_in_progress = 0;
	UCB0NSS = 1;// cs = 1
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



	EUSCI_B0->RXBUF;    // сброс ошибок чтения и запроса прерывания?

	UCB0NSS = 0;// cs = 0
	EUSCI_B0->IE |= (EUSCI_B_IE_TXIE | EUSCI_B_IE_RXIE);
	return 0;
}

#else

/* Completion interrupt for DMA */
void DMA_INT0_IRQHandler(void)
{
    MAP_DMA_disableChannel(DMA_CHANNEL_0);
    MAP_DMA_disableChannel(DMA_CHANNEL_1);
    fram_write_busy = 0;
}

unsigned int FRAM_dma_log_write(uint8_t *wr_data_ptr, unsigned int wr_data_size) {
    if (UCB0NSS != 0) return 1;

    while (fram_write_busy) continue;

    MAP_DMA_assignChannel(DMA_CH0_EUSCIB0TX0);

    MAP_DMA_setChannelControl(DMA_CH0_EUSCIB0TX0 | UDMA_PRI_SELECT,
    UDMA_SIZE_8 | UDMA_SRC_INC_8 | UDMA_DST_INC_NONE | UDMA_ARB_1);

    /* Setup the TX transfer characteristics & buffers */
    MAP_DMA_setChannelTransfer(DMA_CH0_EUSCIB0TX0 | UDMA_PRI_SELECT,
    UDMA_MODE_BASIC, (void *) wr_data_ptr,
            (void *) MAP_SPI_getTransmitBufferAddressForDMA(EUSCI_B0_BASE),
            wr_data_size);

    /* Assigning/Enabling Interrupts */
    MAP_DMA_assignInterrupt(DMA_INT0, DMA_CHANNEL_0);
    MAP_Interrupt_enableInterrupt(INT_DMA_INT0);
    MAP_DMA_enableInterrupt(INT_DMA_INT0);

    fram_write_busy = 1;

    MAP_DMA_enableChannel(0);

    return 0;
}

unsigned int FRAM_dma_log_Stop(void) {

    if (UCB0NSS != 0) return 1;
    while (fram_write_busy) continue;
    UCB0NSS = 1;// cs = 1
    return 0;
}

unsigned int FRAM_dma_log_Start(uint32_t FRAM_Addr) {
    static uint8_t FRAM_wr_buffer[] = {0x02, 0x00, 0x00, 0x00};

    if (UCB0NSS == 0) return 1;

    UCB0NSS = 0;// cs = 0
    FRAM_wr_buffer[0] = 0x06;
    FRAM_dma_log_write(FRAM_wr_buffer, 1);
    FRAM_dma_log_Stop();

    FRAM_wr_buffer[3] = (FRAM_Addr >> 0) & 0xFF;
    FRAM_wr_buffer[2] = (FRAM_Addr >> 8) & 0xFF;
    FRAM_wr_buffer[1] = (FRAM_Addr >> 16) & 0xFF;
    FRAM_wr_buffer[0] = 0x02;
    UCB0NSS = 0;// cs = 0
    return FRAM_dma_log_write(FRAM_wr_buffer, sizeof(FRAM_wr_buffer));
}

unsigned int FRAM_dma_log_read(uint8_t *rd_data_ptr, unsigned int rd_data_size) {
    if (UCB0NSS != 0) return 1;

    while (fram_write_busy) continue;

    EUSCI_B0->RXBUF;

    MAP_DMA_assignChannel(DMA_CH0_EUSCIB0TX0);
    MAP_DMA_assignChannel(DMA_CH1_EUSCIB0RX0);

    MAP_DMA_setChannelControl(DMA_CH0_EUSCIB0TX0 | UDMA_PRI_SELECT,
    UDMA_SIZE_8 | UDMA_SRC_INC_8 | UDMA_DST_INC_NONE | UDMA_ARB_1);

    MAP_DMA_setChannelControl(DMA_CH1_EUSCIB0RX0 | UDMA_PRI_SELECT,
    UDMA_SIZE_8 | UDMA_SRC_INC_NONE | UDMA_DST_INC_8 | UDMA_ARB_1);

    /* Setup the RX transfer characteristics & buffers */
    MAP_DMA_setChannelTransfer(DMA_CH1_EUSCIB0RX0 | UDMA_PRI_SELECT,
    UDMA_MODE_BASIC,
            (void *) MAP_SPI_getReceiveBufferAddressForDMA(EUSCI_B0_BASE),
            rd_data_ptr,
            rd_data_size);

    /* Setup the TX transfer characteristics & buffers */
    MAP_DMA_setChannelTransfer(DMA_CH0_EUSCIB0TX0 | UDMA_PRI_SELECT,
    UDMA_MODE_BASIC, (void *) rd_data_ptr,
            (void *) MAP_SPI_getTransmitBufferAddressForDMA(EUSCI_B0_BASE),
            rd_data_size);


    /* Assigning/Enabling Interrupts */
    MAP_DMA_assignInterrupt(DMA_INT0, DMA_CHANNEL_1);
    MAP_Interrupt_enableInterrupt(INT_DMA_INT0);
    MAP_DMA_enableInterrupt(INT_DMA_INT0);

    fram_write_busy = 1;

    MAP_DMA_enableChannel(1);
    MAP_DMA_enableChannel(0);

    return 0;
}

unsigned int FRAM_dma_read_Start(uint32_t FRAM_Addr) {
    static uint8_t FRAM_rd_buffer[] = {0x03, 0x00, 0x00, 0x00};

    if (UCB0NSS == 0) return 1;

    FRAM_rd_buffer[3] = (FRAM_Addr >> 0) & 0xFF;
    FRAM_rd_buffer[2] = (FRAM_Addr >> 8) & 0xFF;
    FRAM_rd_buffer[1] = (FRAM_Addr >> 16) & 0xFF;
    FRAM_rd_buffer[0] = 0x03;
    UCB0NSS = 0;// cs = 0

    return FRAM_dma_log_read(FRAM_rd_buffer, sizeof(FRAM_rd_buffer));
}

void FRAM_dma_wait_EOT(void) {
    while (fram_write_busy) continue;
}

unsigned int FRAM_dma_wrsr(uint8_t block_protection) {
    static uint8_t FRAM_wr_buffer[] = {0x01, 0x00};

    if (UCB0NSS == 0) return 1;
    UCB0NSS = 0;// cs = 0
    FRAM_wr_buffer[0] = 0x06;
    FRAM_dma_log_write(FRAM_wr_buffer, 1);
    FRAM_dma_log_Stop();


    FRAM_wr_buffer[1] = block_protection;
    FRAM_wr_buffer[0] = 0x01;
    UCB0NSS = 0;// cs = 0;
    FRAM_dma_log_write(FRAM_wr_buffer, 2);

    return FRAM_dma_log_Stop();
}

unsigned int FRAM_dma_rdsr(uint8_t *block_protection) {
    static uint8_t FRAM_wr_bp[] = {0x05, 0x00};

    if (UCB0NSS == 0) return 1;
    UCB0NSS = 0;// cs = 0
    FRAM_wr_bp[0] = 0x05;
    FRAM_dma_log_read(FRAM_wr_bp, 2);
    FRAM_dma_log_Stop();

    *block_protection = FRAM_wr_bp[1];

    return 0;
}

#endif

#include "Motor.h"
#include "main.h"
#include "Maze.h"
#include "Tachometer.h"
#include "Reflectance.h"
#include "ADC.h"
#include "configure.h"

unsigned int frames_to_go = 0;

void FRAM_log_data(void){
    static volatile data_buffer_t log_buffer;

    if (frames_to_go) {
        log_buffer.StepsLeft    =  LeftSteps;
        log_buffer.StepsRight   = RightSteps;
        log_buffer.RealSpeedLeft = RealSpeedL;
        log_buffer.RealSpeedRight = RealSpeedR;
        log_buffer.setspeedLeft  = XstartL;
        log_buffer.setspeedRight = XstartR;
        log_buffer.Time         = time;
        log_buffer.vbat         = ((LPF_battery.Sum/LPF_battery.Size) * data.volt_calibr) >> 14;
        log_buffer.sensors      = current_sensor;
        log_buffer.whereami     = where_am_i;
        FRAM_dma_log_write((uint8_t *)&log_buffer, sizeof(log_buffer));
        frames_to_go--;
    }
}

void FRAM_log_slow(coordinate_t coordinate,  int length, unsigned int index) {
    static volatile data_buffer_t slow_buffer;

    if (frames_to_go) {
        slow_buffer.Time         = time;
        slow_buffer.whereami = 0;
        slow_buffer.coordX = coordinate.east;
        slow_buffer.coordY = coordinate.north;
        slow_buffer.nodeNum = index;
        slow_buffer.segmentLength = length;
        FRAM_dma_log_write((uint8_t*)&slow_buffer, sizeof(slow_buffer));
        frames_to_go--;
    }
}
