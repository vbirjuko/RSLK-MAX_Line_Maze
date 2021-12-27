#include "msp.h"
#include "SPI_EEProm.h"
#include "configure.h"
#include "dma.h"
#include "main.h"
#include "Timer32.h"


#define USE_DMA_FOR_SPI
 //configure P10.0 - P10.3 as primary module function
#define UCB3SOMI	(1u << 3)
#define UCB3SIMO    (1u << 2)
#define UCB3CLK     (1u << 1)
#define UCB3STE     (1u << 0)
#define NSS					(BITBAND_PERI(P10->OUT, 0))

static volatile unsigned int eeprom_write_busy = 0;
static volatile uint8_t exchange_buffer[256+4];
uint8_t *volatile spi_buffer_tx_ptr, *volatile spi_buffer_rx_ptr;

void SPI_EEProm_Init(void) {
	EUSCI_B3->CTLW0 = EUSCI_B_CTLW0_SWRST; //0x0001;             // hold the eUSCI module in reset mode
    EUSCI_B3->CTLW0 = EUSCI_B_CTLW0_CKPH | EUSCI_B_CTLW0_MSB | EUSCI_B_CTLW0_MST |
              EUSCI_B_CTLW0_MODE_2 | EUSCI_B_CTLW0_SYNC | EUSCI_B_CTLW0_SSEL__SMCLK |
              EUSCI_B_CTLW0_STEM | EUSCI_B_CTLW0_SWRST;
    EUSCI_B3->BRW = 3;  // 4 Mbps

	P10->SEL0 |=  (UCB3SIMO | UCB3SOMI | UCB3CLK | UCB3STE);
    P10->SEL1 &= ~(UCB3SIMO | UCB3SOMI | UCB3CLK | UCB3STE);   //configure P10.1 - P10.3 as primary module function
//		P10->SEL0 &= ~(UCB3STE);
//		P10->SEL1 &= ~(UCB3STE);      // configure P10.0 as GPIO (CS#)
    P10->DIR  |=  (UCB3STE);      // make P10.0 out
    NSS = 1;

//    EUSCI_B3->RXBUF;	// Сброс прерываний
//    EUSCI_B3->RXBUF;	 // Reset possible pending irq
    EUSCI_B3->IE &= ~(EUSCI_B_IE_RXIE | EUSCI_B_IE_TXIE); // disable interrupts
    EUSCI_B3->CTLW0 &= ~EUSCI_B_CTLW0_SWRST;              // enable eUSCI module
	
//    NVIC_EnableIRQ(EUSCIB3_IRQn);
//    NVIC_SetPriority(EUSCIB3_IRQn, 4);

}


void  spi_exchange(unsigned char * send_data_ptr, uint16_t count) {

    while (DMA_getChannelMode(6) != UDMA_MODE_STOP) continue;

    MAP_DMA_assignChannel(DMA_CH6_EUSCIB3TX0);
    MAP_DMA_assignChannel(DMA_CH7_EUSCIB3RX0);

    MAP_DMA_setChannelControl(DMA_CH7_EUSCIB3RX0 | UDMA_PRI_SELECT,
    UDMA_SIZE_8 | UDMA_SRC_INC_NONE | UDMA_DST_INC_8 | UDMA_ARB_1);

    MAP_DMA_setChannelControl(DMA_CH6_EUSCIB3TX0 | UDMA_PRI_SELECT,
    UDMA_SIZE_8 | UDMA_SRC_INC_8 | UDMA_DST_INC_NONE | UDMA_ARB_1);


    /* Setup the RX transfer characteristics & buffers */
    MAP_DMA_setChannelTransfer(DMA_CH7_EUSCIB3RX0 | UDMA_PRI_SELECT,
    UDMA_MODE_BASIC,
            (void *) MAP_SPI_getReceiveBufferAddressForDMA(EUSCI_B3_BASE),
            send_data_ptr,
            count);

    /* Setup the TX transfer characteristics & buffers */
    MAP_DMA_setChannelTransfer(DMA_CH6_EUSCIB3TX0 | UDMA_PRI_SELECT,
    UDMA_MODE_BASIC, (void *) send_data_ptr,
            (void *) MAP_SPI_getTransmitBufferAddressForDMA(EUSCI_B3_BASE),
            count);

    /* Assigning/Enabling Interrupts */
//    MAP_Interrupt_enableInterrupt(INT_DMA_INT2);
//    MAP_DMA_enableInterrupt(INT_DMA_INT2);

    MAP_DMA_enableChannel(7);
    MAP_DMA_enableChannel(6);


    while (DMA_getChannelMode(7) != UDMA_MODE_STOP) continue;
}
/*
void DMA_INT2_IRQHandler(void)
{
//    isrCounter++;
    MAP_DMA_clearInterruptFlag(7);
//    MAP_DMA_disableChannel(6);
//    MAP_DMA_disableChannel(7);

    eeprom_write_busy = 0;
    // Disable the interrupt to allow execution
    MAP_Interrupt_disableInterrupt(INT_DMA_INT2);
    MAP_DMA_disableInterrupt(INT_DMA_INT2);
}
*/
#ifdef USE_DMA_FOR_SPI
unsigned int spi_read_eeprom(unsigned int ee_address, unsigned char * ram_address, unsigned int ee_size) {
    unsigned int bytes_to_read;
//    uint8_t *spi_buffer_ptr;

    while (ee_size) {
        if (ee_size > 256) bytes_to_read = 256;
        else     bytes_to_read = ee_size;
        ee_size -= bytes_to_read;

        exchange_buffer[0] = eeprom_read_data;
        exchange_buffer[1] = ((ee_address & 0x0ff0000) >> 16) & 0xFF;
        exchange_buffer[2] = ((ee_address & 0x000FF00) >>  8) & 0xFF;
        exchange_buffer[3] = ((ee_address & 0x00000ff) >>  0) & 0xFF;

        while (dma_copy_busy) continue;

        spi_exchange((unsigned char *)exchange_buffer, bytes_to_read + 4);

        copy_data_dma ((uint8_t *)exchange_buffer + 4, ram_address, bytes_to_read);

        ee_address += bytes_to_read;
        ram_address += bytes_to_read;
    }
    while (dma_copy_busy) continue;
    return 0;
}

void spi_write_eeprom(unsigned int ee_address, unsigned char * ram_address, unsigned int ee_size) {
//    const uint8_t enable_command_string[] = {eeprom_write_enable};
          uint8_t *data_ptr;
    unsigned int i, eeprom_ptr, bytes_to_copy;

    eeprom_ptr = ee_address;
    for (i = 0; i < (ee_size / 4096)+1; i++) {

        exchange_buffer[0] = eeprom_write_enable;
        spi_exchange((unsigned char *)exchange_buffer, 1);  // Write enable

        exchange_buffer[0] = eeprom_sector_erase;
        exchange_buffer[1] = ((eeprom_ptr & 0x0ff0000) >> 16) & 0xFF;
        exchange_buffer[2] = ((eeprom_ptr & 0x000FF00) >>  8) & 0xFF;
        exchange_buffer[3] = ((eeprom_ptr & 0x00000ff) >>  0) & 0xFF;
        spi_exchange((unsigned char *)exchange_buffer, 4);      // Erase sector
                                                                                                                                    // Wait for complete
        do {
            delay_us(50000);
            exchange_buffer[0] = eeprom_read_status_register;
            spi_exchange((unsigned char *)exchange_buffer, 2);
        } while (exchange_buffer[1] & EEPROM_STATUS_WIP);
        eeprom_ptr += 4096;
    }

    data_ptr = ram_address;
    eeprom_ptr = ee_address;
    while (ee_size) {
        bytes_to_copy = (ee_size > 256) ? 256 : ee_size;
        copy_data_dma(data_ptr, (uint8_t *)&exchange_buffer[4], bytes_to_copy);
        data_ptr += bytes_to_copy;
        ee_size -= bytes_to_copy;
        exchange_buffer[0] = eeprom_write_enable;
        spi_exchange((unsigned char *)exchange_buffer, 1);  // Write enable

        exchange_buffer[0] = eeprom_page_program;
        exchange_buffer[1] = ((eeprom_ptr & 0x0ff0000) >> 16) & 0xFF;
        exchange_buffer[2] = ((eeprom_ptr & 0x000FF00) >>  8) & 0xFF;
        exchange_buffer[3] = ((eeprom_ptr & 0x00000ff) >>  0) & 0xFF;
        while (dma_copy_busy) continue;
        spi_exchange((unsigned char *)exchange_buffer, bytes_to_copy + 4);   // Write page

        eeprom_ptr += 256;

        do {
            delay_us(2000);
            exchange_buffer[0] = eeprom_read_status_register;
            spi_exchange((unsigned char *)exchange_buffer, 2);
        } while (exchange_buffer[1] & EEPROM_STATUS_WIP);
    }
}

#else

static volatile unsigned int spi_buffer_rx_count=0, spi_buffer_tx_count = 0, spi_countdown = 0, spi_count = 0;
static volatile unsigned int errorcode = 0;

static unsigned int spi_write(unsigned char* send_data_ptr, unsigned int send_count, \
											unsigned char* receive_ptr,   unsigned int receive_count) {

		errorcode = 0;
    while (EUSCI_B3->STATW & UCBUSY) continue;
		NSS = 0;
    spi_buffer_tx_ptr = send_data_ptr;
    spi_buffer_rx_ptr = receive_ptr;
    spi_buffer_tx_count = send_count;
	spi_countdown = send_count;
	spi_count = send_count + receive_count;
    spi_buffer_rx_count = receive_count;
		EUSCI_B3->IE = EUSCI_B_IE_TXIE | EUSCI_B_IE_RXIE;
    while (spi_count) {
			if (((spi_buffer_rx_count | spi_buffer_tx_count) == 0) && errorcode) break;
		}
		NSS = 1;
		return errorcode;
}

unsigned int spi_read_eeprom(unsigned int ee_address, unsigned char * ram_address, unsigned int ee_size) {
	uint8_t read_command_string[4];

//		do {
//			time_delay = 50/2;  //  50 ms
//			while (time_delay) continue;
//			spi_write(read_command_string, 1, &read_command_string[1], 1);
//		} while (1);

	read_command_string[0] = eeprom_read_data;
	read_command_string[1] = ((ee_address & 0x0ff0000) >> 16) & 0xFF;
	read_command_string[2] = ((ee_address & 0x000FF00) >>  8) & 0xFF;
	read_command_string[3] = ((ee_address & 0x00000ff) >>  0) & 0xFF;
	while(spi_write(read_command_string, sizeof(read_command_string), ram_address, ee_size));
	return 0;
}

void spi_write_eeprom(unsigned int ee_address, unsigned char * ram_address, unsigned int ee_size) {
	const uint8_t enable_command_string[] = {eeprom_write_enable};
	uint8_t write_command_string[260], *data_ptr;
	unsigned int i, eeprom_ptr;

	eeprom_ptr = ee_address;
  for (i = 0; i < (ee_size / 4096)+1; i++) {
		write_command_string[0] = eeprom_sector_erase;
		write_command_string[1] = ((eeprom_ptr & 0x0ff0000) >> 16) & 0xFF;
		write_command_string[2] = ((eeprom_ptr & 0x000FF00) >>  8) & 0xFF;
		write_command_string[3] = ((eeprom_ptr & 0x00000ff) >>  0) & 0xFF;
		
		spi_write((unsigned char *)enable_command_string, sizeof(enable_command_string), (void *)0, 0); // Write Enable
		spi_write(write_command_string, 4, (void *) 0, 0);		// Erase sector
																																	// Wait for complete
		write_command_string[0] = eeprom_read_status_register;
		do {
			time_delay = 50/2;  //  50 ms
			while (time_delay) continue;
			while (spi_write(write_command_string, 1, &write_command_string[1], 1));
		} while (write_command_string[1] & EEPROM_STATUS_WIP);
		eeprom_ptr += 4096;
	}
	
	data_ptr = ram_address;
	eeprom_ptr = ee_address;
	while (data_ptr < (ram_address + ee_size)) {
		write_command_string[0] = eeprom_page_program;
		write_command_string[1] = ((eeprom_ptr & 0x0ff0000) >> 16) & 0xFF;
		write_command_string[2] = ((eeprom_ptr & 0x000FF00) >>  8) & 0xFF;
		write_command_string[3] = ((eeprom_ptr & 0x00000ff) >>  0) & 0xFF;
		for (i = 4; i < 256+4; i++) {
			write_command_string[i] = *data_ptr++;
		}
		spi_write((unsigned char *)enable_command_string, sizeof(enable_command_string), (void *)0, 0);  // Write enable
		spi_write(write_command_string, 4+256, (void *)0, 0);	// Write page

		eeprom_ptr += 256;

		write_command_string[0] = eeprom_read_status_register;
		do {
			time_delay = 2;  //  2-4 ms
			while (time_delay) continue;
			while (spi_write(write_command_string, 1, &write_command_string[1], 1));
		} while (write_command_string[1] & EEPROM_STATUS_WIP);
	}
}

void EUSCIB3_IRQHandler(void) {
    switch (EUSCI_B3->IV) {
            case 4: // tx buffer empty
                if (spi_buffer_tx_count) {
                    spi_buffer_tx_count--;
                    EUSCI_B3->TXBUF = *spi_buffer_tx_ptr++;
                    if ((spi_buffer_tx_count | spi_buffer_rx_count) == 0) EUSCI_B3->IE &= ~EUSCI_B_IE_TXIE;
                } else if (spi_buffer_rx_count--) {
                    EUSCI_B3->TXBUF = 0xFF;
                    if (spi_buffer_rx_count == 0) EUSCI_B3->IE &= ~EUSCI_B_IE_TXIE;
                }
                break;

            case 2: // rx buffer full
                if (EUSCI_B3->STATW & EUSCI_B_STATW_OE) errorcode = 1;
                if (spi_countdown) {
                    spi_countdown--;
                    EUSCI_B3->RXBUF;
                } else {
                    *spi_buffer_rx_ptr++ = EUSCI_B3->RXBUF;
                }
                if (spi_count) spi_count--;
                break;
    }
}
#endif
