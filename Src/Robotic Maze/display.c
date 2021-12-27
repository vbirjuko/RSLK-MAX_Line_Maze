//#include <stdlib.h>
#include <stdint.h>
#include "msp.h"
#ifdef SSD1306
#include "Adafruit_SSD1306.h"
#elif defined SH1106
#include "Adafruit_SH1106.h"
#endif
#include "main.h"
#include "Maze.h"
#include "display.h"
#include "configure.h"
#include "Clock.h"
#include "dma.h"
#include "Timer32.h"
#include "fonts_oled.h"

unsigned char videobuff[8];
//unsigned char * volatile buffer_ptr; // do not use!!!!!! only for lcdwrite
volatile unsigned int buffer_count = 0;

unsigned char buffer[SSD1306_LCDHEIGHT * SSD1306_LCDWIDTH / 8] = { 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80,
	0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x80, 0x80, 0xC0, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x80, 0xC0, 0xE0, 0xF0, 0xF8, 0xFC, 0xF8, 0xE0, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x80,
	0x80, 0x80, 0x00, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00, 0xFF,
#if (SSD1306_LCDHEIGHT * SSD1306_LCDWIDTH > 96*16)
	0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x80, 0x80, 0x00, 0x00,
	0x80, 0xFF, 0xFF, 0x80, 0x80, 0x00, 0x80, 0x80, 0x00, 0x80, 0x80, 0x80, 0x80, 0x00, 0x80, 0x80,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x00, 0x00, 0x8C, 0x8E, 0x84, 0x00, 0x00, 0x80, 0xF8,
	0xF8, 0xF8, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xE0, 0xE0, 0xC0, 0x80,
	0x00, 0xE0, 0xFC, 0xFE, 0xFF, 0xFF, 0xFF, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0xFF, 0xC7, 0x01, 0x01,
	0x01, 0x01, 0x83, 0xFF, 0xFF, 0x00, 0x00, 0x7C, 0xFE, 0xC7, 0x01, 0x01, 0x01, 0x01, 0x83, 0xFF,
	0xFF, 0xFF, 0x00, 0x38, 0xFE, 0xC7, 0x83, 0x01, 0x01, 0x01, 0x83, 0xC7, 0xFF, 0xFF, 0x00, 0x00,
	0x01, 0xFF, 0xFF, 0x01, 0x01, 0x00, 0xFF, 0xFF, 0x07, 0x01, 0x01, 0x01, 0x00, 0x00, 0x7F, 0xFF,
	0x80, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x7F, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x01, 0xFF,
	0xFF, 0xFF, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x03, 0x0F, 0x3F, 0x7F, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xE7, 0xC7, 0xC7, 0x8F,
	0x8F, 0x9F, 0xBF, 0xFF, 0xFF, 0xC3, 0xC0, 0xF0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC, 0xFC, 0xFC,
	0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xF8, 0xF8, 0xF0, 0xF0, 0xE0, 0xC0, 0x00, 0x01, 0x03, 0x03, 0x03,
	0x03, 0x03, 0x01, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x03, 0x03, 0x03, 0x01, 0x01,
	0x03, 0x01, 0x00, 0x00, 0x00, 0x01, 0x03, 0x03, 0x03, 0x03, 0x01, 0x01, 0x03, 0x03, 0x00, 0x00,
	0x00, 0x03, 0x03, 0x00, 0x00, 0x00, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
	0x03, 0x03, 0x03, 0x03, 0x03, 0x01, 0x00, 0x00, 0x00, 0x01, 0x03, 0x01, 0x00, 0x00, 0x00, 0x03,
	0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
#if (SSD1306_LCDHEIGHT == 64)
	0x00, 0x00, 0x00, 0x80, 0xC0, 0xE0, 0xF0, 0xF9, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x3F, 0x1F, 0x0F,
	0x87, 0xC7, 0xF7, 0xFF, 0xFF, 0x1F, 0x1F, 0x3D, 0xFC, 0xF8, 0xF8, 0xF8, 0xF8, 0x7C, 0x7D, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0x3F, 0x0F, 0x07, 0x00, 0x30, 0x30, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0xFE, 0xFE, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0xC0, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x30, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0xC0, 0xFE, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0x7F, 0x3F, 0x1F,
	0x0F, 0x07, 0x1F, 0x7F, 0xFF, 0xFF, 0xF8, 0xF8, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0xF8, 0xE0,
	0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0xFE, 0x00, 0x00,
	0x00, 0xFC, 0xFE, 0xFC, 0x0C, 0x06, 0x06, 0x0E, 0xFC, 0xF8, 0x00, 0x00, 0xF0, 0xF8, 0x1C, 0x0E,
	0x06, 0x06, 0x06, 0x0C, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0xFE, 0xFE, 0x00, 0x00, 0x00, 0x00, 0xFC,
	0xFE, 0xFC, 0x00, 0x18, 0x3C, 0x7E, 0x66, 0xE6, 0xCE, 0x84, 0x00, 0x00, 0x06, 0xFF, 0xFF, 0x06,
	0x06, 0xFC, 0xFE, 0xFC, 0x0C, 0x06, 0x06, 0x06, 0x00, 0x00, 0xFE, 0xFE, 0x00, 0x00, 0xC0, 0xF8,
	0xFC, 0x4E, 0x46, 0x46, 0x46, 0x4E, 0x7C, 0x78, 0x40, 0x18, 0x3C, 0x76, 0xE6, 0xCE, 0xCC, 0x80,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x01, 0x07, 0x0F, 0x1F, 0x1F, 0x3F, 0x3F, 0x3F, 0x3F, 0x1F, 0x0F, 0x03,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x0F, 0x00, 0x00,
	0x00, 0x0F, 0x0F, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x0F, 0x00, 0x00, 0x03, 0x07, 0x0E, 0x0C,
	0x18, 0x18, 0x0C, 0x06, 0x0F, 0x0F, 0x0F, 0x00, 0x00, 0x01, 0x0F, 0x0E, 0x0C, 0x18, 0x0C, 0x0F,
	0x07, 0x01, 0x00, 0x04, 0x0E, 0x0C, 0x18, 0x0C, 0x0F, 0x07, 0x00, 0x00, 0x00, 0x0F, 0x0F, 0x00,
	0x00, 0x0F, 0x0F, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x0F, 0x00, 0x00, 0x00, 0x07,
	0x07, 0x0C, 0x0C, 0x18, 0x1C, 0x0C, 0x06, 0x06, 0x00, 0x04, 0x0E, 0x0C, 0x18, 0x0C, 0x0F, 0x07,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
#endif
#endif
	};


//configure P9.3 and P9.6 as GPIO (Reset and D/C pins)
#define DC          (BITBAND_PERI(P9->OUT, 6)) //(*((volatile uint8_t *)0x42099058))   /* Port 9 Output, bit 6 is DC*/
#define RESET       (BITBAND_PERI(P9->OUT, 3)) //(*((volatile uint8_t *)0x4209904C))   /* Port 9 Output, bit 3 is RESET*/
#define DC_BIT      (1ul << 6)
#define RESET_BIT   (1ul << 3)
 //configure P9.7, P9.5, and P9.4 as primary module function
#define UCA3SIMO    (1u << 7)
#define UCA3CLK     (1u << 5)
#define UCA3STE     (1u << 4)


// This is a helper function that sends 8-bit data to the LCD.
// Inputs: data  8-bit data to transmit
// Outputs: none
// Assumes: UCA3 and Port 9 have already been initialized and enabled
// The Data/Command pin must be valid when the eighth bit is
// sent.  The eUSCI module has no hardware FIFOs.
// 1) Wait for transmitter to be empty (let previous frame finish)
// 2) Set DC for data (1)
// 3) Write data to TXBUF, starts SPI
/*
void static lcddatawrite(unsigned char* data_ptr, unsigned int count){
// write this code as part of Lab 11
//    while (EUSCI_A3->IE & EUSCI_A_IE_TXIE) continue;
    while (EUSCI_A3->STATW & UCBUSY) continue;
    DC = 1; // data
//    EUSCI_A3->TXBUF = data;
    buffer_ptr = data_ptr;
    buffer_count = count;
    EUSCI_A3->IE |= EUSCI_A_IE_TXIE;
}

void static lcdcommandwrite(unsigned char* data_ptr, unsigned int count) {
//    while (EUSCI_A3->IE & EUSCI_A_IE_TXIE) continue;
    while (EUSCI_A3->STATW & UCBUSY) continue;
    DC = 0; // command
//    EUSCI_A3->TXBUF = data;
    buffer_ptr = data_ptr;
    buffer_count = count;
    EUSCI_A3->IE |= EUSCI_A_IE_TXIE;
}
*/

//#define OBSOLETE
typedef enum {
	lcdcommand = 0,
	lcddata = 1,
} lcddatacommand ;
#ifdef OBSOLETE
void static lcdwrite(unsigned char* data_ptr, unsigned int count, lcddatacommand dc){
    while (buffer_count || (EUSCI_A3->STATW & UCBUSY)) continue;
    DC = dc; // data
    buffer_ptr = data_ptr;
    buffer_count = count;
    EUSCI_A3->IE |= EUSCI_A_IE_TXIE;
}

void EUSCIA3_IRQHandler(void) {
    switch (EUSCI_A3->IV) {
        case 4: // tx buffer empty
            if (buffer_count--) {
                EUSCI_A3->TXBUF = *buffer_ptr++;
                if (buffer_count == 0) EUSCI_A3->IE &= ~EUSCI_A_IE_TXIE;
            }
            break;

        case 2: // rx buffer full
            EUSCI_A3->RXBUF; // пока что.
    }
}
#else
volatile unsigned int lcd_write_busy = 0;

void static lcdwrite(unsigned char* data_ptr, uint16_t count, lcddatacommand dc){

  while (DMA_getChannelMode(6) != UDMA_MODE_STOP) continue;

  while (lcd_write_busy) continue;
  DC = dc; // data

  /* Configuring DMA module */
  lcd_write_busy = 1;

  /* Assign DMA channel 6 to EUSCI_A3_TX0 */
  MAP_DMA_assignChannel(DMA_CH6_EUSCIA3TX);

  /* Setup the TX transfer characteristics & buffers */
  MAP_DMA_setChannelControl(DMA_CH6_EUSCIA3TX | UDMA_PRI_SELECT,
  UDMA_SIZE_8 | UDMA_SRC_INC_8 | UDMA_DST_INC_NONE | UDMA_ARB_1);
  MAP_DMA_setChannelTransfer(DMA_CH6_EUSCIA3TX | UDMA_PRI_SELECT,
  UDMA_MODE_BASIC, (void *) data_ptr,
          (void *) MAP_SPI_getTransmitBufferAddressForDMA(EUSCI_A3_BASE),
          count);

  MAP_DMA_assignInterrupt(INT_DMA_INT2, 6);

  /* Assigning/Enabling Interrupts */
  MAP_Interrupt_enableInterrupt(INT_DMA_INT2);
  MAP_DMA_enableInterrupt(INT_DMA_INT2);

  MAP_DMA_enableChannel(6);
}

void DMA_INT2_IRQHandler(void)
{
//    isrCounter++;
    MAP_DMA_clearInterruptFlag(6);
    MAP_DMA_disableChannel(6);

    lcd_write_busy = 0;
    // Disable the interrupt to allow execution
    MAP_Interrupt_disableInterrupt(INT_DMA_INT2);
    MAP_DMA_disableInterrupt(INT_DMA_INT2);
}

#endif

void print_dump(char *pData) {
	int i, j, k;
	unsigned char *scrbuffer_ptr;
	
	scrbuffer_ptr = buffer;
	for (j=0; j<8; j++) {
		for (i=0; i<14; i++) {
			for (k=0; k<8; k++) {
				unsigned char show_data;
				show_data = *(pData + ((13-i)*4 + (j>>1) ));
				*scrbuffer_ptr++ = (j & 0x01) ? ((char) IMAGES[((j & 0x01) ?  (show_data & 0x0F) : (show_data >> 4))] [k]) >> 1 :
																		 ((char) IMAGES[((j & 0x01) ?  (show_data & 0x0F) : (show_data >> 4))] [k]);
			};
			*scrbuffer_ptr++ = 0;
		};
		for (i=0; i<2; i++) {
			*scrbuffer_ptr++ = 0;
		}
	}
	update_display();
}

void show_hystogram(uint8_t mask, uint8_t *photo_sensor, unsigned int threshold, int number) {
	int i, j;
	int divider = 1000;
	unsigned char *scrbuffer_ptr;
	
//	if (threshold > 99) threshold = 99;
//	if (number >  99999) number =  99999;
//	if (number < -99999) number = -99999;
	
	for (i=0; i<3; i++) videobuff[i] = 0x11;
	
	videobuff[0] =  threshold / 100;
	videobuff[1] = (threshold % 100) / 10;
	videobuff[2] =  threshold % 10;

	for (i=3; i<8; i++) videobuff[i] = 0;
	
	if (number < 0) {
		number = -number;
		videobuff[3] = 0x10;
	} else {
	    videobuff[3] = 0x11;
	}
	
	for (i = 4; i<8; i++) {
		while (number >= divider) {
			number -= divider;
			videobuff[i]++;
		}
		divider /= 10;
	}
	
	scrbuffer_ptr = buffer;
	for (i=0; i<8 ; i++) {
		for (j=0; j < 116; j++) { 
			if (j < (photo_sensor[i]>>1)) {
				if (mask & (1<<i)) *scrbuffer_ptr = 0x7e;
				else 			   			*scrbuffer_ptr = (j & 0x01) ? 0x2a : 0x2a << 1;
			} else 							*scrbuffer_ptr = 0x00; 
			
			if (j == threshold>>4) *scrbuffer_ptr |= 0xAA;   //   10101010
			scrbuffer_ptr++;
		}
		for (j=0; j<4; j++) {
			*scrbuffer_ptr++ = ((mask & (1<<i)) ? IMAGES[18][j] : IMAGES[17][j]);
		}
		for (j=0; j<8; j++) {
			*scrbuffer_ptr++ = IMAGES[videobuff[i]][j];
		}
	}
	update_display();
}

void update_display(void) {
#ifdef SSD1306
    unsigned char start_col[] = {SSD1306_COLUMNADDR, 0, SSD1306_LCDWIDTH-1, SSD1306_PAGEADDR, 0, 7};
    lcdwrite(start_col, sizeof(start_col), lcdcommand);
    lcdwrite(buffer, SSD1306_LCDHEIGHT * SSD1306_LCDWIDTH / 8, lcddata);
#elif defined(SH1106)
    unsigned char start_col[] = {SH1106_SETLOWCOLUMN | 2, SH1106_SETHIGHCOLUMN, SH1106_PAGEADDR};
		unsigned int i;
		for (i = 0; i < 8; i++) {
			start_col[2] = SH1106_PAGEADDR | i;
			lcdwrite(start_col, sizeof(start_col), lcdcommand);
			lcdwrite(buffer + 128*i, SH1106_LCDWIDTH, lcddata);
		}
#endif
//  i2c_wr_reg(SSD1306_I2C_ADDRESS, 0x00, start_col, sizeof(start_col));
//  i2c_wr_reg(SSD1306_I2C_ADDRESS, 0x40, buffer, 1024);
}

void display_init(void) {
    volatile uint32_t delay;
#ifdef SSD1306
//	#define UPSIDEDOWN
		static const unsigned char init_sequence[] = {
        // Init sequence for 128x64 OLED module
        SSD1306_DISPLAYOFF,
        SSD1306_SETDISPLAYCLOCKDIV,	0x80,
        SSD1306_SETMULTIPLEX,	0x3F,
        SSD1306_SETDISPLAYOFFSET,	0x0,
        SSD1306_SETSTARTLINE | 0x0,
        SSD1306_CHARGEPUMP,	0x14,
        SSD1306_MEMORYMODE,	0x00,
#ifndef UPSIDEDOWN
        SSD1306_SEGREMAP | 0x1,     // orientation
				SSD1306_COMSCANDEC,
#else
        SSD1306_SEGREMAP | 0x0,     // orientation
				SSD1306_COMSCANINC,
#endif
				SSD1306_SETCOMPINS,	0x12,
        SSD1306_SETCONTRAST,	0xCF,
        SSD1306_SETPRECHARGE,	0xF1,
        SSD1306_SETVCOMDETECT,	0x40,
        SSD1306_DISPLAYALLON_RESUME,
        SSD1306_NORMALDISPLAY,
        SSD1306_DISPLAYON //--turn on oled panel
    };
#elif defined(SH1106)
		// Init sequence for 128x64 OLED module
    static const unsigned char init_sequence[] = {
    SH1106_DISPLAYOFF,                    // 0xAE
    SH1106_SETDISPLAYCLOCKDIV, 0x80,			// the suggested ratio 0x80
    SH1106_SETMULTIPLEX, 0x3F,
    SH1106_SETDISPLAYOFFSET, 0x00,				// no offset
	
    SH1106_SETSTARTLINE | 0x0,						// line #0 0x40
 //   SH1106_CHARGEPUMP, 0x14,
//    SH1106_MEMORYMODE, 0x00,              // 0x0 act like ks0108
    SH1106_SEGREMAP | 0x1,
    SH1106_COMSCANDEC,
    SH1106_SETCOMPINS, 0x12,
    SH1106_SETCONTRAST, 0xCF,
    SH1106_SETPRECHARGE, 0xF1,
    SH1106_SETVCOMDETECT, 0x40,
    SH1106_DISPLAYALLON_RESUME,           // 0xA4
    SH1106_NORMALDISPLAY,                 // 0xA6
    SH1106_DISPLAYON //--turn on oled panel
		};
#endif
		EUSCI_A3->CTLW0 = EUSCI_A_CTLW0_SWRST; //0x0001;             // hold the eUSCI module in reset mode
    EUSCI_A3->CTLW0 = EUSCI_A_CTLW0_CKPH | EUSCI_A_CTLW0_MSB | EUSCI_A_CTLW0_MST |
              EUSCI_A_CTLW0_MODE_2 | EUSCI_A_CTLW0_SYNC | EUSCI_A_CTLW0_SSEL__SMCLK |
              EUSCI_A_CTLW0_STEM | EUSCI_A_CTLW0_SWRST; // 0xAD83;
    EUSCI_A3->BRW = 3;
    // modulation is not used in SPI mode, so clear UCA3MCTLW
    EUSCI_A3->MCTLW = 0;
    P9->SEL0 |=  (UCA3SIMO | UCA3CLK | UCA3STE);
    P9->SEL1 &= ~(UCA3SIMO | UCA3CLK | UCA3STE);  // configure P9.7, P9.5, and P9.4 as primary module function
    P9->SEL0 &= ~(DC_BIT|RESET_BIT);
    P9->SEL1 &= ~(DC_BIT|RESET_BIT);      // configure P9.3 and P9.6 as GPIO (Reset and D/C pins)
    P9->DIR |= (DC_BIT|RESET_BIT);        // make P9.3 and P9.6 out (Reset and D/C pins)
    EUSCI_A3->CTLW0 &= ~EUSCI_A_CTLW0_SWRST;              // enable eUSCI module
    EUSCI_A3->IE &= ~(EUSCI_A_IE_RXIE | EUSCI_A_IE_TXIE); // disable interrupts

    RESET = 0;                            // reset the LCD to a known state, RESET low
    delay_us(1); // delay minimum 100 ns
    RESET = 1;                            // hold RESET high

#ifdef OBSOLETE
    NVIC_EnableIRQ(EUSCIA3_IRQn);
    NVIC_SetPriority(EUSCIA3_IRQn, 4);
#endif
    lcdwrite((unsigned char *)init_sequence, sizeof(init_sequence), lcdcommand);
    update_display();

    //	return i2c_wr_reg(SSD1306_I2C_ADDRESS, 0x00, init_sequence, sizeof(init_sequence));
}

void show_number(unsigned int number, int decimal) {

    static const unsigned char *ptr;
    int digit, i, j, k, leadingzero, digitcount;
    static const int offset[] = {6, 46, 86, 0, 30, 61, 92, 0, 36, 72, 92, 0, 36, 56, 92, 0, 20, 56, 92};
    const int *offset_ptr;
    const unsigned int base[] = {0xffffffff, 1, 10, 100, 1000, 10000}, *divider;
    unsigned char *scrbuffer_ptr;

    if (decimal > 3) decimal = 0;

    while (buffer_count) continue;	// если на экран передаётся буфер - подождём.

    scrbuffer_ptr = buffer;
    for (i=0; i<(SSD1306_LCDHEIGHT * SSD1306_LCDWIDTH / 8); i++) {
        *scrbuffer_ptr++ = 0;
    }
    if (number < 1000) {
        divider = base + 3;
        //      number = (number % 1000);
        if (!decimal) {
            offset_ptr = &offset[0];
            digitcount = 3;
        }	else	{
            offset_ptr = &offset[3] + decimal*4;
            digitcount = 4;
        }
    } else {
        divider = base + 4;
        number = (number % 10000);
        offset_ptr = &offset[3];
        digitcount = 4;
        decimal = 0;				// если больше 3 цифр - запятую не рисуем.
    }

    k = 0;
    //    for (k = 0; k < digitcount; k++) {
    while (k < digitcount) {
        digit = number / *divider;
        number  %= *divider--;
        if (decimal && ((digitcount - 1 - decimal) == k)) videobuff[k++] = 10;
        videobuff[k++] = digit;
    }

    leadingzero = 1;
    for (k = 0; k < digitcount; k++, offset_ptr++) {
        if (videobuff[k]) leadingzero = 0;

        if (!leadingzero) {
            for (j = 0; j < 8; j++) {
                ptr = &Verdana36x64[videobuff[k]][0][j];
                scrbuffer_ptr = buffer + (j * SSD1306_LCDWIDTH) + *offset_ptr;
                for (i = 0; i < 36; i++){
                    *scrbuffer_ptr++ = *ptr;
                    ptr += 8;
                }
            }
        }
    }
    update_display();
}

void putstr(int x, int y, char * string, unsigned int invert) {
#ifdef SSD1306
  unsigned char start_col[] = {SSD1306_COLUMNADDR, 0, SSD1306_LCDWIDTH-1, SSD1306_PAGEADDR, 0, 7};
#elif defined SH1106
	unsigned char start_col[] = {SH1106_SETLOWCOLUMN, SH1106_SETHIGHCOLUMN, SH1106_PAGEADDR};
#endif
	unsigned int i, bytecount; //
	unsigned char *scrbuffer_ptr;

	bytecount = x*8;
	scrbuffer_ptr = buffer + 8*x + SSD1306_LCDWIDTH*y;
	while (*string) {
		for (i = 0; i < 8; i++) {
			*scrbuffer_ptr++ = ASCIITAB[*string][i] ^ ((invert) ? 0xff : 0x00);
			bytecount++;
		}
		string++;
	}
#ifdef SSD1306
	if ((bytecount-1) < SSD1306_LCDWIDTH) {
		start_col[1] = x*8;
		start_col[2] = bytecount-1;
		start_col[4] = y;
		start_col[5] = y;
	} else {
		x = 0;
		start_col[1] = x;
		start_col[2] = SSD1306_LCDWIDTH-1;
		start_col[4] = y;
		start_col[5] = y;
		while (bytecount > (SSD1306_LCDWIDTH-1)) {
			start_col[5]++;
			bytecount -=SSD1306_LCDWIDTH;
		}
	}
	scrbuffer_ptr = buffer + 8*x + SSD1306_LCDWIDTH*y;
	lcdwrite(start_col, sizeof(start_col), lcdcommand);
	lcdwrite(scrbuffer_ptr, (start_col[5]-start_col[4]+1) * (start_col[2]-start_col[1]+1), lcddata);
#elif SH1106
	while (bytecount) {
		start_col[0] = SH1106_SETLOWCOLUMN | ((x*8+2) & 0x0f);
		start_col[1] = SH1106_SETHIGHCOLUMN | (((x*8+2) >> 4) & 0x0f);
		start_col[2] = SH1106_PAGEADDR | y;
		scrbuffer_ptr = buffer + 8*x + SH1106_LCDWIDTH*y;
		lcdwrite(start_col, sizeof(start_col), lcdcommand);
		lcdwrite(scrbuffer_ptr, (bytecount-x), lcddata);
		if (bytecount > SH1106_LCDWIDTH) {
			bytecount -= SH1106_LCDWIDTH;
		} else {
			break;
		}		
		x = 0;
		y++;
	}
#endif
}

void show_arrow(unsigned int x, unsigned int y, rotation_dir_t direction) {
	unsigned int index;
#ifdef SSD1306
  static unsigned char start_col[] = {SSD1306_COLUMNADDR, 0, 31, SSD1306_PAGEADDR, 0, 1};
#elif defined SH1106
	unsigned char start_col[] = {SH1106_SETLOWCOLUMN, SH1106_SETHIGHCOLUMN, SH1106_PAGEADDR};
#endif
		 
		switch (direction) {
			case left:
			 index = 0; break;
			 
			case straight:
			 index = 1; break;
			 
			case right:
				index = 2; break;
			 
			default:
				index = 3; break;
		}
#ifdef SSD1306		 
		start_col[1] = x;
		start_col[2] = x+15;
		 
		start_col[4] = y;
		start_col[5] = y+1;
		 
		lcdwrite(start_col, sizeof(start_col), lcdcommand);
		lcdwrite((uint8_t *)arrow[index], 32, lcddata);
#elif defined(SH1106)
		start_col[0] = SH1106_SETLOWCOLUMN | ((2 + x) & 0x0f);
		start_col[1] = SH1106_SETHIGHCOLUMN | (((2 + x) >> 4) & 0x0f);
		start_col[2] = SH1106_PAGEADDR | y;
		lcdwrite(start_col, sizeof(start_col), lcdcommand);
		lcdwrite((uint8_t *)arrow[index], 16, lcddata);

		start_col[0] = SH1106_SETLOWCOLUMN | ((2 + x) & 0x0f);
		start_col[1] = SH1106_SETHIGHCOLUMN | (((2 + x) >> 4) & 0x0f);
		start_col[2] = SH1106_PAGEADDR | y+1;
		lcdwrite(start_col, sizeof(start_col), lcdcommand);
		lcdwrite((uint8_t *)arrow[index] + 16, 16, lcddata);
#endif
}

void squareXY(unsigned int startX, unsigned int startY, unsigned int endX, unsigned int endY, unsigned int fill) {
	unsigned int i, x, y,	mask, mask_even, mask_odd;
	unsigned char * scrbuffer_ptr;

	if ((startX > SSD1306_LCDWIDTH-1) || (endX > SSD1306_LCDWIDTH-1) || (startY > SSD1306_LCDHEIGHT-1) || (endY > SSD1306_LCDHEIGHT-1)) return;
	if (startX > endX) {	
		startX = startX + endX;   // A + B = AB  =>  AB B
		endX = startX - endX;     // AB - B = A  =>  AB A
		startX = startX - endX;		// AB - A = B  =>  B  A
	}
	if (startY > endY) {	
		startY = startY + endY;   // A + B = AB  =>  AB B
		endY 	 = startY - endY;   // AB - B = A  =>  AB A
		startY = startY - endY;		// AB - A = B  =>  B  A
	}

	for (y = (startY >> 3); y <= ((endY >> 3)); y++) {
		mask = 0;
		for (i = y << 3; i <= ((y << 3) | 0x07); i++) {
			mask >>= 1;
			if ((i <= endY) && (i >= startY)) mask |= 0x80;
		}
		mask_even = mask_odd = mask;
		if (!fill) mask = ~mask;
		else if (fill == 2) mask_even = mask & 0xaa, mask_odd = mask & 0x55;
		else if (fill == 3) mask_even = mask & 0x55, mask_odd = mask & 0xaa;
		scrbuffer_ptr = buffer + y*SSD1306_LCDWIDTH + startX;
		for (x = startX; x <= endX; x++) {
			if (fill) {
				if (x & 1) *scrbuffer_ptr++ |= mask_odd;
				else 			 *scrbuffer_ptr++ |= mask_even;
			}	else			 *scrbuffer_ptr++ &= mask;
		}
	}
}

#define ABS(x)  (((x) < 0) ? (-(x)) : (x))

void lineXY(unsigned int startX, unsigned int startY, unsigned int endX, unsigned int endY, unsigned int fill) {
	int deltaX, deltaY, error, deltaerr, dirY;
	unsigned int X, Y;
	if ((startX > SSD1306_LCDWIDTH-1) || (endX > SSD1306_LCDWIDTH-1) || (startY > SSD1306_LCDHEIGHT-1) || (endY > SSD1306_LCDHEIGHT-1)) return;

	deltaX = ABS((int)endX - (int)startX);
	deltaY = ABS((int)endY - (int)startY);

	if (((deltaX >= deltaY) && (startX > endX)) || ((deltaX < deltaY) && (startY > endY))) {	
					startX = startX + endX;   // A + B = AB  =>  AB B
					endX = startX - endX;     // AB - B = A  =>  AB A
					startX = startX - endX;		// AB - A = B  =>  B  A
					startY = startY + endY;   // A + B = AB  =>  AB B
					endY 	 = startY - endY;   // AB - B = A  =>  AB A
					startY = startY - endY;		// AB - A = B  =>  B  A
	}

	error = 0;
	if (deltaX >= deltaY) {
		deltaerr = deltaY + 1;
		Y = startY;
		dirY = (((int)endY - (int)startY) > 0) ? 1 : -1;
		for (X = startX; X <= endX; X++) {
			//   plot (X, Y);
			if (fill) 			*(buffer + (Y >> 3)*SSD1306_LCDWIDTH + X) |=  (1 << (Y & 0x7));
			else 						*(buffer + (Y >> 3)*SSD1306_LCDWIDTH + X) &= ~(1 << (Y & 0x7));
			error = error + deltaerr;
			if (error >= deltaX + 1) {
				Y += dirY;
				error -= deltaX + 1;
			}
		}
	} else {
		deltaerr = deltaX + 1;
		X = startX;
		dirY = (((int)endX - (int)startX) > 0) ? 1 : -1;
		for (Y = startY; Y <= endY; Y++) {
			//   plot (X, Y);
			if (fill) 			*(buffer + (Y >> 3)*SSD1306_LCDWIDTH + X) |=  (1 << (Y & 0x7));
			else 						*(buffer + (Y >> 3)*SSD1306_LCDWIDTH + X) &= ~(1 << (Y & 0x7));
			error = error + deltaerr;
			if (error >= deltaY + 1) {
				X += dirY;
				error -= deltaY + 1;
			}
		}		
	}
}

char bin2hex(unsigned int x) {
	return (x > 9) ? x-10+'A' : x+'0';
}

unsigned int num2str(int x, char* string) {
	int divider;
	unsigned int leading_zero, digit, count = 0;
	divider = 1000000000;
	if (x < 0) {
		x = -x;
		*string++ = '-';
		count++;
	}
	leading_zero = 1;
	while (divider > 1) {
		digit = 0;
		while (x >= divider) {
			x -= divider;
			digit++;
		}
		divider /= 10;
		if ((leading_zero == 0) || (digit > 0)) {
			leading_zero = 0;
			*string++ = digit + '0';
			count++;
		}
	}
	*string++ = x + '0';
	*string = '\0';
	return ++count;
}


void Draw_Map(void) {
	unsigned int i, j, transpose=0;
	int maxX=0, maxY=0, minX=0, minY=0;
//	uint32_t *data_ptr, *src_ptr;

	typedef struct {
		uint8_t		coordX;
		uint8_t		coordY;
	} scr_map_t;
	
	scr_map_t	scr_map[MAX_MAP_SIZE];

	if (data.map_size == 0) return;
	squareXY(0,0, 127, 63, 0);
	for (i=0; i<data.map_size; i++) {
		if (map[i].coordinate.north > maxY) maxY = map[i].coordinate.north;
		if (map[i].coordinate.north < minY) minY = map[i].coordinate.north;
		if (map[i].coordinate.east  > maxX) maxX = map[i].coordinate.east;
		if (map[i].coordinate.east  < minX) minX = map[i].coordinate.east;
	}
	if ((maxX - minX) < (maxY - minY)) transpose = 1;
	for (i=0; i<data.map_size; i++) {
		if (transpose) {
			scr_map[i].coordY = (0+2) + ((map[i].coordinate.north - minY) * (128 - 2 - 2) / (maxY - minY));
			scr_map[i].coordX = (0+2) + ((map[i].coordinate.east  - minX) * (63 - 2 - 2) / (maxX - minX));
		} else {
			scr_map[i].coordX = (0 +2) + ((map[i].coordinate.east  - minX) * (128 - 2 - 2) / (maxX - minX));
			scr_map[i].coordY = (63-2) - ((map[i].coordinate.north - minY) * (63 - 2 - 2) / (maxY - minY));
		}
	}
	update_display();
	for (i=0; i<data.map_size; i++) {
		if (transpose) {
			squareXY(	scr_map[i].coordY - 1, 
								scr_map[i].coordX - 1, 
								scr_map[i].coordY + 1, 
								scr_map[i].coordX + 1, 1);
			for ( j = 0; j < 4; j++) {
				if (map[i].node_link[j] != UNKNOWN) {
					lineXY(	scr_map[i].coordY, 
										scr_map[i].coordX, 
										scr_map[map[i].node_link[j]].coordY,
										scr_map[map[i].node_link[j]].coordX, 1);
				}
			}
		} else {
			squareXY(	scr_map[i].coordX - 1, 
								scr_map[i].coordY - 1, 
								scr_map[i].coordX + 1,
								scr_map[i].coordY + 1, 1);
			for ( j = 0; j < 4; j++) {
				if (map[i].node_link[j] != UNKNOWN) {
					lineXY(	scr_map[i].coordX, 
										scr_map[i].coordY, 
										scr_map[map[i].node_link[j]].coordX,
										scr_map[map[i].node_link[j]].coordY, 1);
				}
			}
		}
	}
	//  последний штрих - нулевая точка
	if (transpose) {
		squareXY(	scr_map[data.green_cell_nr].coordY-2, 
							scr_map[data.green_cell_nr].coordX-2,
							scr_map[data.green_cell_nr].coordY+2, 
							scr_map[data.green_cell_nr].coordX+2, 1);
		squareXY(	scr_map[data.green_cell_nr].coordY-1, 
							scr_map[data.green_cell_nr].coordX-1,
							scr_map[data.green_cell_nr].coordY+1, 
							scr_map[data.green_cell_nr].coordX+1, 0);
	} else {
		squareXY(	scr_map[data.green_cell_nr].coordX-2, 
							scr_map[data.green_cell_nr].coordY-2, 
							scr_map[data.green_cell_nr].coordX+2,
							scr_map[data.green_cell_nr].coordY+2, 1);
		squareXY(	scr_map[data.green_cell_nr].coordX-1, 
							scr_map[data.green_cell_nr].coordY-1, 
							scr_map[data.green_cell_nr].coordX+1,
							scr_map[data.green_cell_nr].coordY+1, 0);
	}	
	// и конечная точка.
	if (data.red_cell_nr) {
		if (transpose) {
			squareXY(	scr_map[data.red_cell_nr].coordY-2, 
								scr_map[data.red_cell_nr].coordX-2,
								scr_map[data.red_cell_nr].coordY+2, 
								scr_map[data.red_cell_nr].coordX+2, 1);
		} else {
			squareXY(	scr_map[data.red_cell_nr].coordX-2, 
								scr_map[data.red_cell_nr].coordY-2, 
								scr_map[data.red_cell_nr].coordX+2,
								scr_map[data.red_cell_nr].coordY+2, 1);
		}	
	}
	update_display();
	// Теперь маршрут
	if (data.pathlength) 	{
		int direction;
		unsigned int prev_pos, curr_pos;
		prev_pos = data.green_cell_nr;

		for (direction = north; direction <= west; direction++) {
			if ((curr_pos = map[prev_pos].node_link[direction]) != UNKNOWN) break;
		}
		for (i = 0; i <= data.pathlength; i++) {
			if (transpose) {
				if (direction == north || direction == west)
					squareXY(scr_map[prev_pos].coordY + 1, scr_map[prev_pos].coordX - 1,
									 scr_map[curr_pos].coordY - 1, scr_map[curr_pos].coordX + 1, 2);
				else
					squareXY(scr_map[prev_pos].coordY - 1, scr_map[prev_pos].coordX + 1,
									 scr_map[curr_pos].coordY + 1, scr_map[curr_pos].coordX - 1, 2);
			} else {
				if (direction == north || direction == east)
					squareXY(scr_map[prev_pos].coordX + 1, scr_map[prev_pos].coordY - 1,
									 scr_map[curr_pos].coordX - 1, scr_map[curr_pos].coordY + 1, 2);
				else 
					squareXY(scr_map[prev_pos].coordX - 1, scr_map[prev_pos].coordY + 1,
									 scr_map[curr_pos].coordX + 1, scr_map[curr_pos].coordY - 1, 2);
			}
			prev_pos = curr_pos;
			direction += data.path[i];
			direction &= TURN_MASK;
			curr_pos = map[curr_pos].node_link[direction];
			if (curr_pos == UNKNOWN) break;
		}
	}
	update_display();
}

void ColorSensorTest(uint16_t *colors, unsigned int new) {
	unsigned int levels[3], i;
	char string[5];

	if (new) {
		squareXY(0, 0, 127, 63, 0);
		for (i = 1; i < 8; i++) {
			squareXY(0, i*8, 6, i*8, 1);
		}
		for (i = 0; i < 3; i++) {
			squareXY(i*40+8, 0, i*40+8, 63, 1);
		}
		squareXY( 9, 63-(data.color_red_thr>>4)  , 14, 63-(data.color_red_thr>>4)  , 1);
		squareXY(49, 63-(data.color_green_thr>>4), 54, 63-(data.color_green_thr>>4), 1);
		squareXY(89, 63-(data.color_blue_thr>>4) , 94, 63-(data.color_blue_thr>>4) , 1);
	}
	levels[0] = ((colors[1]<<10)/colors[0]) >> 4;
	levels[1] = ((colors[2]<<10)/colors[0]) >> 4;
	levels[2] = ((colors[3]<<10)/colors[0]) >> 4;

	for (i=0; i<3; i++) {
		squareXY(i*40+16, 63, (i+1)*40, 63-levels[i], 1);
		squareXY(i*40+16,  0, (i+1)*40, 63-levels[i], 0);
		string[0] = bin2hex((colors[i+1] >>12) & 0x0f);
		string[1] = bin2hex((colors[i+1] >> 8) & 0x0f);
		string[2] = bin2hex((colors[i+1] >> 4) & 0x0f);
		string[3] = bin2hex((colors[i+1] >> 0) & 0x0f);
		string[4] = 0;
		putstr(i*5+1, 0, string, 0);
	}
	update_display();
}

void ColorSensorTestHSI(uint16_t *colors, unsigned int new) {
	unsigned int levels[3], i;
	char string[5];

	if (new) {
		squareXY(0, 0, 127, 63, 0);
		for (i = 1; i < 8; i++) {
			squareXY(0, i*8, 6, i*8, 1);
		}
		for (i = 0; i < 3; i++) {
			squareXY(i*40+8, 0, i*40+8, 63, 1);
		}
		squareXY( 9, 63-(data.color_threshold>>4), 14, 63-(data.color_threshold>>4), 1);
		squareXY(49, 63-(512>>4), 54, 63-(512>>4), 1);
//		squareXY(89, 63-(data.color_blue_thr>>4) , 94, 63-(data.color_blue_thr>>4) , 1);
	}
	{
		unsigned int invsaturation, minRGB=0xffffffff;
//		int hueX, hueY;
		colors[1] = (colors[1] * data.color_red_thr) >> 10;
		colors[2] = (colors[2] * data.color_red_thr) >> 10;
		colors[3] = (colors[3] * data.color_red_thr) >> 10;
		for (i=1; i<4; i++) {
			if (minRGB > colors[i]) minRGB = colors[i];
		}
		invsaturation = 3*1024 * minRGB / (colors[1]+colors[2]+colors[3]);

		levels[0] = ((colors[0]) >> 4);
		levels[1] = (invsaturation >> 4);
		levels[2] = ((colors[3]) >> 4);

		for (i=0; i<3; i++) {
			squareXY(i*40+16, 63, (i+1)*40, 63-levels[i], 1);
			squareXY(i*40+16,  0, (i+1)*40, 63-levels[i], 0);
			string[0] = bin2hex((colors[i+1] >>12) & 0x0f);
			string[1] = bin2hex((colors[i+1] >> 8) & 0x0f);
			string[2] = bin2hex((colors[i+1] >> 4) & 0x0f);
			string[3] = bin2hex((colors[i+1] >> 0) & 0x0f);
			string[4] = 0;
			putstr(i*5+1, 0, string, 0);
		}
	}
	update_display();
}
