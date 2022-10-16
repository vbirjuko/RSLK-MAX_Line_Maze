/*********************************************************************
This is a library for our Monochrome OLEDs based on SSD1306 drivers

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/category/63_98

These displays use SPI to communicate, 4 or 5 pins are required to  
interface

Adafruit invests time and resources providing this open source code, 
please support Adafruit and open-source hardware by purchasing 
products from Adafruit!

Written by Limor Fried/Ladyada  for Adafruit Industries.  
BSD license, check license.txt for more information
All text above, and the splash screen must be included in any redistribution
*********************************************************************/

/*********************************************************************
I change the adafruit SSD1306 to SH1106

SH1106 driver similar to SSD1306 so, just change the display() method.

However, SH1106 driver don't provide several functions such as scroll commands.


*********************************************************************/

#define BLACK 0
#define WHITE 1
#define INVERSE 2

#define SH1106_I2C_ADDRESS   (0x3C << 1) // 011110+SA0+RW - 0x3C or 0x3D
// Address for 128x32 is 0x3C
// Address for 128x64 is 0x3D (default) or 0x3C (if SA0 is grounded)

/*=========================================================================
    SH1106 Displays
    -----------------------------------------------------------------------
    The driver is used in multiple displays (128x64, 128x32, etc.).
    Select the appropriate display below to create an appropriately
    sized framebuffer, etc.

    SH1106_128_64  128x64 pixel display

    SH1106_128_32  128x32 pixel display

    SH1106_96_16

    -----------------------------------------------------------------------*/
   #define SH1106_128_64
//   #define SH1106_128_32
//   #define SH1106_96_16
/*=========================================================================*/

#if defined SH1106_128_64 && defined SH1106_128_32
  #error "Only one SH1106 display can be specified at once in SH1106.h"
#endif
#if !defined SH1106_128_64 && !defined SH1106_128_32 && !defined SH1106_96_16
  #error "At least one SH1106 display must be specified in SH1106.h"
#endif

#if defined SH1106_128_64
  #define SH1106_LCDWIDTH                  128
  #define SH1106_LCDHEIGHT                 64
#endif
#if defined SH1106_128_32
  #define SH1106_LCDWIDTH                  128
  #define SH1106_LCDHEIGHT                 32
#endif
#if defined SH1106_96_16
  #define SH1106_LCDWIDTH                  96
  #define SH1106_LCDHEIGHT                 16
#endif

#define SH1106_SETPUMPVOLTAGE	0x30
#define SH1106_SETCONTRAST 0x81
#define SH1106_DISPLAYALLON_RESUME 0xA4
#define SH1106_DISPLAYALLON 0xA5
#define SH1106_NORMALDISPLAY 0xA6
#define SH1106_INVERTDISPLAY 0xA7
#define SH1106_DISPLAYOFF 0xAE
#define SH1106_DISPLAYON 0xAF
#define SH1106_SETDCDC	0xAD
#define SH_1106_DCDCOFF 0x8A
#define SH1106_SETDISPLAYOFFSET 0xD3
#define SH1106_SETCOMPINS 0xDA

#define SH1106_SETVCOMDETECT 0xDB

#define SH1106_SETDISPLAYCLOCKDIV 0xD5
#define SH1106_SETPRECHARGE 0xD9

#define SH1106_SETMULTIPLEX 0xA8

#define SH1106_SETLOWCOLUMN 0x00
#define SH1106_SETHIGHCOLUMN 0x10

#define SH1106_SETSTARTLINE 0x40

//		#define SH1106_MEMORYMODE 0x20
#define SH1106_PAGEADDR   0xB0

#define SH1106_COMSCANINC 0xC0
#define SH1106_COMSCANDEC 0xC8

#define SH1106_SEGREMAP 0xA0

//		#define SH1106_CHARGEPUMP 0x8D

//		#define SH1106_EXTERNALVCC 0x1
//		#define SH1106_SWITCHCAPVCC 0x2

