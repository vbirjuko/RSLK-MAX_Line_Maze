#ifndef DISPLAY_H
#define DISPLAY_H

#include <stdint.h>
#include "Adafruit_SSD1306.h"
#include "maze.h"


typedef enum {
    lcdcommand = 0,
    lcddata = 1,
} lcddatacommand ;

//extern unsigned char videobuff[8];
extern unsigned char buffer[SSD1306_LCDHEIGHT * SSD1306_LCDWIDTH / 8];


void display_bcd(int number, int threshold);
void print_dump(char *pData);
void show_hystogram(uint8_t mask, uint8_t *photo_sensor, unsigned int threshold, int number);
void display_init(void);
void update_display(void);
void show_number(unsigned int number, int decimal);
void putstr(int x, int y, char * string, unsigned int invert);
void show_arrow(unsigned int x, unsigned int y, rotation_dir_t direction);
void squareXY(unsigned int startX, 	unsigned int startY, 
							unsigned int endX, 		unsigned int endY, 
							unsigned int fill);
void lineXY(unsigned int startX, unsigned int startY,
            unsigned int endX, unsigned int endY,
            unsigned int fill);
char bin2hex(unsigned int x);
unsigned int num2str(int x, char* string);
void Draw_Map(void);
void ColorSensorTest(uint16_t * colors, unsigned int new);
void ColorSensorTestHSI(uint16_t * colors, unsigned int new);

void ssd1351_Init(void);
void setAddrWindow(unsigned int x1, unsigned int y1,
                   unsigned int w, unsigned int h);

void line_test (void);
#endif
