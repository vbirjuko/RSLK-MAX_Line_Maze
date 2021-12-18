#include "color_sensor.h"
#include "i2c_drv.h"
#include "configure.h"
#include "Reflectance.h"

#define TCS_ADDR	(0x29 << 1)
#define TCS_COMMAND	0x80
#define TCS_INCREMENT	0x20

enum classReg {
		ENABLE = 0x00,
		ATIME = 0x01,
		WTIME = 0x03,
		AILTL = 0x04,
		AILTH = 0x05,
		AIHTL = 0x06,
		AIHTH = 0x07,
		PERS = 0x0C,
		CONFIG = 0x0D,
		CONTROL = 0x0F,
		ID = 0x12,
		STATUS = 0x13,
		CDATAL = 0x14,
		CDATAH = 0x15,
		RDATAL = 0x16,
		RDATAH = 0x17,
		GDATAL = 0x18,
		GDATAH = 0x19,
		BDATAL = 0x1A,
		BDATAH = 0x1B,
};

enum classMask {
		ENABLE_AIEN = 0x10,
		ENABLE_WEN = 0x08,
		ENABLE_AEN = 0x02,
		ENABLE_PON = 0x01,
		STATUS_AINT = 0x10,
		STATUS_AVALID = 0x01
};

enum classGain { X01 = 0, X04 = 1, X16 = 2, X60 = 3 };

unsigned char param;
volatile uint16_t color_sensors[4];
unsigned int color_sensor_present = 0;
static uint16_t *log_array_ptr = 0;

unsigned int color_sensor_init(void) {
		param = 0;
	i2c_rd_reg(TCS_ADDR, (TCS_COMMAND | ID), &param, 1);
	if (param != 0x44) return 1;
		param = ENABLE_PON; // PON
	i2c_wr_reg(TCS_ADDR, TCS_COMMAND | ENABLE, &param, sizeof(param));
		param = (256*5 - 50*12)/5 ; // ATIME 50 ms
	i2c_wr_reg(TCS_ADDR, TCS_COMMAND | ATIME, &param, sizeof(param));
		param = X01; // AGAIN 1x
	i2c_wr_reg(TCS_ADDR, TCS_COMMAND | CONTROL, &param, sizeof(param));
	color_sensor_present = 1;
	return 0;
}

t_color check_color(void) {
	
	if (color_sensor_present == 0) return black; // я слеп.
	
	param = ENABLE_PON | ENABLE_AEN; // PON & AEN
	i2c_wr_reg(TCS_ADDR, TCS_COMMAND | ENABLE, &param, sizeof(param));
	param = 0xff;
	do {
		i2c_rd_reg(TCS_ADDR, TCS_COMMAND | STATUS, &param, sizeof(param));
	} while ((param & STATUS_AVALID) == 0);
	i2c_rd_reg(TCS_ADDR, TCS_COMMAND | TCS_INCREMENT | CDATAL, (uint8_t *) color_sensors, 8);

  if (log_array_ptr) {
    *log_array_ptr++ = color_sensors[0];
    *log_array_ptr++ = color_sensors[1];
    *log_array_ptr++ = color_sensors[2];
    *log_array_ptr++ = color_sensors[3];
    if (log_array_ptr > (uint16_t *)&log_array[1999][15]) log_array_ptr = 0;
  }
  color_sensors[1] = (color_sensors[1] * data.color_red_thr) >> 10;
  color_sensors[2] = (color_sensors[2] * data.color_green_thr) >> 10;
  color_sensors[3] = (color_sensors[3] * data.color_blue_thr) >> 10;

	if (color_sensors[0] < data.color_threshold) {
		return black;
  } else {
		unsigned int i, invsaturation, minRGB=0x10000;
		int hueX, hueY;
		for (i=1; i<4; i++) {
			if (minRGB > color_sensors[i]) minRGB = color_sensors[i];
		}
		invsaturation = 3*1024 * minRGB / (color_sensors[1]+color_sensors[2]+color_sensors[3]);
		if ((int)invsaturation > data.color_saturation) return white;
		
		hueX = 2*color_sensors[1] - color_sensors[2] - color_sensors[3];
		hueY = 3*(color_sensors[2] - color_sensors[3]);

#define ABS(x)  (((x) < 0) ? (-(x)) : (x))
		if (ABS(hueX) > ABS(hueY)) {
			// Red or Cyan:
			if (hueX < 0)       return cyan;
			else 				return red;
		} else {
			if (hueX < 0) {
				if (hueY < 0)   return blue;
				else 			return green;
			} else {
				if (hueY < 0)   return magenta;
				else 			return yellow;
			}
		}
	}
/*	
	if (color_sensors[0] < data.color_threshold) {
		return black;
  } else {
		unsigned int bitfield = 0;
		if (color_sensors[1]*1024ul/color_sensors[0]  > data.color_red_thr) bitfield = 1;
		if (color_sensors[2]*1024ul/color_sensors[0]  > data.color_green_thr) bitfield += 4;
		if (color_sensors[3]*1024ul/color_sensors[0]  > data.color_blue_thr) bitfield += 2;

		switch (bitfield) {
				case 0: return black;
				case 1: return red;
				case 2: return blue;
				case 3: return magenta;
				case 4: return green;
				case 5: return yellow;
				case 6: return cyan;
				default: return white;
		}
	}
*/
}

#include "keyboard.h"
#include "LaunchPad.h"
#include "display.h"

void TestColor(void) {
  uint16_t color_array[4] = {0, 0, 0, 0};
  unsigned int i;
  t_color field_test_color;

  ColorSensorTestHSI(color_array, 1);
  log_array_ptr = (uint16_t *)&log_array[0][0];
  LaunchPad_Output1(GREEN);
  while (kbdread() != KEY_DOWN) {
      field_test_color = check_color();
      if (log_array_ptr == 0) LaunchPad_Output1(RED);
//        copy_data_dma((uint8_t *)color_sensors, (uint8_t *)color_array, sizeof(color_array));
    switch (field_test_color) {
      case red:       LaunchPad_Output(RED); break;
      case green:     LaunchPad_Output(GREEN); break;
      case blue:      LaunchPad_Output(BLUE); break;
      case yellow:    LaunchPad_Output(RED  | GREEN); break;
      case cyan:      LaunchPad_Output(BLUE | GREEN); break;
      case magenta:   LaunchPad_Output(BLUE | RED); break;
      case white:     LaunchPad_Output(BLUE | RED | GREEN); break;
      case black:     LaunchPad_Output(0x00); break;
    }
    for (i=0; i<4; i++) {
      color_array[i] = color_sensors[i];
    }
//    while(dma_copy_busy) WaitForInterrupt();
    ColorSensorTestHSI(color_array, 0);
  }
  LaunchPad_Output(0);
}
