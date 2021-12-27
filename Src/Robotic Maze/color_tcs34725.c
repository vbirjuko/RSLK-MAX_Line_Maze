#include "color_sensor.h"
#include "i2c_drv.h"
#include "configure.h"

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

unsigned int color_sensor_init(void) {
		param = 0;
	i2c_rd_reg(TCS_ADDR, (TCS_COMMAND | ID), &param, 1);
	if (param != 0x44) return 1;
		param = ENABLE_PON; // PON
	i2c_wr_reg(TCS_ADDR, TCS_COMMAND | ENABLE, &param, sizeof(param));
		param = 243; // ATIME 33ms
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


	if (color_sensors[0] < data.color_threshold) {
		return black;
  } else {
		unsigned int i, invsaturation, minRGB=0xffffffff;
		int hueX, hueY;
		color_sensors[1] = (color_sensors[1] * data.color_red_thr) >> 10;
		color_sensors[2] = (color_sensors[2] * data.color_green_thr) >> 10;
		color_sensors[3] = (color_sensors[3] * data.color_blue_thr) >> 10;
		for (i=1; i<4; i++) {
			if (minRGB > color_sensors[i]) minRGB = color_sensors[i];
		}
		invsaturation = 3*1024 * minRGB / (color_sensors[1]+color_sensors[2]+color_sensors[3]);
		if (invsaturation > 512) return white;
		
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
}
