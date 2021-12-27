#include "color_sensor.h"
#include "i2c_drv.h"
#include "configure.h"

#define VEML_ADDR	(0x10 << 1)

enum classReg {
		VEML_CONF = 0x00,
		R_DATA = 0x08,
		G_DATA = 0x09,
		B_DATA = 0x0A,
		W_DATA = 0x0B,
};

enum classMask {
		CONF_SD = 0x01,
		CONF_AF = 0x02,
		CONF_TRIG = 0x04
};

enum classIT {
	t40ms = (0 << 4),
	t80ms = (1 << 4),
	t160ms = (2 << 4),
	t320ms = (3 << 4),
	t640ms = (4 << 4),
	t1280ms = (5 << 4)
};

uint16_t param;
volatile uint16_t color_sensors[4];
uint16_t color_shadow[4];
unsigned int color_sensor_present = 0;

unsigned int color_sensor_init(void) {
		param = t40ms;
	color_sensor_present = 1;
	if (I2C_SUCCESS == i2c_wr_reg(VEML_ADDR, VEML_CONF, (uint8_t*)&param, 2))	return 0;
	color_sensor_present = 0;
	return 1;
}

t_color check_color(void) {
	unsigned int i, valid_data;
// unsigned int bitfield = 0;
	
	if (color_sensor_present == 0) return black; // я слеп.
	
	i2c_rd_reg(VEML_ADDR, R_DATA, (uint8_t *) &color_shadow[1], 2);
	i2c_rd_reg(VEML_ADDR, G_DATA, (uint8_t *) &color_shadow[2], 2);
	i2c_rd_reg(VEML_ADDR, B_DATA, (uint8_t *) &color_shadow[3], 2);
	i2c_rd_reg(VEML_ADDR, W_DATA, (uint8_t *) &color_shadow[0], 2);

	do {
		i2c_rd_reg(VEML_ADDR, R_DATA, (uint8_t *) &color_sensors[1], 2);
		i2c_rd_reg(VEML_ADDR, G_DATA, (uint8_t *) &color_sensors[2], 2);
		i2c_rd_reg(VEML_ADDR, B_DATA, (uint8_t *) &color_sensors[3], 2);
		i2c_rd_reg(VEML_ADDR, W_DATA, (uint8_t *) &color_sensors[0], 2);
		
		valid_data = 1;
		for (i = 0; i < 4; i++) {
			if (color_shadow[i] != color_sensors[i]) {
				valid_data = 0;
				color_shadow[i] = color_sensors[i];
			}
		}
	} while (valid_data == 0);
	
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
		if((int)invsaturation > data.color_saturation) return white;
		
		hueX = 2*color_sensors[1] - color_sensors[2] - color_sensors[3];
		hueY = 3*(color_sensors[2] - color_sensors[3]);

        #define ABS(x)  (((x) < 0) ? (-(x)) : (x))
		if (ABS(hueX) > ABS(hueY)) {
			// Red or Cyan:
			if (hueX < 0)       return cyan;
			else 			    return red;
		} else {
			if (hueX < 0) {
				if (hueY < 0)   return blue;
				else 			return green;
			} else {
				if (hueY < 0)   return magenta;
				else 		    return yellow;
			}
		}
	}
}
