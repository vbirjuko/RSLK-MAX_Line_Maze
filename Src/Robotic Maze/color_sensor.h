/*
 * color_sensor.h
 *
 *  Created on: 22 февр. 2019 г.
 *      Author: wl
 */

#ifndef COLOR_SENSOR_H_
#define COLOR_SENSOR_H_
#include <stdint.h>

typedef enum {
    black = 0,
    red,
    green,
    yellow,
    blue,
    magenta,
    cyan,
    white
} t_color;


unsigned int color_sensor_init(void);
t_color check_color(void);
extern volatile uint16_t color_sensors[4];

#endif /* COLOR_SENSOR_H_ */
