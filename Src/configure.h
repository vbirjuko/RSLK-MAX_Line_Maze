#ifndef _CONFIGURE_H_
#define _CONFIGURE_H_

#include "stdint.h"

#define MAX_MAP_SIZE	200

typedef struct {
    int threshold;
    int maxspeed;
    int turnspeed;
    int minspeed;
    int acceleration;
    int k_error;
    int k_diff;
    int k_integral;
    int on_way;
    int timetorun;
    int lefthand;
    int runnumber;
    int color_red_thr;
    int color_green_thr;
    int color_blue_thr;
    int color_threshold;
    int pathlength;
    int sq_init;
    int tolerance;
    int guarddist;
    int sensor_offset;
    int map_size;
    int	brake_path;
    int red_cell_nr;
    int maxmotor;
    int motor_Kint;
    int motor_Kprop;
    int ir_led_level;
    int low_battery_level;
    int ignore_coordinate_error;
    int	turncost;
    int	crosscost;
    int green_cell_nr;
    int volt_calibr;
    int stepcost;
    int p_step;
    int str_accel;
    int color_saturation;
    uint8_t path[MAX_MAP_SIZE * 2];
    int length[MAX_MAP_SIZE * 2];
    int log_watermark;
    int crc32;
} eedata_t ;

extern eedata_t data;

typedef enum {none, decimal, hex, hex32, execute, speed_sel, direction_sel} datatype_t;

typedef struct {
    unsigned char todisplay[16];
    void *variable;
    datatype_t datatype;
} menuitem_t;

void edit_path(void);
void do_menu(menuitem_t* item, unsigned int last_menu_item);

#define EEPROM_COPY_ADDRESS	            0x7F000
#define EEPROM_CONFIG_ADDRESS           0x00000
#define ROM_map_addr					0x10000
#define LOG_ADDRESS						0x20000
#define LOG_END_ADDRESS                 0x3FFFF

#endif
