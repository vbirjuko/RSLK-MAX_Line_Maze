#include "msp.h"
#include "color_sensor.h"
#include "configure.h"
//#include "main.h"
#ifdef OBSOLETE
void TimerA2Init(void) {
    TIMER_A2->CTL &= ~0x0030;       // halt Timer A2
    TIMER_A2->EX0 = 0;
    TIMER_A2->CTL = TIMER_A_CTL_SSEL__TACLK | TIMER_A_CTL_MC__STOP | TIMER_A_CTL_CLR;
}

#define S2  (BITBAND_PERI(P10->OUT, 1))
#define S3  (BITBAND_PERI(P10->OUT, 0))
// p4.2 clock input
unsigned int color_sensor_init(void) {
    P10->SEL0 &= ~((1u << 0) | (1u << 1));
    P10->SEL1 &= ~((1u << 0) | (1u << 1));    // 1) configure GPIO
    P10->DIR  |=  ((1u << 0) | (1u << 1));    // 2) make S2/S3 out
    P4->SEL0 &= ~(1u << 2);               // P4.2 secondary function ta2clk
    P4->SEL1 |=  (1u << 2);
    P4->DIR  &= ~(1u << 2);
    TimerA2Init();
		return 0;
}

volatile uint16_t color_sensors[4]; // 0 - clear, 1 - Red, 2 - Green, 3 - Blue
void color_iteration(void) {
    static unsigned int sensor_no = 0;
    static uint16_t prev_timer = 0;
    TIMER_A2->CTL = TIMER_A_CTL_SSEL__TACLK | TIMER_A_CTL_MC__STOP;
    color_sensors[sensor_no] = TIMER_A2->R - prev_timer;
    prev_timer = TIMER_A2->R;
    if (++sensor_no > 3) sensor_no = 0;
    S2 = (sensor_no & 0x01) ? 0 : 1;
    S3 = (sensor_no & 0x02) ? 1 : 0;
    TIMER_A2->CTL = TIMER_A_CTL_SSEL__TACLK | TIMER_A_CTL_MC__CONTINUOUS;
}

t_color check_color(void) {
    unsigned int bitfield = 0;
    if (color_sensors[0] < data.color_threshold) {
        return black;
    } else {
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
}
#endif
