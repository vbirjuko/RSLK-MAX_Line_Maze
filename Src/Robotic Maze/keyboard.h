#ifndef _KEYBOARD_H_
#define _KEYBOARD_H_
// Функция опроса клавиатуры

#include "resources.h"

#define I2C_PCA_ADDR	0x82
enum pca_reg {
    pca_reg_input_port = 0x00,
    pca_reg_output_port = 0x01,
    pca_reg_polarity_inversion = 0x02,
    pca_reg_configuration = 0x03,
};

typedef enum {
    DEBOUNCE_DELAY = (SCANPERSECOND*3/100), // 30 ms - подавление дребезга
    REPEAT_DELAY   = (SCANPERSECOND*3/4),   // 0.75c - задержка перед повтором
    REPEAT_PERIOD  = (SCANPERSECOND/3)      // 0.33c - период автоповтора
} KEY_DELAYS;

#ifdef RSLK_MAX
#ifdef PCA
#define KEY_UP        	(1u << 0)
#define KEY_DOWN        (1u << 2)
#define KEY_SET         (1u << 1)
#else
#define KEY_UP        (1u << 0)
#define KEY_DOWN        (1u << 2)
#define KEY_SET         (1u << 3)
#define KEY_PORT        P10
#endif
#else
#define KEY_UP        (1u << 0)
#define KEY_DOWN       (1u << 2)
#define KEY_SET         (1u << 1)
#define KEY_PORT        P5
#endif

#define KEY_LEFT			(1u << 8)
#define KEY_RIGHT			(1u << 9)

#define KEY_MASK        (KEY_UP | KEY_DOWN | KEY_SET)
#define HOLDED          (1u << 7)

unsigned int kbdread(void);
void kbd_init(void);
extern volatile unsigned int kbddelay;

#endif
