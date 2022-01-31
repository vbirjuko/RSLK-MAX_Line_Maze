#include "keyboard.h"
#include "stdint.h"
#include "LaunchPad.h"
#include "msp.h"
#include "rand.h"
#include "i2c_drv.h"
//#include "commandline.h"

__WEAK void BlueTooth_parse(void) {
    return;
}

volatile unsigned int kbddelay;
unsigned int readkey;

void kbd_init(void) {
#ifdef PCA
    readkey = 0xFF;
    i2c_wr_reg(I2C_PCA_ADDR, pca_reg_configuration, (unsigned char *)&readkey, 1);
    i2c_wr_reg(I2C_PCA_ADDR, pca_reg_polarity_inversion, (unsigned char *)&readkey, 1);
    i2c_rd_reg(I2C_PCA_ADDR, pca_reg_input_port, (unsigned char *)&readkey, 1);
#else	
    KEY_PORT->SEL0 &= ~KEY_MASK;
    KEY_PORT->SEL1 &= ~KEY_MASK;    // 1) configure P1.4 and P1.1 as GPIO
    KEY_PORT->DIR &= ~KEY_MASK;     // 2) make P1.4 and P1.1 in
    KEY_PORT->REN |= KEY_MASK;      // 3) enable pull resistors on P1.4 and P1.1
    KEY_PORT->OUT |= KEY_MASK;      //    P1.4 and P1.1 are pull-up
#endif
}

unsigned int kbdread(void) {
    static unsigned int repeated, prevkey;

    Rand();
    BlueTooth_parse();
#ifdef PCA
    if (i2c_rd(I2C_PCA_ADDR, (unsigned char *)&readkey, 1)) {
        //		LaunchPad_LED(1);
        readkey = (kbddelay) ? (prevkey & 0xFF) : 0 ;
    }
    readkey |= (LaunchPad_Input() << 8);
#else
    readkey = ((KEY_PORT->IN & KEY_MASK) ^ KEY_MASK) | (LaunchPad_Input() << 8);
#endif
    if (readkey == 0) {  // not pressed
        kbddelay = 0;
        repeated = 0;
        prevkey = 0;
        return 0;
    } else {
        if (prevkey != readkey){ // now pressed
            prevkey = readkey;
            repeated = 0;
            kbddelay = DEBOUNCE_DELAY;
            return 0;
        } else {
            if (readkey == prevkey) {
                if (kbddelay == 0) {
                    if (repeated) {
                        kbddelay = REPEAT_PERIOD;
                        return readkey | HOLDED;
                    }
                    else {
                        kbddelay = REPEAT_DELAY;
                        repeated = 1;
                        return readkey;
                    }
                }
            }
        }
    }
    return 0;
}
