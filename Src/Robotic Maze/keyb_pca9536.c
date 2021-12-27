#include "keyboard.h"
#include "stdint.h"
#include "LaunchPad.h"
#include "i2c_drv.h"
#include "msp.h"

volatile unsigned int kbddelay;
unsigned int readkey;

void kbd_init(void) {
}

unsigned int kbdread(void) {
  static unsigned int repeated, prevkey;

  readkey = i2c_rd_reg(I2C_PCA_ADDR, input_port, (unsigned char *)&readkey, 1);
  if (readkey == 0) {  // not pressed
    repeated = 0;
    prevkey = 0;
    kbddelay = 0;
    return 0;
  } else {
    if (prevkey != readkey){ // now pressed
      prevkey = readkey;
      repeated = 0;
      kbddelay = DEBOUNCE_DELAY;
      return 0;
    }
    else {
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
