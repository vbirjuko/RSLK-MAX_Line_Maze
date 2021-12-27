#include <stm32f0xx.h>
#include "keyboard.h"
#include "i2c_drv.h"

volatile unsigned int kbddelay;

unsigned int kbdread(void) {
  unsigned int readkey;
	static unsigned char i2c_key;
  static unsigned int repeated, prevkey;

//	if (!flag.no_display) {
		if (i2c_rd(0x70, &i2c_key, 1) != I2C_SUCCESS) { 
			i2c_key = KEY_MASK;
		}
//	} else i2c_key = KEY_MASK;
	
  readkey = (((GPIOB->IDR) & KEY_MASKB) ^ KEY_MASKB) | ((i2c_key & KEY_MASK) ^ KEY_MASK) ; // | (((GPIOA->IDR) & KEY_MASKA) ^ KEY_MASKA );
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
