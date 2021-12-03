To compile this project You must define some symbols:

RSLK_MAX - mandatory ;-). Because this code was started with Classic RSLK and has old port and pin definitions

SSD1306 or SH1106 - depends of type of your OLED display. SSD1306 is preffered.

UPSIDEDOWN - display orientation. Use it if display show picture in wrong orientation

PCA - use keys conected thru PCA9536. If not - expects keys connected to MC pins, but this will conflict with SPI eeprom. So, mandatory too.

WITH_BGX - optional if you have BGX13 connected to EUSCI_A2. and define and initialize pin where BGX conection status connected.

BLINKER_SEGMENT or BLINKER_MOTOR - what should show blinker LEDs.<br>
<li>BLINKER_MOTOR - front yellow signalize motor STOP state
                - rear red if motor powered backward.
                
<li>BLINKER_SEGMENT - what speed is selected running on maze segment
  
