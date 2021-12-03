To compile this project You must define some predefined parameters:

RSLK_MAX - mandatory ;-). Because this code biginning with Classic RSLK and had old port and pin definitions

SSD1306 or SH1106 - depends of type of your OLED display. SSD1306 is preffered.

UPSIDEDOWN - display orientation

PCA - use keys conected thru PCA9536. If not - expects keys connected to MC pins, but this will conflict with SPI eeprom.

WITH_BGX - optional if you have BGX13 connected to EUSCI_A2.

BLINKER_SEGMENT or BLINKER_MOTOR - what should show blinker LEDs.
