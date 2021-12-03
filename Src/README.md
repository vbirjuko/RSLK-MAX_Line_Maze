To compile this project You must define some symbols:

<b>RSLK_MAX</b> - mandatory ;-). Because this code was started with Classic RSLK and has old port and pin definitions

<b>SSD1306</b> or <b>SH1106</b> - depends of type of your OLED display. SSD1306 is preffered.

<b>UPSIDEDOWN</b> - display orientation. Use it if display show picture in wrong orientation

<b>PCA</b> - use keys conected thru PCA9536. If not - expects keys connected to MC pins, but this will conflict with SPI eeprom. So, mandatory too.

<b>WITH_BGX</b> - optional if you have BGX13 connected to EUSCI_A2. and define and initialize pin where BGX conection status connected.

<b>BLINKER_SEGMENT</b> or <b>BLINKER_MOTOR</b> - what should show blinker LEDs.<br>
<li><b>BLINKER_MOTOR</b>
- front yellow signalize motor STOP state<br>
- rear red if motor powered backward.
                
<li><b>BLINKER_SEGMENT</b> - what speed is selected running on maze segment. Just for debug purposes.
  
