# RSLK-MAX_Line_Maze
RSLK-MAX Competitive Robot

This robot based on http://TI.com/RSLK learning kit and curriculum. Intended to participate in Line Maze Solve competitions. Example rules like: https://robotic.tsi.lv/robotic/rules/labyrinth
Of course, there is some addition necessary, to work:
1. SPI EEPROM to hold configuration parameters, map, path (in my robot I use S25FL204K). Connect to port P10.0-P10.3
2. Color sensor module, because our competitions define start as green cell, and finish - red cell. Possible i2c module VEML6040 or TCS34725.
3. 3 buttons connected thru i2c GPIO extender PCA9536 and SPI OLED 128x64 display (as original RSLK intended)
4. (optional) BGX13 to get telemetry thru BLE.
5. On RSLK-MAX board add 1/4 resistor divider to control battery voltage. Using on board "spare" op.amplifier in voltage follower mode. https://github.com/vbirjuko/RSLK-MAX_Line_Maze/blob/main/IMG/IMG_20211219_135045_223.jpg
6. And some mechanical modifications: install front ball caster too and mount line reflection sensor board in front of this ball caster.


There is WiKi, in russian.

And small video https://youtu.be/V5qwzTzCMWw
