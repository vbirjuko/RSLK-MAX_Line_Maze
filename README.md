# RSLK-MAX_Line_Maze
RSLK-MAX Competitive Robot

This robot based on http://TI.com/RSLK learning kit and curriculum. Intended to participate in Line Maze Solve competitions. Example rules like: https://robotic.tsi.lv/robotic/rules/labyrinth
Of course, there is some addition necessary, to work:
1. SPI EEPROM to hold configuration parameters, map, path (in my robot I use S25FL204K).
2. Color sensor module, because our competitions define start as green cell, and finish - red cell. Possible i2c module VEML6040 or TCS34725.
3. 3 buttons connected thru i2c GPIO extender PCA9536 and OLED display (as originally intended)
4. BGX13 (optional) to get telemetry thru BLE.
5. On RSLK-MAX board add 1/4 resistor divider to control battery voltage. Using on board "spare" op.amplifier in voltage follower mode.
6. And some mechanical modifications: install front ball caster too and mount line reflection sensor board in front of this ball caster.
