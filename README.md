# lcd_drivers
A couple drivers for LCDs built around on the HDD44780 chip <br>

The basic_driver/ folder is an example for the STM32F030 microcontroller <br>
The basic_driver_hl/ folder is a "higher level" version of basic_driver/ in a way that you can use pins from different GPIO ports <br>
The basic_driver_i2c/ folder is an example for the STM32F746 microcontroller which uses the I2C2 peripheral. It's made to work with PCF8574 based LCD modules
