# lcd_drivers
A couple drivers for LCDs built around on the [HD44780](https://www.sparkfun.com/datasheets/LCD/HD44780.pdf) chip <br>

The `basic_driver/` folder is an example for the STM32F030 microcontroller (running at 8MHz). It takes advantage of GPIO banks/ports for "more efficient" code. <br>
The `basic_driver_hl/` folder is a "higher level" version of `basic_driver/` in a way that you can use pins from different GPIO ports <br>
The `basic_driver_i2c/` folder is an example for the STM32F746 microcontroller which uses the I2C2 peripheral. It's made to work with [PCF8574](https://www.ti.com/lit/ds/symlink/pcf8574.pdf?ts=1708409217795) based LCD modules.
