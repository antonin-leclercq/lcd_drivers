/*
 * lcd_driver_i2c.h
 *
 *  Created on: 2 juil. 2022
 *      Author: LECLERCQ Antonin
 */

#ifndef INC_LCD_DRIVER_I2C_H_
#define INC_LCD_DRIVER_I2C_H_

#include "stm32f7xx.h"
#include "simple_delay.h"

#define I2C_PRESCALER 5
#define I2C_SCLH 53
#define I2C_SCLL 53

// Change slave address depending on your configuration
#define SLAVE_ADDR 0x27

#define BACKLIGHT 	0b00001000
#define REG_SEL   	0b00000001
#define READ_WRITE	0b00100010
#define EN_LCD	  	0b00000100

#define FUNCTION_SET		0b00101000
#define SCREEN_CURSOR_ON	0b00001100
#define CLEAR_SCREEN		0b00000001

typedef enum {
	NO_ERR = 0,
	TX_ERR = 1,
	STOPF_ERR = 2
} I2C_STATUS;

static const uint8_t lcd_set_to_4bit_sequence[3] = {
		0x33,
		0x32,
		0x20
};

void LCD_init_i2c(void);
void LCD_send_command_i2c(const uint8_t command);
void LCD_send_data_i2c(const uint8_t data);

const I2C_STATUS LCD_write_packet_i2c(const uint8_t* data, const uint8_t data_len);

void LCD_print_string_i2c(const char* str);
void LCD_set_cursorXY_i2c(const uint8_t row, const uint8_t column);

/* Basic delay functions*/
void TIMER_delay_init(void);
void TIMER_delay_ms(uint32_t ms);

static inline void status_led(const I2C_STATUS s)
{
	switch(s)
	{
	case NO_ERR:
		GPIOB->ODR |= GPIO_ODR_OD0;
		break;
	case TX_ERR:
		GPIOB->ODR |= GPIO_ODR_OD14;
		break;
	case STOPF_ERR:
		GPIOB->ODR |= GPIO_ODR_OD7;
		break;
	default:
		GPIOB->ODR &= ~(GPIO_ODR_OD14 | GPIO_ODR_OD7 | GPIO_ODR_OD0);
	}
}

#endif /* INC_LCD_DRIVER_I2C_H_ */
