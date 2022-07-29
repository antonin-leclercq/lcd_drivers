/*
 * lcd_driver_hl.h
 *
 *  Created on: 2 mai 2022
 *      Author: LECLERCQ Antonin
 */

#ifndef INC_LCD_DRIVER_HL_H_
#define INC_LCD_DRIVER_HL_H_

#include "stm32f0xx.h"

#if defined LCD_4BIT_MODE
	#define DATA_LINES 4
	#define LCD_4BIT_ENABLE					0b00100000 // 0x20
	#define LCD_FUNCTION_SET_COMMAND 		0b00101000 // 0x28 : 4 bit mode, 2 lines, 5x8 matrix
#elif defined LCD_8BIT_MODE
	#define DATA_LINES 8
	#define LCD_FUNCTION_SET_COMMAND 		0b00111000 // 0x38 : 8 bit mode, 2 lines, 5x8 matrix
#else
	#error Please choose a correct LCD mode : either LCD_4BIT_MODE or LCD_8BIT_MODE
#endif

#define GPIO_MODER_OUT 					0x01UL
#define GPIO_MODER_MSK 					0x03UL
#define GPIO_OSPEEDR_MSK				0x03UL
#define GPIO_OSPEEDR_MEDIUM				0x01UL
#define GPIO_PIN_ADDRESS_OFFSET 		2U

#define LCD_CLEAR_COMMAND 				0b00000001 // 0x01
#define LCD_SCREEN_CURSOR_ON_COMMAND	0b00001100 // 0x0C
#define LCD_AUTO_INCREMENT_NO_SHIFT		0b00000110 // 0x06
#define LCD_SCREEN_CURSOR_BLINK_COMMAND	0b00001111 // 0x0F
#define LCD_CURSOR_HOME_COMMAND			0b00000010 // 0x02
#define LCD_CURSOR_LINE_ONE_COMMAND		0b10000000 // 0x80

typedef struct
{
	GPIO_TypeDef* gpio_port;
	uint8_t pin_n;
} gpio_pin ;

typedef struct
{
	gpio_pin EN;
	gpio_pin RS;
	gpio_pin DATA[DATA_LINES];
} LCD_TypeDef ;

void LCD_init(LCD_TypeDef* lcd);

void LCD_send_command(LCD_TypeDef* lcd, const uint8_t command);
void LCD_send_data(LCD_TypeDef* lcd, const uint8_t data);

void LCD_print_string(LCD_TypeDef* lcd, const char* str);
void LCD_set_cursorXY(LCD_TypeDef* lcd, const uint8_t row, const uint8_t col);

/* For basic delay timer */
void TIMER_delay_init(void);
void TIMER_delay_ms(const uint16_t ms);

static inline void LCD_send_command_pulse(LCD_TypeDef* lcd)
{
	/* Set RS to low and enable to high */
	lcd->RS.gpio_port->ODR &= ~(0x01UL << lcd->RS.pin_n);
	lcd->EN.gpio_port->ODR |= (0x01UL << lcd->EN.pin_n);

	/* Wait a little */
	TIMER_delay_ms(5);

	/* Set enable to low */
	lcd->EN.gpio_port->ODR &= ~(0x01UL << lcd->EN.pin_n);

	/* Clear previous data from ODR */
	for(uint8_t i = 0; i < DATA_LINES; i++)
	{
		lcd->DATA[i].gpio_port->ODR &= ~(0x01UL << lcd->DATA[i].pin_n);
	}
}

static inline void LCD_send_data_pulse(LCD_TypeDef* lcd)
{
	/* Set RS to low and enable to high */
	lcd->RS.gpio_port->ODR |= (0x01UL << lcd->RS.pin_n);
	lcd->EN.gpio_port->ODR |= (0x01UL << lcd->EN.pin_n);

	/* Wait a little */
	TIMER_delay_ms(5);

	/* Set enable to low */
	lcd->EN.gpio_port->ODR &= ~(0x01UL << lcd->EN.pin_n);

	/* Clear previous data from ODR */
	for(uint8_t i = 0; i < DATA_LINES; i++)
	{
		lcd->DATA[i].gpio_port->ODR &= ~(0x01UL << lcd->DATA[i].pin_n);
	}
}

#endif /* INC_LCD_DRIVER_HL_H_ */
