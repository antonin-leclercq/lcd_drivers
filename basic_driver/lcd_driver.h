/*
 * lcd_header.h
 *
 *  Created on: 23 avr. 2022
 *      Author: LECLERCQ Antonin
 */

#ifndef INC_LCD_DRIVER_H_
#define INC_LCD_DRIVER_H_

#include "stm32f0xx.h"

/* For 16x2 LCD */
#ifdef LCD_4BIT_MODE
	#define LCD_4BIT_ENABLE					0b00100000 // 0x20
	#define LCD_FUNCTION_SET_COMMAND 		0b00101000 // 0x28 : 4 bit mode, 2 lines, 5x8 matrix
#endif

#ifdef LCD_8BIT_MODE
	#define LCD_FUNCTION_SET_COMMAND 		0b00111000 // 0x38 : 8 bit mode, 2 lines, 5x8 matrix
#endif

#if !(defined LCD_8BIT_MODE || defined LCD_4BIT_MODE)
	#error Please choose a correct LCD mode : either LCD_4BIT_MODE or LCD_8BIT_MODE
#endif

#define LCD_CLEAR_COMMAND 				0b00000001 // 0x01
#define LCD_SCREEN_CURSOR_ON_COMMAND	0b00001100 // 0x0C
#define LCD_AUTO_INCREMENT_NO_SHIFT		0b00000110 // 0x06
#define LCD_SCREEN_CURSOR_BLINK_COMMAND	0b00001111 // 0x0F
#define LCD_CURSOR_HOME_COMMAND			0b00000010 // 0x02
#define LCD_CURSOR_LINE_ONE_COMMAND		0b10000000 // 0x80

void LCD_init(void);

void LCD_send_command(const uint8_t command);
void LCD_send_data(const uint8_t data);

void LCD_print_string(const char* str);
void LCD_set_cursorXY(const uint8_t row, uint8_t col);

/* For basic delay timer */
void TIMER_delay_init(void);
void TIMER_delay_ms(const uint16_t ms);

static inline void LCD_send_command_pulse(void)
{
	/* Set RS to low and enable to high */
	GPIOC->ODR &= ~GPIO_ODR_1;
	GPIOC->ODR |= GPIO_ODR_0;

	/* Wait a little */
	TIMER_delay_ms(1);

	/* Set enable to low */
	GPIOC->ODR &= ~GPIO_ODR_0;

#if defined LCD_4BIT_MODE
	/* Clear previous data from ODR */
	GPIOB->ODR &= ~0b1111;
#elif defined LCD_8BIT_MODE
	/* Clear previous data from ODR */
	GPIOB->ODR &= ~0b11111111;
#endif
}

static inline void LCD_send_data_pulse(void)
{
	/* Set RS and enable to high */
	GPIOC->ODR |= GPIO_ODR_0 | GPIO_ODR_1;

	/* Wait a little */
	TIMER_delay_ms(1);

	/* Set enable to low */
	GPIOC->ODR &= ~GPIO_ODR_0;

#if defined LCD_4BIT_MODE
	/* Clear previous data from ODR */
	GPIOB->ODR &= ~0b1111;
#elif defined LCD_8BIT_MODE
	/* Clear previous data from ODR */
	GPIOB->ODR &= ~0b11111111;
#endif
}

#endif /* INC_LCD_DRIVER_H_ */
