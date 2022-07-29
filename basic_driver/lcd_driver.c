/*
 * lcd_driver.c
 *
 *  Created on: 23 avr. 2022
 *      Author: LECLERCQ Antonin
 */


#include "lcd_driver.h"

void LCD_init(void)
{
	/* LCD driver example for STM32F030 microcontroller
	 *
	 * NOTE: Change the pins according to your device
	 *
	 * 4 Bit mode :
	 * 		- EN pin is PC0, RS pin is PC1
	 * 		- D4 to D7 is PB0 to PB3
	 *
	 * 8 Bit mode:
	 * 		- EN pin is PC0, RS pin is PC1
	 * 		- D0 to D7 is PB0 to PB7
	 *
	 */

	/* Start GPIOC and GPIOB clock */
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIOBEN;

	/* Configure Pins EN and RS as output */
	GPIOC->MODER &= ~(GPIO_MODER_MODER0_Msk | GPIO_MODER_MODER1_Msk);
	GPIOC->MODER |= (0x01 << GPIO_MODER_MODER0_Pos) | (0x01 << GPIO_MODER_MODER1_Pos);

	/* Set Pins EN and RS speed to medium */
	GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR0_Msk | GPIO_OSPEEDR_OSPEEDR1_Msk);
	GPIOC->OSPEEDR |= (0x01 << GPIO_OSPEEDR_OSPEEDR0_Pos) | (0x01 << GPIO_OSPEEDR_OSPEEDR1_Pos);

#if defined LCD_4BIT_MODE
	/* Configure Pins as output */
	GPIOB->MODER &= ~(GPIO_MODER_MODER0_Msk | GPIO_MODER_MODER1_Msk | GPIO_MODER_MODER2_Msk | GPIO_MODER_MODER3_Msk);
	GPIOB->MODER |= (0x01 << GPIO_MODER_MODER0_Pos) | (0x01 << GPIO_MODER_MODER1_Pos) | (0x01 << GPIO_MODER_MODER2_Pos) | (0x01 << GPIO_MODER_MODER3_Pos);

	/* Set Pins speed to medium */
	GPIOB->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR0_Msk | GPIO_OSPEEDR_OSPEEDR1_Msk | GPIO_OSPEEDR_OSPEEDR2_Msk | GPIO_OSPEEDR_OSPEEDR3_Msk);
	GPIOB->OSPEEDR |= (0x01 << GPIO_OSPEEDR_OSPEEDR0_Pos) | (0x01 << GPIO_OSPEEDR_OSPEEDR1_Pos) | (0x01 << GPIO_OSPEEDR_OSPEEDR2_Pos) |
			(0x01 << GPIO_OSPEEDR_OSPEEDR3_Pos);

#elif defined LCD_8BIT_MODE
	/* Configure Pins as output */
	GPIOB->MODER &= ~(GPIO_MODER_MODER0_Msk | GPIO_MODER_MODER1_Msk | GPIO_MODER_MODER2_Msk | GPIO_MODER_MODER3_Msk | GPIO_MODER_MODER4_Msk |
			GPIO_MODER_MODER5_Msk | GPIO_MODER_MODER6_Msk | GPIO_MODER_MODER7_Msk);
	GPIOB->MODER |= (0x01 << GPIO_MODER_MODER0_Pos) | (0x01 << GPIO_MODER_MODER1_Pos) | (0x01 << GPIO_MODER_MODER2_Pos) | (0x01 << GPIO_MODER_MODER3_Pos) |
			(0x01 << GPIO_MODER_MODER4_Pos) | (0x01 << GPIO_MODER_MODER5_Pos) | (0x01 << GPIO_MODER_MODER6_Pos) | (0x01 << GPIO_MODER_MODER7_Pos);

	/* Set Pins speed to medium */
	GPIOB->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR0_Msk | GPIO_OSPEEDR_OSPEEDR1_Msk | GPIO_OSPEEDR_OSPEEDR2_Msk | GPIO_OSPEEDR_OSPEEDR3_Msk | GPIO_OSPEEDR_OSPEEDR4_Msk |
			GPIO_OSPEEDR_OSPEEDR5_Msk | GPIO_OSPEEDR_OSPEEDR6_Msk | GPIO_OSPEEDR_OSPEEDR7_Msk);
	GPIOB->OSPEEDR |= (0x01 << GPIO_OSPEEDR_OSPEEDR0_Pos) | (0x01 << GPIO_OSPEEDR_OSPEEDR1_Pos) | (0x01 << GPIO_OSPEEDR_OSPEEDR2_Pos) |
			(0x01 << GPIO_OSPEEDR_OSPEEDR3_Pos) | (0x01 << GPIO_OSPEEDR_OSPEEDR4_Pos) | (0x01 << GPIO_OSPEEDR_OSPEEDR5_Pos) | (0x01 << GPIO_OSPEEDR_OSPEEDR6_Pos) |
			(0x01 << GPIO_OSPEEDR_OSPEEDR7_Pos);
#endif


	/* Set PC0, PC1 and data pins low */
	LCD_send_data(0);
	GPIOC->ODR &= ~(GPIO_ODR_0 | GPIO_ODR_1);

#if defined LCD_4BIT_MODE
	/* 4 bit Initialization */
	LCD_send_command(0x33);
	LCD_send_command(0x32);
	LCD_send_command(LCD_4BIT_ENABLE);
#endif

	/* Send configuration */
	LCD_send_command(LCD_FUNCTION_SET_COMMAND);
	LCD_send_command(LCD_SCREEN_CURSOR_ON_COMMAND);
	LCD_send_command(LCD_AUTO_INCREMENT_NO_SHIFT);
	LCD_send_command(LCD_CLEAR_COMMAND);
	LCD_send_command(LCD_CURSOR_LINE_ONE_COMMAND);
}

void LCD_send_command(const uint8_t command)
{
#if defined LCD_4BIT_MODE
	/* Send Upper nibble first */
	GPIOB->ODR |= (command >> 4) & 0x0F;

	LCD_send_command_pulse();

	TIMER_delay_ms(1);

	/* Send Lower nibble then */
	GPIOB->ODR |= command & 0x0F;

	LCD_send_command_pulse();

	TIMER_delay_ms(1);

#elif defined LCD_8BIT_MODE
	GPIOB->ODR |= command;
	LCD_send_command_pulse();
	TIMER_delay_ms(1);
#endif
}

void LCD_send_data(const uint8_t data)
{
#if defined LCD_4BIT_MODE
	/* Send Upper nibble first */
	GPIOB->ODR |= (data >> 4) & 0x0F;

	LCD_DISPLAY_SEND_DATA_PROCEDURE();

	TIMER_delay_ms(1);

	/* Send Lower nibble then */
	GPIOB->ODR |= data & 0x0F;

	LCD_DISPLAY_SEND_DATA_PROCEDURE();

	TIMER_delay_ms(1);

#elif defined LCD_8BIT_MODE
	GPIOB->ODR |= data;
	LCD_send_data_pulse();
	TIMER_delay_ms(1);
#endif
}

void LCD_print_string(const char* str)
{
	char* str_ptr = (char*)str;
	while(*str_ptr != '\0')
	{
		LCD_send_data(*str_ptr++);
	}
}

void LCD_set_cursorXY(const uint8_t row, const uint8_t col)
{
	switch (row)
	{
	case 0:
		LCD_send_command(col | 0x80);
		return;
	case 1:
		LCD_send_command(col | 0xC0);
		return;
	case 2:
		LCD_send_command(col | 0x94);
		return;
	case 3:
		LCD_send_command(col | 0xD4);
		return;
	}
}

void TIMER_delay_init(void)
{
	/*
	 * Using Timer 6 (Basic timer) to handle delays
	 * 1kHz counting frequency
	 * Maximum delay of 65.535 s
	 * */

	/* Start TIM6 clock */
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;

	/* Reset Timer 6 configuration */
	TIM6->CR1 |= 0x0000;

	/* Auto-reload pre-load enable (timer is buffered) */
	TIM6->CR1 |= TIM_CR1_ARPE;

	/* Set the timer pre-scaler for 1kHz counting frequency */
	TIM6->PSC = (uint16_t)8000 - 1;

	/* Set the timer auto-reload to max (default) */
	TIM6->ARR = 0xFFFF;

	/* Disable Timer 6 for the moment */
	TIM6->CR1 &= ~TIM_CR1_CEN;
}

void TIMER_delay_ms(const uint16_t ms)
{
	/* Reset Timer 6 */
	TIM6->EGR |= TIM_EGR_UG;

	/* Enable Timer 6 */
	TIM6->CR1 |= TIM_CR1_CEN;

	/* Wait for timer to reach Ms */
	while(TIM6->CNT < ms);

	/* Disable Timer 6 */
	TIM6->CR1 &= ~TIM_CR1_CEN;
}
