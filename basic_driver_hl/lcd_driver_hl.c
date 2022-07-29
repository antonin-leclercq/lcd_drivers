/*
 * lcd_driver_hl.c
 *
 *  Created on: 2 mai 2022
 *      Author: LECLERCQ Antonin
 */

#include <lcd_driver_hl.h>

void LCD_init(LCD_TypeDef* lcd)
{
	lcd->EN.gpio_port->MODER &= ~(GPIO_MODER_MSK << (lcd->EN.pin_n * GPIO_PIN_ADDRESS_OFFSET));
	lcd->RS.gpio_port->MODER &= ~(GPIO_MODER_MSK << (lcd->RS.pin_n * GPIO_PIN_ADDRESS_OFFSET));

	// Set as output
	lcd->EN.gpio_port->MODER |= (GPIO_MODER_OUT << (lcd->EN.pin_n * GPIO_PIN_ADDRESS_OFFSET));
	lcd->RS.gpio_port->MODER |= (GPIO_MODER_OUT << (lcd->RS.pin_n * GPIO_PIN_ADDRESS_OFFSET));

	lcd->EN.gpio_port->OSPEEDR &= ~(GPIO_OSPEEDR_MSK << (lcd->EN.pin_n * GPIO_PIN_ADDRESS_OFFSET));
	lcd->RS.gpio_port->OSPEEDR &= ~(GPIO_OSPEEDR_MSK << (lcd->RS.pin_n * GPIO_PIN_ADDRESS_OFFSET));

	// Set as Medium speed
	lcd->EN.gpio_port->OSPEEDR |= (GPIO_OSPEEDR_MEDIUM << (lcd->EN.pin_n * GPIO_PIN_ADDRESS_OFFSET));
	lcd->RS.gpio_port->OSPEEDR |= (GPIO_OSPEEDR_MEDIUM << (lcd->RS.pin_n * GPIO_PIN_ADDRESS_OFFSET));

	for(uint8_t i = 0; i < DATA_LINES; i++)
	{
		lcd->DATA[i].gpio_port->MODER &= ~(GPIO_MODER_MSK << (lcd->DATA[i].pin_n * GPIO_PIN_ADDRESS_OFFSET)); // Apply mode Mask
		lcd->DATA[i].gpio_port->MODER |= (GPIO_MODER_OUT << (lcd->DATA[i].pin_n * GPIO_PIN_ADDRESS_OFFSET)); // Set as output
		lcd->DATA[i].gpio_port->OSPEEDR &= ~(GPIO_OSPEEDR_MSK << (lcd->DATA[i].pin_n * GPIO_PIN_ADDRESS_OFFSET)); // Apply speed mask
		lcd->DATA[i].gpio_port->OSPEEDR |= (GPIO_OSPEEDR_MEDIUM << (lcd->DATA[i].pin_n * GPIO_PIN_ADDRESS_OFFSET)); // Set as output
	}

	/* Set every pin to low */
	LCD_send_data(lcd, 0);
	lcd->EN.gpio_port->ODR &= ~(0x01UL << lcd->EN.pin_n);
	lcd->RS.gpio_port->ODR &= ~(0x01UL << lcd->RS.pin_n);

#if defined LCD_4BIT_MODE
	/* 4 bit Initialization */
	LCD_send_command(lcd, 0x33);
	LCD_send_command(lcd, 0x32);
	LCD_send_command(lcd, LCD_4BIT_ENABLE);
#endif

	/* Send configuration */
	LCD_send_command(lcd, LCD_FUNCTION_SET_COMMAND);
	LCD_send_command(lcd, LCD_SCREEN_CURSOR_ON_COMMAND);
	LCD_send_command(lcd, LCD_AUTO_INCREMENT_NO_SHIFT);
	LCD_send_command(lcd, LCD_CLEAR_COMMAND);
	LCD_send_command(lcd, LCD_CURSOR_LINE_ONE_COMMAND);
}

void LCD_send_command(LCD_TypeDef* lcd, const uint8_t command)
{
#if defined LCD_4BIT_MODE
	uint8_t i = 0;
	/* Send Upper nibble first */
	for(; i < DATA_LINES; i++)
	{
		lcd->DATA[i].gpio_port->ODR |= (command >> 4) & (1 << i);
	}
	LCD_send_command_pulse(lcd);

	/* Send lower nibble then */
	for(i = 0; i < DATA_LINES; i++)
	{
		lcd->DATA[i].gpio_port->ODR |= (command & 0x0F) & (1 << i);
	}
	LCD_send_command_pulse(lcd);

#elif defined LCD_8BIT_MODE
	for(uint8_t i = 0; i < DATA_LINES; i++)
	{
		lcd->DATA[i].gpio_port->ODR |= command & (1 << i);
	}
	LCD_send_command_pulse(lcd);

#endif
}

void LCD_send_data(LCD_TypeDef* lcd, const uint8_t data)
{
#if defined LCD_4BIT_MODE
	uint8_t i = 0;
	/* Send Upper nibble first */
	for(; i < DATA_LINES; i++)
	{
		lcd->DATA[i].gpio_port->ODR |= (data >> 4) & (1 << i);
	}
	LCD_send_data_pulse(lcd);

	/* Send lower nibble then */
	for(i = 0; i < DATA_LINES; i++)
	{
		lcd->DATA[i].gpio_port->ODR |= (data & 0x0F) & (1 << i);
	}
	LCD_send_data_pulse(lcd);

#elif defined LCD_8BIT_MODE
	for(uint8_t i = 0; i < DATA_LINES; i++)
	{
		lcd->DATA[i].gpio_port->ODR |= data & (1 << i);
	}
	LCD_send_data_pulse(lcd);

#endif
}

void LCD_print_string(LCD_TypeDef* lcd, const char* str)
{
	char* str_ptr = (char*)str;
	while(*str_ptr != '\0')
	{
		LCD_send_data(lcd, *str_ptr++);
	}
}

void LCD_set_cursorXY(LCD_TypeDef* lcd, const uint8_t row, const uint8_t col)
{
	switch (row)
	{
	case 0:
		LCD_send_command(lcd, col | 0x80);
		return;
	case 1:
		LCD_send_command(lcd, col | 0xC0);
		return;
	case 2:
		LCD_send_command(lcd, col | 0x94);
		return;
	case 3:
		LCD_send_command(lcd, col | 0xD4);
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
