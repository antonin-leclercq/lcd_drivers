/*
 * lcd_driver_i2c.c
 *
 *  Created on: 2 juil. 2022
 *      Author: LECLERCQ Antonin
 */

#include "lcd_driver_i2c.h"

void LCD_init_i2c(void)
{

/*
 * I2C on pins PF0 (SDA), PF1 (SCL) --> I2C2 peripheral
 * PF0 and PF1 on alternate function 4
 * I2C2(CLK) = PCLK1 = 54MHz (from CubeMX)
 *
 * I2C Interface Timing Requirements (from PCF8574 data sheet):
 *	- f(SCL) = 100kHz
 *	- 4µs clock high time, 4.7µs clock low time (choose 5µs)
 *	- I2C slave 7 bit write address = 0x40
 *	- I2C slave 7 bit read address = 0x41
 *
 *	Setup LCD in 4 bit mode because we only have 8 available data lines
 *
 */

/*********************************************** GPIO initialization ******************************************************/
	// Enable GPIOF clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;

	// Set PF0 and PF1 on alternate function mode
	GPIOF->MODER &= ~(GPIO_MODER_MODER0_Msk | GPIO_MODER_MODER1_Msk);
	GPIOF->MODER |= (0x02 << GPIO_MODER_MODER0_Pos) | (0x02 << GPIO_MODER_MODER1_Pos);

	// Set open-drain mode for PF0 and PF1 (required for I2C communication)
	GPIOF->OTYPER |= GPIO_OTYPER_OT0 | GPIO_OTYPER_OT1;

	// Set high speed mode for PF0 and PF1
	GPIOF->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR0_Msk | GPIO_OSPEEDER_OSPEEDR1_Msk);
	GPIOF->OSPEEDR |= (0x02 << GPIO_OSPEEDER_OSPEEDR0_Pos) | (0x02 << GPIO_OSPEEDER_OSPEEDR1_Pos);

	// Set PF0 and PF1 to alternate function 4 (I2C)
	GPIOF->AFR[0] &= ~(GPIO_AFRL_AFRL0_Msk | GPIO_AFRL_AFRL1_Msk);
	GPIOF->AFR[0] |= (0x04 << GPIO_AFRL_AFRL0_Pos) | (0x04 << GPIO_AFRL_AFRL1_Pos);

/*********************************************** I2C2 initialization ******************************************************/

	/* Remove comment to select SYSCLK (216 MHz) as I2C clock input
	 * (make sure to update I2C_SCLH, I2C_SCLL and I2C_PRESCALER if you do so)
	 *
	 * RCC->DCKCFGR2 |= (0x01 << RCC_DCKCFGR2_I2C2SEL_Pos);
	 *
	 *
	 * */

	// Enable I2C2 clock
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;

	// Make sure I2C2 is disabled
	I2C2->CR1 &= ~I2C_CR1_PE;

	// Reset I2C2 configuration
	I2C2->CR1 = 0x00000000;
	I2C2->CR2 = 0x00000000;
	I2C2->TIMINGR = 0x00000000;

	// Set I2C2 timing characteristics
	I2C2->TIMINGR |= ((I2C_PRESCALER -1) << I2C_TIMINGR_PRESC_Pos); // Clock pre-scaler is 5, f(PRESC) = f(I2C2) / 5 = 54 / 5 = 10.8MHz
	I2C2->TIMINGR |= ((I2C_SCLH -1) << I2C_TIMINGR_SCLH_Pos); // 50% duty cycle, 5µs high time
	I2C2->TIMINGR |= ((I2C_SCLL -1) << I2C_TIMINGR_SCLL_Pos);

	// Enable I2C1
	I2C2->CR1 |= I2C_CR1_PE;

	// Set slave address
	I2C2->CR2 &= ~I2C_CR2_SADD_Msk;
	I2C2->CR2 |= ((SLAVE_ADDR << 1U) << I2C_CR2_SADD_Pos);

/*********************************************** LCD initialization ******************************************************/

	// GPIO PB0, PB7 and PB14 for I2C status (for debug purposes)
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
	GPIOB->MODER &= ~(GPIO_MODER_MODER14_Msk | GPIO_MODER_MODER0_Msk | GPIO_MODER_MODER7_Msk);
	GPIOB->MODER |= (0x01 << GPIO_MODER_MODER14_Pos) | (0x01 << GPIO_MODER_MODER0_Pos) | (0x01 << GPIO_MODER_MODER7_Pos);

	// HDD44780 requires a delay after power up (100ms should be plenty enough)
	TIMER_delay_ms(100);

	/*
	 * Display only works in 4 bit mode because there are only 8 data lines
	 */
	LCD_send_command_i2c(lcd_set_to_4bit_sequence[0]);
	TIMER_delay_ms(100);
	LCD_send_command_i2c(lcd_set_to_4bit_sequence[1]);
	TIMER_delay_ms(100);
	LCD_send_command_i2c(lcd_set_to_4bit_sequence[2]);
	TIMER_delay_ms(100);

	// Send 0x28 (function set)
	LCD_send_command_i2c(FUNCTION_SET);
	TIMER_delay_ms(100);
}

void LCD_send_data_i2c(const uint8_t data)
{
	const uint8_t data_array[6] =
	{
			// Upper nibble
			(((data >> 4) & 0x0F) << 4) | BACKLIGHT | REG_SEL,
			(((data >> 4) & 0x0F) << 4) | BACKLIGHT | REG_SEL | EN_LCD,
			(((data >> 4) & 0x0F) << 4) | BACKLIGHT | REG_SEL,

			// Lower nibble
			((data & 0x0F) << 4) | BACKLIGHT | REG_SEL,
			((data & 0x0F) << 4) | BACKLIGHT | REG_SEL | EN_LCD,
			((data & 0x0F) << 4) | BACKLIGHT | REG_SEL,
	};
	status_led(LCD_write_packet_i2c(data_array, 6));
}

void LCD_send_command_i2c(const uint8_t command)
{
	const uint8_t command_array[6] =
	{
			// Upper nibble
			(((command >> 4) & 0x0F) << 4) | BACKLIGHT,
			(((command >> 4) & 0x0F) << 4) | BACKLIGHT | EN_LCD,
			(((command >> 4) & 0x0F) << 4) | BACKLIGHT,

			// Lower nibble
			((command & 0x0F) << 4) | BACKLIGHT,
			((command & 0x0F) << 4) | BACKLIGHT | EN_LCD,
			((command & 0x0F) << 4) | BACKLIGHT,
	};
	status_led(LCD_write_packet_i2c(command_array, 6));
}

const I2C_STATUS LCD_write_packet_i2c(const uint8_t* data, const uint8_t data_len)
{
	/*
	 * No register address on the PCF8574, write directly after sending device address (0x27)
	 */

	uint8_t* buffer = (uint8_t*) data;
	uint32_t timeout = 100000;

	// Set I2C in Write mode
	I2C2->CR2 &= ~I2C_CR2_RD_WRN_Msk;

	// Transfer data_len bytes
	I2C2->CR2 &= ~I2C_CR2_NBYTES_Msk;

	// Set number of bytes to send
	I2C2->CR2 |= ((data_len) << I2C_CR2_NBYTES_Pos);

	// Enable Auto-end
	I2C2->CR2 |= I2C_CR2_AUTOEND;

	// Clear STOPF flag
	I2C2->ICR |= I2C_ICR_STOPCF;

	// Start I2C transaction
	I2C2->CR2 |= I2C_CR2_START;

	for (int n = 0; n < data_len; n++, buffer++)
	{
		// Wait for TXIS with timeout
		while ((I2C2->ISR & I2C_ISR_TXIS) != I2C_ISR_TXIS)
		{
			timeout--;
			if (timeout == 0) return TX_ERR;
		}

		// Send data
		I2C2->TXDR = *buffer;
	}

	// Wait for STOPF with timeout
	timeout = 100000;
	while ((I2C2->ISR & I2C_ISR_STOPF) != I2C_ISR_STOPF)
	{
		timeout--;
		if (timeout == 0) return STOPF_ERR;
	}

	// Return success
	return NO_ERR;
}

void LCD_print_string_i2c(const char* str)
{
	for(char* str_ptr = (char*)str; *str_ptr != '\0'; str_ptr++)
	{
		LCD_send_data_i2c((uint8_t)*str_ptr);
	}
}

void LCD_set_cursorXY_i2c(const uint8_t row, const uint8_t column)
{
	switch (row)
	{
	case 0:
		LCD_send_command_i2c(column | 0x80);
		return;
	case 1:
		LCD_send_command_i2c(column | 0xC0);
		return;
	case 2:
		LCD_send_command_i2c(column | 0x94);
		return;
	case 3:
		LCD_send_command_i2c(column | 0xD4);
		return;
	default:
		return;
	}
}

void TIMER_delay_init(void)
{
	/*
	 * APB1 timer clock is double that of the peripheral clock
	 * APB1 timer clock is 108 MHz
	 * F(counter) = F(APB1) / (pre-scaler + 1) = 108MHz / (107999 + 1) = 1KHz
	 */

	// Enable TIM6 clock
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;

	// Reset TIM6 configuration
	TIM6->CR1 = 0x0000;
	TIM6->CR2 = 0x0000;

	// Enable auto-reload pre-load
	TIM6->CR1 |= TIM_CR1_ARPE;

	// Set pre-scaler value for 1KHz counting frequency
	TIM6->PSC = (uint16_t) 108000 -1;

	// Leave ARR to default value
	TIM6->ARR = (uint16_t) 0xFFFF;

	// Make sure to disable TIM6
	TIM6->CR1 &= ~TIM_CR1_CEN;
}

void TIMER_delay_ms(uint32_t ms)
{
	// Reset the timer
	TIM6->EGR |= TIM_EGR_UG;

	// Start the timer
	TIM6->CR1 |= TIM_CR1_CEN;

	// Wait
	while(TIM6->CNT < ms);

	// Stop the timer
	TIM6->CR1 &= ~TIM_CR1_CEN;
}
