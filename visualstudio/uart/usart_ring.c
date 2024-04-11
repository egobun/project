/*
 * usart_ring.c
 *
 *  Created on: 19 авг. 2019 г.
 *      Author: dima
 */

#include "usart_ring.h"

extern UART_HandleTypeDef MYUART;

/////////////////// USART /////////////////////
volatile uint16_t rx_buffer_head = 0;
volatile uint16_t rx_buffer_tail = 0;
uint8_t rx_buffer[UART_RX_BUFFER_SIZE] = { 0, };

void clear_uart_buff()
{
	__HAL_UART_DISABLE_IT(&MYUART, UART_IT_RXNE);
	rx_buffer_head = 0;
	rx_buffer_tail = 0;
	__HAL_UART_ENABLE_IT(&MYUART, UART_IT_RXNE);
}

uint16_t uart_available(void)
{
	return ((uint16_t)(UART_RX_BUFFER_SIZE + rx_buffer_head - rx_buffer_tail)) % UART_RX_BUFFER_SIZE;
}

uint8_t uart_read(void)
{
	if (rx_buffer_head == rx_buffer_tail)
	{
		return 0;
	}
	else
	{
		uint8_t c = rx_buffer[rx_buffer_tail];
		rx_buffer_tail = (uint16_t)(rx_buffer_tail + 1) % UART_RX_BUFFER_SIZE;
		return c;
	}
}

void uart_receive(void)
{
	if (uart_available()) // есть ли что-то в приёмном буфере, тогда читаем
	{
		char str[SIZE_BF] = { 0, };
		uint8_t i = 0;

		while (uart_available())
		{
			str[i++] = uart_read(); // читаем байт

			if (i == SIZE_BF - 1)
			{
				str[i] = '\0';
				break;
			}

			//HAL_Delay(1);
		}

		str[i] = '\0';

		HAL_UART_Transmit(&MYUART, (uint8_t*)str, strlen(str), 100); // отправляем обратно что получили
	}
}