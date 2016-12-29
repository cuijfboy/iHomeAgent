#include <stdio.h>
#include <string.h>
#include "hal_uart.h"
#include "fifo.h"
#include "uart.h"

#define UART_KEY_START      0x55
#define UART_KEY_ESCAPE     0xAA
#define UART_STA_SIZE       0x01
#define UART_STA_BODY       0x02
#define UART_STA_DONE       0x04

uint8_t uart_char_cnt = 0, uart_char_pre = 0, uart_char_curr = 0;
uint8_t uart_msg[MSG_SIZE], uart_msg_size = 0;
uint8_t uart_msg_state = 0, uart_msg_len = 0, *uart_msg_ptr;

uint8_t* uart_msg_put_char(uint8_t ch, uint8_t *msg, uint8_t *len);

void uart_put_string(char *str)
{
	while(*str != 0)
		hal_uart_putchar(*str++);
}

void uart_put_data(uint8_t *dat, uint8_t len)
{
	while(len--)
		hal_uart_putchar(*dat++);
}

void uart_rx_msg(struct uart_msg_fifo *fifo)
{
	uart_char_cnt = hal_uart_chars_available();
	while(uart_char_cnt)
	{
		uart_char_cnt--;
		uart_char_pre = uart_char_curr;
		uart_char_curr = hal_uart_getchar();
		
		if(uart_char_pre != UART_KEY_ESCAPE && uart_char_curr == UART_KEY_ESCAPE)
			continue;
		
		if(uart_char_pre != UART_KEY_ESCAPE && uart_char_curr == UART_KEY_START)
		{
			uart_msg_ptr = uart_msg;
			uart_msg_size = 0;
			uart_msg_len = 0;
			uart_msg_state = UART_STA_SIZE;
			continue;
		}
		
		if(uart_msg_state == UART_STA_SIZE)
		{
			uart_msg_size = uart_char_curr;
			uart_msg_len = 0;
			uart_msg_state = UART_STA_BODY;
			continue;
		}
		
		if(uart_msg_state == UART_STA_BODY)
		{
			*uart_msg_ptr++ = uart_char_curr;
			if(++uart_msg_len == uart_msg_size)
			{
				uart_msg_state = UART_STA_DONE;
				uart_msg_fifo_offer(fifo, uart_msg, uart_msg_size);
				//uart_put_data(uart_msg, uart_msg_size);
			}                
			continue;
		}
	
	}
}

void uart_tx_msg(struct uart_msg_fifo *fifo)
{
	uint8_t msg[MSG_SIZE], len, uart_msg[MSG_SIZE], uart_msg_len;
	
	while(hal_uart_tx_complete() && uart_msg_fifo_poll(fifo, msg, &len))
	{
		uart_new_msg(msg, len, uart_msg, &uart_msg_len);
		uart_put_data(uart_msg, uart_msg_len);
	}
}

void uart_new_msg(uint8_t *src, uint8_t src_len, 
									uint8_t *dst, uint8_t *dst_len)
{
	uint8_t *len_ptr;
	
	*dst++ = UART_KEY_START;
	*dst_len = 1;
	
	len_ptr = dst;
	
	dst = uart_msg_put_char(src_len, dst, dst_len);
	
	while(src_len--)
		dst = uart_msg_put_char(*src++, dst, dst_len);
	
	*len_ptr = *dst_len - 2;
}

uint8_t* uart_msg_put_char(uint8_t ch, uint8_t *msg, uint8_t *len)
{
	if(ch == UART_KEY_START || ch == UART_KEY_ESCAPE)
	{
		*msg++ = UART_KEY_ESCAPE;
		(*len)++;
	}
	*msg++ = ch;
	(*len)++;
	
	return msg;
}

