#ifndef UART_H__
#define UART_H__

#include <stdio.h>
#include <stdint.h>
#include <string.h>

void uart_put_string(char *str);
void uart_put_data(uint8_t *dat, uint8_t len);

void uart_rx_msg(struct uart_msg_fifo *fifo);
void uart_tx_msg(struct uart_msg_fifo *fifo);
void uart_new_msg(uint8_t *src, uint8_t src_len, 
									uint8_t *dst, uint8_t *dst_len);

#endif // UART_H__