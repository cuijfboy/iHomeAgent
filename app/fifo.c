#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "fifo.h"

void uart_msg_fifo_init(struct uart_msg_fifo *fifo)
{
	fifo->head = 0;
	fifo->tail = 0;
	fifo->length = 0;
}

bool uart_msg_fifo_empty(struct uart_msg_fifo *fifo)
{
	return fifo->length <= 0;
}

bool uart_msg_fifo_full(struct uart_msg_fifo *fifo)
{
	return fifo->length >= FIFO_SIZE;
}

bool uart_msg_fifo_offer(struct uart_msg_fifo *fifo, uint8_t* src, uint8_t len)
{
	if(fifo->length >= FIFO_SIZE || len > MSG_SIZE)
		return false;
	
	fifo->msg_len[fifo->tail] = len;
	memcpy(fifo->msg[fifo->tail], src, len); 
	
	fifo->tail = (fifo->tail + 1) % FIFO_SIZE;
	fifo->length++;
	return true;
}

bool uart_msg_fifo_poll(struct uart_msg_fifo *fifo, uint8_t* dst, uint8_t* len)
{
	if(fifo->length <= 0 )
		return false;
	
	*len = fifo->msg_len[fifo->head];
	memcpy(dst, fifo->msg[fifo->head], *len); 
	
	fifo->head = (fifo->head + 1) % FIFO_SIZE;
	fifo->length--;
	return true;
}

void nrf_msg_fifo_init(struct nrf_msg_fifo *fifo)
{
	fifo->head = 0;
	fifo->tail = 0;
	fifo->length = 0;
}

bool nrf_msg_fifo_empty(struct nrf_msg_fifo *fifo)
{
	return fifo->length <= 0;
}

bool nrf_msg_fifo_full(struct nrf_msg_fifo *fifo)
{
	return fifo->length >= FIFO_SIZE;
}

bool nrf_msg_fifo_offer(struct nrf_msg_fifo *fifo, uint8_t* src, uint8_t len, uint8_t pip)
{
	if(fifo->length >= FIFO_SIZE || len > MSG_SIZE || pip > 5 || pip < 0)
		return false;
	
	fifo->msg_pip[fifo->tail] = pip;
	fifo->msg_len[fifo->tail] = len;
	memcpy(fifo->msg[fifo->tail], src, len); 
	
	fifo->tail = (fifo->tail + 1) % FIFO_SIZE;
	fifo->length++;
	return true;
}

bool nrf_msg_fifo_poll(struct nrf_msg_fifo *fifo, uint8_t* dst, uint8_t* len, uint8_t* pip)
{
	if(fifo->length <= 0)
		return false;
	
	*pip = fifo->msg_pip[fifo->head];
	*len = fifo->msg_len[fifo->head];
	memcpy(dst, fifo->msg[fifo->head], *len); 
	
	fifo->head = (fifo->head + 1) % FIFO_SIZE;
	fifo->length--;
	return true;
}
