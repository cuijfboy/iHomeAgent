#ifndef FIFO_H__
#define FIFO_H__

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#define FIFO_SIZE 		3
#define MSG_SIZE			32

struct uart_msg_fifo 
{
	uint8_t head;
	uint8_t tail;
	uint8_t length;
	uint8_t msg[FIFO_SIZE][MSG_SIZE];
	uint8_t msg_len[FIFO_SIZE];
};

struct nrf_msg_fifo 
{
	uint8_t head;
	uint8_t tail;
	uint8_t length;
	uint8_t msg[FIFO_SIZE][MSG_SIZE];
	uint8_t msg_len[FIFO_SIZE];
	uint8_t msg_pip[FIFO_SIZE];
};

void uart_msg_fifo_init(struct uart_msg_fifo *fifo);
bool uart_msg_fifo_empty(struct uart_msg_fifo *fifo);
bool uart_msg_fifo_full(struct uart_msg_fifo *fifo);
bool uart_msg_fifo_offer(struct uart_msg_fifo *fifo, uint8_t* src, uint8_t len);
bool uart_msg_fifo_poll(struct uart_msg_fifo *fifo, uint8_t* dst, uint8_t* len);

void nrf_msg_fifo_init(struct nrf_msg_fifo *fifo);
bool nrf_msg_fifo_empty(struct nrf_msg_fifo *fifo);
bool nrf_msg_fifo_full(struct nrf_msg_fifo *fifo);
bool nrf_msg_fifo_offer(struct nrf_msg_fifo *fifo, uint8_t* src, uint8_t len, uint8_t pip);
bool nrf_msg_fifo_poll(struct nrf_msg_fifo *fifo, uint8_t* dst, uint8_t* len, uint8_t* pip);

#endif // FIFO_H__