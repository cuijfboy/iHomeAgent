#ifndef AGENT_H__
#define AGENT_H__

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "nrf24le1.h"

#define COMMON_ADDR					0x00

#define UART_ADDR						0xFF
#define UART_PIP						0xFF
#define UART_SEQ 						0xFF

#define MY_ADDR							0x0A

#define UART_DIR_MASK       0xE7
#define PORT_0              P0
#define PORT_0_DIR          P0DIR
#define PORT_0_MASK         0xE7
#define PORT_1              P1
#define PORT_1_DIR          P1DIR
#define PORT_1_MASK         0x7F

void agent_init(struct uart_msg_fifo *uart_msg_rx_fifo_tmp,
								struct uart_msg_fifo *uart_msg_tx_fifo_tmp,
								struct nrf_msg_fifo *nrf_msg_rx_fifo_tmp,
								struct nrf_msg_fifo *nrf_msg_tx_fifo_tmp);

void agent_process(void);

#endif // AGENT_H__