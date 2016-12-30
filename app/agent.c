#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "nrf24le1.h"
#include "agent.h"
#include "fifo.h"
#include "uart.h"
#include "hal_uart.h"
#include "hal_flash.h"
#include "gzll.h"

#define UART_ADDR						0xFF
#define UART_PIP						0x3F

#define MY_ADDR							0x0A

#define PMW 								0x10
#define NV_PAGE_ADDR				32
#define NV_ADDR_ADDR				0xFA00

#define STA_ACK_FLAG        0x80
#define STA_UART_FLAG				0x40
#define STA_PIP_MASK				0x3F

#define CMD_NOP             0x00
#define CMD_GET_PORT	    	0x01
#define CMD_SET_PORT	    	0x02
#define CMD_GET_DIR         0x03
#define CMD_SET_DIR         0x04
#define CMD_GET_ADDR   	    0x05
#define CMD_SET_ADDR   		  0x06
#define CMD_FWD							0x3F

#define RES_NOP							0x00
#define RES_FWD_DST					0x01
#define RES_FWD_UART				0x02
#define RES_REP_SRC					0x04

#define UART_MSG						0x01
#define NRF_MSG							0x02

static struct uart_msg_fifo 	*uart_msg_rx_fifo, *uart_msg_tx_fifo;
static struct nrf_msg_fifo 	*nrf_msg_rx_fifo, *nrf_msg_tx_fifo;

static uint8_t my_addr;

void agent_set_my_addr(void);
void agent_task(uint8_t type, uint8_t *in, uint8_t in_len);
uint8_t agent_action( uint8_t src, uint8_t dst, uint8_t sta, uint8_t seq,
											uint8_t *dat, uint8_t dat_len,
											uint8_t *out_dat, uint8_t* out_dat_len);

void agent_init(struct uart_msg_fifo *uart_msg_rx_fifo_tmp,
								struct uart_msg_fifo *uart_msg_tx_fifo_tmp,
								struct nrf_msg_fifo *nrf_msg_rx_fifo_tmp,
								struct nrf_msg_fifo *nrf_msg_tx_fifo_tmp)
{
	uart_msg_rx_fifo = uart_msg_rx_fifo_tmp;
	uart_msg_tx_fifo = uart_msg_tx_fifo_tmp;
	nrf_msg_rx_fifo = nrf_msg_rx_fifo_tmp;
	nrf_msg_tx_fifo = nrf_msg_tx_fifo_tmp;

	PCON &= ~PMW;
	my_addr = hal_flash_byte_read(NV_ADDR_ADDR);
	if(my_addr == 0xFF)
		my_addr = MY_ADDR;

	agent_set_my_addr();
}

void agent_set_my_addr(void)
{
	uint8_t tmp_addr[GZLL_ADDRESS_WIDTH] = GZLL_DEFAULT_ADDRESS_PIPE1;
  tmp_addr[0] = my_addr;
	hal_nrf_set_address(HAL_NRF_PIPE1, tmp_addr);
}

void agent_process(void)
{
	uint8_t msg[MSG_SIZE], len, pip;

	while(!uart_msg_fifo_empty(uart_msg_rx_fifo) && !nrf_msg_fifo_full(nrf_msg_tx_fifo))
	{
		uart_msg_fifo_poll(uart_msg_rx_fifo, msg, &len);
		agent_task(UART_MSG, msg, len);
		//nrf_msg_fifo_offer(nrf_msg_tx_fifo, msg, len, 0);
	}

	while(!nrf_msg_fifo_empty(nrf_msg_rx_fifo) && !uart_msg_fifo_full(uart_msg_tx_fifo))
	{
		nrf_msg_fifo_poll(nrf_msg_rx_fifo, msg, &len, &pip);
		//uart_put_data(msg, len);
		agent_task(NRF_MSG, msg, len);
	}
}

void agent_task(uint8_t type, uint8_t *in, uint8_t in_len)
{
	uint8_t pip, out[MSG_SIZE], out_len, *out_dat, out_dat_len, res;

	pip = in[2] & STA_PIP_MASK;
	out_dat = out + 4;

	// src, dst, sta, seq, dat, dat_len, out_dat, out_dat_len
	res = agent_action(in[0], in[1], in[2], in[3], in + 4, in_len - 4, out_dat, &out_dat_len);

	if(res == RES_NOP)
		return;

	if(res & RES_FWD_DST)
	{
		if(type == UART_MSG)
		{
			// in_src = my_addr
			in[0] = my_addr;
			// set uart flag to in_sta
			in[2] |= STA_UART_FLAG;
		}

		nrf_msg_fifo_offer(nrf_msg_tx_fifo, in, in_len, pip);
	}

	if(res & RES_FWD_UART)
		uart_msg_fifo_offer(uart_msg_tx_fifo, in, in_len);

	if(res & RES_REP_SRC)
	{
		// out_src = my_addr
		out[0] = my_addr;
		// out_dst = in_src
		out[1] = in[0];
		// out_sta = in_sta & ack
		out[2] = in[2] | STA_ACK_FLAG;
		// out_seq = in_seq
		out[3] = in[3];
		out_len = out_dat_len + 4;

		if(type == UART_MSG)
			uart_msg_fifo_offer(uart_msg_tx_fifo, out, out_len);
		else
			nrf_msg_fifo_offer(nrf_msg_tx_fifo, out, out_len, pip);
	}
}

uint8_t agent_action( uint8_t src, uint8_t dst, uint8_t sta, uint8_t seq,
											uint8_t *dat, uint8_t dat_len,
											uint8_t *out_dat, uint8_t* out_dat_len)
{
	if(dst != my_addr && (dst != UART_ADDR || src != UART_ADDR))
		return RES_FWD_DST;

	if(sta & STA_ACK_FLAG)
		if(sta & STA_UART_FLAG)
			return RES_FWD_UART;
		else
			return RES_NOP;

	if(dat[0] == CMD_FWD)
		return RES_FWD_UART;

	out_dat[0] = dat[0];

	switch(dat[0])
	{
		case CMD_NOP:
			*out_dat_len = dat_len;
			break;

		case CMD_SET_PORT:
			PORT_0 |= dat[1] &  dat[2] &  PORT_0_MASK;
			PORT_0 &= dat[1] | ~dat[2] | ~PORT_0_MASK;
			PORT_1 |= dat[3] &  dat[4] &  PORT_1_MASK;
			PORT_1 &= dat[3] | ~dat[4] | ~PORT_1_MASK;

		case CMD_GET_PORT:
			out_dat[1] = PORT_0      & dat[2];
			out_dat[2] = PORT_0_MASK & dat[2];
			out_dat[3] = PORT_1      & dat[4];
			out_dat[4] = PORT_1_MASK & dat[4];
			*out_dat_len = 5;
			break;

		case CMD_SET_DIR:
			PORT_0_DIR |= dat[1] &  dat[2] &  PORT_0_MASK;
			PORT_0_DIR &= dat[1] | ~dat[2] | ~PORT_0_MASK;
			PORT_1_DIR |= dat[3] &  dat[4] &  PORT_1_MASK;
			PORT_1_DIR &= dat[3] | ~dat[4] | ~PORT_1_MASK;

		case CMD_GET_DIR:
			out_dat[1] = PORT_0_DIR  & dat[2];
			out_dat[2] = PORT_0_MASK & dat[2];
			out_dat[3] = PORT_1_DIR  & dat[4];
			out_dat[4] = PORT_1_MASK & dat[4];
			*out_dat_len = 5;
			break;

		case CMD_SET_ADDR:
			PCON &= ~PMW;
			hal_flash_page_erase(NV_PAGE_ADDR);
			hal_flash_byte_write(NV_ADDR_ADDR, dat[1]);
			my_addr = hal_flash_byte_read(NV_ADDR_ADDR);
			agent_set_my_addr();

		case CMD_GET_ADDR:
			out_dat[1] = my_addr;
			*out_dat_len = 2;
			break;

		default:
			*out_dat_len = 1;
			break;
	}

	return RES_REP_SRC;
}
