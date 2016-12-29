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
#define UART_PIP						0xFF
#define UART_SEQ 						0xFF

#define MY_ADDR							0x0A

#define PMW 								0x10

#define NV_PAGE_ADDR				32
#define NV_ADDR_ADDR				0xFA00

#define CMD_ACK_FLAG        0x80
#define CMD_UART_FLAG				0x40
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

static uint8_t /*addrs[6],*/ my_addr;
static uint8_t msg_seq;

void agent_set_my_addr(void);
void agent_task(uint8_t msg_type, uint8_t *msg, uint8_t len);
uint8_t agent_atcion( uint8_t src, uint8_t dst, uint8_t pip, uint8_t seq, 
											uint8_t *cmd, uint8_t cmd_len, 
											uint8_t *res_cmd, uint8_t* res_cmd_len);
uint8_t get_msg_seq(void);

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

void agent_task(uint8_t msg_type, uint8_t *msg, uint8_t len)
{
	uint8_t src, dst, pip, seq, *cmd, cmd_len;
	uint8_t res[MSG_SIZE], res_len, *res_cmd, res_cmd_len, res_code;

	src = msg[0];
	dst = msg[1];
	pip = msg[2];
	seq = msg[3];
	cmd = msg + 4;
	cmd_len = len - 4;
	
	res_cmd = res + 4;
	
	res_code = agent_atcion(src, dst, pip, seq, 
													cmd, cmd_len, res_cmd, &res_cmd_len);
	
	if(res_code == RES_NOP)
		return;
	
	if(res_code & RES_FWD_DST)
	{
		if(src == UART_ADDR)
		{	
			msg[0] = my_addr;
			cmd[0] |= CMD_UART_FLAG;
		}
		if(seq == UART_SEQ)
			msg[3] = get_msg_seq();
	
		nrf_msg_fifo_offer(nrf_msg_tx_fifo, msg, len, pip);
	}
	
	if(res_code & RES_FWD_UART)
	{
		uart_msg_fifo_offer(uart_msg_tx_fifo, msg, len);
	}
	
	if(res_code & RES_REP_SRC)
	{
		res_cmd[0] |= CMD_ACK_FLAG;
		res[0] = my_addr;
		res[1] = src;
		res[2] = pip;
		res[3] = seq;
		res_len = res_cmd_len + 4;
		
		if(src == UART_ADDR)
			uart_msg_fifo_offer(uart_msg_tx_fifo, res, res_len);
		else
			nrf_msg_fifo_offer(nrf_msg_tx_fifo, res, res_len, pip);
	}
}

uint8_t agent_atcion( uint8_t src, uint8_t dst, uint8_t pip, uint8_t seq, 
											uint8_t *cmd, uint8_t cmd_len, 
											uint8_t *res_cmd, uint8_t* res_cmd_len)
{
	uint8_t cmd_val;
	
	if(dst != my_addr && (dst != UART_ADDR || src != UART_ADDR))
		return RES_FWD_DST;
	
	if(cmd[0] & CMD_ACK_FLAG)
		if(cmd[0] & CMD_UART_FLAG)
			return RES_FWD_UART;
		else
			return RES_NOP;
	
	cmd_val = cmd[0] & ~CMD_UART_FLAG & ~CMD_ACK_FLAG;
		
	if(cmd_val == CMD_FWD)
		return RES_FWD_UART;
	
	res_cmd[0] = cmd[0];
	
	switch(cmd_val)
	{
		case CMD_FWD:
			return RES_FWD_UART;
		
		case CMD_NOP:
			*res_cmd_len = cmd_len;
			break;
		
		case CMD_SET_PORT:
			PORT_0 |= cmd[1] &  cmd[2] &  PORT_0_MASK;
			PORT_0 &= cmd[1] | ~cmd[2] | ~PORT_0_MASK;
			PORT_1 |= cmd[3] &  cmd[4] &  PORT_1_MASK;
			PORT_1 &= cmd[3] | ~cmd[4] | ~PORT_1_MASK;
		
		case CMD_GET_PORT:
			res_cmd[1] = PORT_0      & cmd[2];
			res_cmd[2] = PORT_0_MASK & cmd[2];
			res_cmd[3] = PORT_1      & cmd[4];
			res_cmd[4] = PORT_1_MASK & cmd[4];
		  *res_cmd_len = 5;
			break;
		
		case CMD_SET_DIR:
			PORT_0_DIR |= cmd[1] &  cmd[2] &  PORT_0_MASK;
			PORT_0_DIR &= cmd[1] | ~cmd[2] | ~PORT_0_MASK; 
			PORT_1_DIR |= cmd[3] &  cmd[4] &  PORT_1_MASK;
			PORT_1_DIR &= cmd[3] | ~cmd[4] | ~PORT_1_MASK; 
		
		case CMD_GET_DIR:
			res_cmd[1] = PORT_0_DIR  & cmd[2];
			res_cmd[2] = PORT_0_MASK & cmd[2];
			res_cmd[3] = PORT_1_DIR  & cmd[4];
			res_cmd[4] = PORT_1_MASK & cmd[4];
			*res_cmd_len = 5;
			break;
		
		case CMD_SET_ADDR:
			PCON &= ~PMW;
			hal_flash_page_erase(NV_PAGE_ADDR);
		  hal_flash_byte_write(NV_ADDR_ADDR, cmd[1]);
		  my_addr = hal_flash_byte_read(NV_ADDR_ADDR);
		  agent_set_my_addr();
		
		case CMD_GET_ADDR:
			res_cmd[1] = my_addr;
			*res_cmd_len = 2;
			break;
		
		default:
			*res_cmd_len = 1;
			break;
	}
	
	return RES_REP_SRC;
}

uint8_t get_msg_seq(void)
{
	msg_seq++;
	if(msg_seq == UART_SEQ)
		msg_seq = 0;
	return msg_seq;
}
