#include <stdio.h>
#include <string.h>
#include "nrf24le1.h"
#include "hal_clk.h"
#include "hal_delay.h"
#include "hal_uart.h"
#include "gzll_mcu.h"
#include "gzll.h"
#include "nrf2.h"
#include "uart.h"
#include "agent.h"
#include "fifo.h"

static struct uart_msg_fifo 	uart_msg_rx_fifo, uart_msg_tx_fifo; 
static struct nrf_msg_fifo 	nrf_msg_rx_fifo, 	nrf_msg_tx_fifo;

void main(void)
{
	mcu_init();
	
	PORT_0 = 0xFF;
	PORT_1 = 0xFF;
	PORT_0_DIR = 0x00;
	PORT_1_DIR = 0x00;
	
	// Initializes the UART
	PORT_0_DIR &= 0xF7;
	PORT_0_DIR |= 0x10;
	hal_uart_init(UART_BAUD_38K4);
	
	// Wait for XOSC to start to ensure proper UART baudrate
	while(hal_clk_get_16m_source() != HAL_CLK_XOSC16M)
	{}
		
	gzll_init();
		
	uart_msg_fifo_init(&uart_msg_rx_fifo);
	uart_msg_fifo_init(&uart_msg_tx_fifo);
	nrf_msg_fifo_init(&nrf_msg_rx_fifo);
	nrf_msg_fifo_init(&nrf_msg_tx_fifo);
		
	agent_init(	&uart_msg_rx_fifo, 
							&uart_msg_tx_fifo, 
							&nrf_msg_rx_fifo, 
							&nrf_msg_tx_fifo);
		
	EA = 1;

	gzll_rx_start(); 

	for(;;)
	{
		uart_rx_msg(&uart_msg_rx_fifo);
		nrf_rx_msg(&nrf_msg_rx_fifo);

		agent_process();
	
		nrf_tx_msg(&nrf_msg_tx_fifo);
		uart_tx_msg(&uart_msg_tx_fifo);
	}
}
