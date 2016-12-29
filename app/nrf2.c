#include <stdio.h>
#include <string.h>
#include "nrf2.h"
#include "gzll.h"
#include "fifo.h"
#include "hal_uart.h"
#include "uart.h"

void nrf_rx_msg(struct nrf_msg_fifo *fifo)
{
	uint8_t msg[MSG_SIZE], len, pip;
	
	if(gzll_get_state() == GZLL_DEVICE_ACTIVE)
		return;
	
	while(!nrf_msg_fifo_full(fifo) && gzll_rx_data_ready(0xFF))
		if(gzll_rx_fifo_read(msg, &len, &pip))
			nrf_msg_fifo_offer(fifo, msg, len, pip);
}

void nrf_tx_msg(struct nrf_msg_fifo *fifo)
{
	uint8_t msg[MSG_SIZE], len, pip;
	
	if(nrf_msg_fifo_empty(fifo))
		return;
	
	if(gzll_get_state() == GZLL_HOST_ACTIVE)
		gzll_goto_idle();
	
	while(nrf_msg_fifo_poll(fifo, msg, &len, &pip))
		gzll_tx_data(msg, len, pip);
	
	gzll_rx_start();		
}