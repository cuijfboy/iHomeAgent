#ifndef NRF2_H__
#define NRF2_H__

#include <stdio.h>
#include <stdint.h>
#include <string.h>

void nrf_rx_msg(struct nrf_msg_fifo *fifo);
void nrf_tx_msg(struct nrf_msg_fifo *fifo);

#endif // NRF2_H__