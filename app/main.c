#include <stdio.h>
#include <string.h>
#include "nrf24le1.h"
#include "hal_uart.h"
#include "hal_clk.h"
#include "gzll_mcu.h"
#include "gzll.h"

#define UART_MSG_SIZE       16  // GZLL_MAX_PAYLOAD_LENGTH
#define UART_KEY_START      0x55
#define UART_KEY_ESCAPE     0xAA
#define UART_STA_SIZE       0x01
#define UART_STA_BODY       0x02
#define UART_STA_DONE       0x04
#define UART_FIFO_SIZE      8

#define NRF_MSG_SIZE        16  // GZLL_MAX_PAYLOAD_LENGTH
#define NRF_FIFO_SIZE       8

#define UART_DIR_MASK       0xE7
#define PORT_0              P0
#define PORT_0_DIR          P0DIR
#define PORT_0_MASK         0xE7
#define PORT_1              P1
#define PORT_1_DIR          P1DIR
#define PORT_1_MASK         0x7F

#define CMD_ACK_FLAG        0x80
#define CMD_NOP             0x00
#define CMD_PORT_GET	    0x01
#define CMD_PORT_SET	    0x02
#define CMD_DIR_GET         0x03
#define CMD_DIR_SET         0x04

// COMMON
uint8_t agent_msg[NRF_MSG_SIZE], agent_msg_size = 0, agent_msg_pipe = 0;
bool result;

// UART RX
uint8_t uart_char_cnt = 0, uart_char_pre = 0, uart_char_curr = 0;
uint8_t uart_msg[UART_MSG_SIZE], uart_msg_size = 0;
uint8_t uart_msg_state = 0, uart_msg_len = 0, *uart_msg_ptr;
uint8_t uart_msg_rx_fifo[UART_FIFO_SIZE][UART_MSG_SIZE], uart_size_rx_fifo[UART_FIFO_SIZE];
uint8_t uart_rx_fifo_head = 0, uart_rx_fifo_tail = 0, uart_rx_fifo_len = 0;

// UART TX
uint8_t uart_msg_tx_fifo[UART_FIFO_SIZE][UART_MSG_SIZE], uart_size_tx_fifo[UART_FIFO_SIZE];
uint8_t uart_tx_fifo_head = 0, uart_tx_fifo_tail = 0, uart_tx_fifo_len = 0;

// NRF RX
//uint8_t nrf_msg[NRF_MSG_SIZE], nrf_msg_size = 0, nrf_msg_pipe = 0;
uint8_t nrf_msg_rx_fifo[NRF_FIFO_SIZE][NRF_MSG_SIZE], nrf_size_rx_fifo[NRF_FIFO_SIZE], 
        nrf_pipe_rx_fifo[NRF_FIFO_SIZE];
uint8_t nrf_rx_fifo_head = 0, nrf_rx_fifo_tail = 0, nrf_rx_fifo_len = 0;

// NRF TX
uint8_t nrf_msg_tx_fifo[NRF_FIFO_SIZE][NRF_MSG_SIZE], nrf_size_tx_fifo[NRF_FIFO_SIZE], 
        nrf_pipe_tx_fifo[NRF_FIFO_SIZE];
uint8_t nrf_tx_fifo_head = 0, nrf_tx_fifo_tail = 0, nrf_tx_fifo_len = 0;

void putstring(char *s);
void putdata(uint8_t *d, uint8_t l);
void get_msg_from_uart();
void forward_msg_from_uart_to_nrf();
void get_msg_from_nrf();
void forward_msg_to_uart(uint8_t *msg, uint8_t size);
void agent_task(uint8_t * msg, uint8_t size, uint8_t pipe);
void agent_process_msg();
void send_nrf_msg();
void send_uart_msg();
        
void putstring(char *s)
{
    while(*s != 0)
        hal_uart_putchar(*s++);
}

void putdata(uint8_t *d, uint8_t l)
{
    while(l--)
        hal_uart_putchar(*d++);
}

void get_msg_from_uart()
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
                if(uart_rx_fifo_len < UART_FIFO_SIZE)
                {
                    memcpy(uart_msg_rx_fifo[uart_rx_fifo_tail], uart_msg, uart_msg_size); 
                    uart_size_rx_fifo[uart_rx_fifo_tail] = uart_msg_size;
                    
                    uart_rx_fifo_tail = (uart_rx_fifo_tail + 1) % UART_FIFO_SIZE;
                    uart_rx_fifo_len++;
                }
            }                
            continue;
        }
    }
}

void forward_msg_from_uart_to_nrf()
{
    if(uart_rx_fifo_len > 0)
    {
        result = true;
        if(gzll_get_state() == GZLL_HOST_ACTIVE)
            gzll_goto_idle();
        do
        {
            result = gzll_tx_data(uart_msg_rx_fifo[uart_rx_fifo_head],
                uart_size_rx_fifo[uart_rx_fifo_head], 0);
            if(result)
            {
                uart_rx_fifo_head = (uart_rx_fifo_head + 1) % UART_FIFO_SIZE;
                uart_rx_fifo_len--;
            }
        }while(result && uart_rx_fifo_len > 0);
        while(gzll_get_state() != GZLL_IDLE)
        {}
        gzll_rx_start();
    }
}

void get_msg_from_nrf()
{
    if(gzll_get_state() != GZLL_DEVICE_ACTIVE)
    {
        while(nrf_rx_fifo_len < NRF_FIFO_SIZE && gzll_rx_data_ready(0xFF)) // && *len > 0)
        {
            result = true;
            memset(agent_msg, 0, NRF_MSG_SIZE);
            agent_msg_size = agent_msg_pipe = 0;
            result = gzll_rx_fifo_read(agent_msg, &agent_msg_size, &agent_msg_pipe);
            if(result == false || agent_msg_size <= 0 || agent_msg_size > NRF_MSG_SIZE)
                continue;
            
            memcpy(nrf_msg_rx_fifo[nrf_rx_fifo_tail], agent_msg, agent_msg_size); 
            nrf_size_rx_fifo[nrf_rx_fifo_tail] = agent_msg_size;
            nrf_pipe_rx_fifo[nrf_rx_fifo_tail] = agent_msg_pipe;
            
            nrf_rx_fifo_tail = (nrf_rx_fifo_tail + 1) % NRF_FIFO_SIZE;
            nrf_rx_fifo_len++;
        }
    }
}

void forward_msg_to_uart(uint8_t *msg, uint8_t len)
{
    uint8_t *fifo_ptr, *msg_ptr, cnt;
    if(uart_tx_fifo_len < UART_FIFO_SIZE)
    {
        fifo_ptr = uart_msg_tx_fifo[uart_tx_fifo_tail];
        msg_ptr = msg;
        cnt = len;
        *fifo_ptr++ = UART_KEY_START;
        if(len == UART_KEY_START || len == UART_KEY_ESCAPE)
        {
            *fifo_ptr++ = UART_KEY_ESCAPE;
            len++;
        }
        *fifo_ptr++ = cnt;
        len += 2;
        while(cnt--)
        {
            if(*msg_ptr == UART_KEY_START || *msg_ptr == UART_KEY_ESCAPE)
            {
                *fifo_ptr++ = UART_KEY_ESCAPE;
                len++;
            }
            *fifo_ptr++ = *msg_ptr++;
        }
        uart_size_tx_fifo[uart_tx_fifo_tail] = len;
        
        uart_tx_fifo_tail = (uart_tx_fifo_tail + 1) % UART_FIFO_SIZE;
        uart_tx_fifo_len++;
    }
}

void agent_task(uint8_t *msg, uint8_t len, uint8_t pip)
{ 
    result = true;
    memcpy(agent_msg, msg, len);
    memset(agent_msg + len, 0, NRF_MSG_SIZE - len);
    switch(agent_msg[0])
    {
        case CMD_NOP:
            break;
        case CMD_PORT_SET:
            PORT_0 |= agent_msg[1] &  agent_msg[2] &  PORT_0_MASK;
            PORT_0 &= agent_msg[1] | ~agent_msg[2] | ~PORT_0_MASK;
            PORT_1 |= agent_msg[3] &  agent_msg[4] &  PORT_1_MASK;
            PORT_1 &= agent_msg[3] | ~agent_msg[4] | ~PORT_1_MASK;
        case CMD_PORT_GET:
            agent_msg[2] = PORT_0_MASK & agent_msg[2];
            agent_msg[1] = PORT_0      & agent_msg[2];
            agent_msg[4] = PORT_1_MASK & agent_msg[4];
            agent_msg[3] = PORT_1      & agent_msg[4];
            break;
        case CMD_DIR_SET:
            PORT_0_DIR |= agent_msg[1] &  agent_msg[2] &  PORT_0_MASK;
            PORT_0_DIR &= agent_msg[1] | ~agent_msg[2] | ~PORT_0_MASK; 
            PORT_1_DIR |= agent_msg[3] &  agent_msg[4] &  PORT_1_MASK;
            PORT_1_DIR &= agent_msg[3] | ~agent_msg[4] | ~PORT_1_MASK; 
        case CMD_DIR_GET:
            agent_msg[2] = PORT_0_MASK & agent_msg[2];
            agent_msg[1] = PORT_0_DIR  & agent_msg[2];
            agent_msg[4] = PORT_1_MASK & agent_msg[4];
            agent_msg[3] = PORT_1_DIR  & agent_msg[4];
            break;
        default:
            result = false;
            break;
    }
    if(result && nrf_tx_fifo_len < NRF_FIFO_SIZE)
    {
        agent_msg[0] |= CMD_ACK_FLAG;
        memcpy(nrf_msg_tx_fifo[nrf_tx_fifo_tail], agent_msg, len); 
        nrf_size_tx_fifo[nrf_tx_fifo_tail] = len;
        nrf_pipe_tx_fifo[nrf_tx_fifo_tail] = pip;
        
        nrf_tx_fifo_tail = (nrf_tx_fifo_tail + 1) % NRF_FIFO_SIZE;
        nrf_tx_fifo_len++;
    }
}

void agent_process_msg()
{
    uint8_t *msg, len, pip;
    while(nrf_rx_fifo_len > 0)
    {
        msg = nrf_msg_rx_fifo[nrf_rx_fifo_head];
        len = nrf_size_rx_fifo[nrf_rx_fifo_head];
        pip = nrf_pipe_rx_fifo[nrf_rx_fifo_head];
        
        forward_msg_to_uart(msg, len);
        agent_task(msg, len, pip);
        
        nrf_rx_fifo_head = (nrf_rx_fifo_head + 1) % NRF_FIFO_SIZE;
        nrf_rx_fifo_len--;
    }
}

void send_nrf_msg()
{
    if(nrf_tx_fifo_len > 0)
    {
        result = true;
        if(gzll_get_state() == GZLL_HOST_ACTIVE)
            gzll_goto_idle();
        do
        {
            result = gzll_tx_data(nrf_msg_tx_fifo[nrf_tx_fifo_head], 
                    nrf_size_tx_fifo[nrf_tx_fifo_head], nrf_pipe_tx_fifo[nrf_tx_fifo_head]);  
            if(result)
            {
                nrf_tx_fifo_head = (nrf_tx_fifo_head + 1) % NRF_FIFO_SIZE;
                nrf_tx_fifo_len--;
            }
        }while(result && nrf_tx_fifo_len > 0);
        while(gzll_get_state() != GZLL_IDLE)
        {}
        gzll_rx_start();
    }
}

void send_uart_msg()
{
    while(uart_tx_fifo_len > 0 && hal_uart_tx_complete())
    {
        putdata(uart_msg_tx_fifo[uart_tx_fifo_head], uart_size_tx_fifo[uart_tx_fifo_head]);
                
        uart_tx_fifo_head = (uart_tx_fifo_head + 1) % UART_FIFO_SIZE;
        uart_tx_fifo_len--;
    }
}

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
    
    // Enable global interrupts
    EA = 1;

    // Enter host mode (start monitoring for data)
    gzll_rx_start();         

    for(;;)
    {
        get_msg_from_uart();
        forward_msg_from_uart_to_nrf();
        get_msg_from_nrf();
        agent_process_msg();
        send_nrf_msg();
        send_uart_msg();
    }
}