
#ifndef STM32_USART1_H
#define STM32_USART1_H
/******************************************************************************/
#include "stdio.h"
#include "stm32f10x.h"

/******************************************************************************/
#define USART_REC_LEN 256
#define BUFFER_SIZE 69
/******************************************************************************/
extern unsigned char USART_RX_BUF[USART_REC_LEN];
extern unsigned short USART_RX_STA;
extern uint8_t USART_RX_BUF2[USART_REC_LEN];
extern unsigned short USART_RX_STA2;
extern volatile uint8_t rx_complete_flag;
extern int max_indexs;
/******************************************************************************/
void uart_init(unsigned long bound);
void uart3_init(unsigned long bound2);
void commander_uart(void);
void actual_pressure(uint8_t data[256]);
float find_max(int *max_index);
/******************************************************************************/


#endif
