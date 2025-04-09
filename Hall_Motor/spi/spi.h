#ifndef __SPI_H
#define __SPI_H

#include "stm32f10x.h"
#include "stdio.h"
#define SPI_REC_LEN 256
extern unsigned short SPI_RX_STA;   
extern unsigned char SPI_RX_BUF[SPI_REC_LEN]; 

void SPI2_Slave_Init(void);                

#define SPI_CS_LOW()  GPIO_ResetBits(GPIOA, GPIO_Pin_4)
#define SPI_CS_HIGH() GPIO_SetBits(GPIOA, GPIO_Pin_4)

#endif 
