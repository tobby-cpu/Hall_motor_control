#include "SPI.h"       
unsigned short SPI_RX_STA;   
unsigned char SPI_RX_BUF[SPI_REC_LEN]; 

void SPI2_Slave_Init(void) 
{
    GPIO_InitTypeDef GPIO_InitStruct;
    SPI_InitTypeDef SPI_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

		

		NVIC_InitStruct.NVIC_IRQChannel = SPI2_IRQn;
		NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
		NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStruct);
	
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_15;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    // MISO (PB14) 
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_14;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    // NSS (PB12) 
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU; 
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStruct.SPI_Mode = SPI_Mode_Slave;
    SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;     
    SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;    
    SPI_InitStruct.SPI_NSS = SPI_NSS_Hard;       
    SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8; 
    SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_Init(SPI2, &SPI_InitStruct);
		SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_RXNE, ENABLE);
		SPI_Cmd(SPI2, ENABLE); 
}
void SPI2_IRQHandler(void)

{
		
		unsigned char Res;
	printf("Type_angle_openloop!\r\n");
    if(SPI_I2S_GetITStatus(SPI2, SPI_I2S_IT_RXNE) != RESET) 
    {
		
        Res = SPI_I2S_ReceiveData(SPI2);  
        SPI_I2S_SendData(SPI2, Res);  
        
        if((SPI_RX_STA & 0x8000) == 0) 
        {
            if(SPI_RX_STA & 0x4000) 
            {
                if(Res != 0x0A) {     
                    SPI_RX_STA = 0;  
                } else {              
                    SPI_RX_STA |= 0x8000;  
                    SPI_RX_BUF[SPI_RX_STA & 0x3FFF] = '\0'; 
                }
            }
            else  
            {
                if(Res == 0x0D) {   
                    SPI_RX_STA |= 0x4000;  
                } else {
                    SPI_RX_BUF[SPI_RX_STA & 0x3FFF] = Res;
                    SPI_RX_STA++;                       
                    if(SPI_RX_STA > (SPI_REC_LEN-1)) {
                        SPI_RX_STA = 0; 
                    }
                }
            }
        }
        SPI_I2S_ClearITPendingBit(SPI2, SPI_I2S_IT_RXNE); 
    }
}
