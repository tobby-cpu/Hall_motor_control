
#include "usart.h"
#include "user.h"

//本段代码参考了正点原子，微调
/******************************************************************************/
//加入以下代码，支持printf函数，而不需要选择MicroLIB
#if 1
#pragma import(__use_no_semihosting)
//标准库需要的支持函数
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;
//定义_sys_exit避免使用半主机模式
_sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);
    USART1->DR = (u8) ch;      
	return ch;
}
#endif 

 /* 
int fputc(int ch, FILE *f)
{
	USART_SendData(USART1, (uint8_t) ch);

	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) {}	
   
    return ch;
}
int GetKey (void)  { 

    while (!(USART1->SR & USART_FLAG_RXNE));

    return ((int)(USART1->DR & 0x1FF));
}
*/
	
/******************************************************************************/
unsigned char USART_RX_BUF[USART_REC_LEN];//接收缓冲，usart.h中定义长度
uint8_t USART_RX_BUF2[USART_REC_LEN];
volatile uint16_t rx_index = 0;            // 接收索引
volatile uint8_t rx_complete_flag = 0;     // 接收完成标志
//接收状态
//bit15  接收完成标志
//bit14  接收到0x0D
//bit13~0  接收的字节数
unsigned short USART_RX_STA=0; //接收状态标志
unsigned short USART_RX_STA2=0;
int jj;
int ii;
float voltage0[100];
int max_idx;
int max_indexs;
/******************************************************************************/
void uart_init(unsigned long bound)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);
  
	//USART1_TX   GPIOA.9 = TX
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA9
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
   
  //USART1_RX	  GPIOA.10 = RX
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;    //抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;         //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			       //IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);
  
	USART_InitStructure.USART_BaudRate = bound;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART1, &USART_InitStructure);       //串口1初始化
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);  //使能接收中断
  USART_Cmd(USART1, ENABLE);                      //使能串口1

}
/******************************************************************************/
void USART1_IRQHandler(void)   //串口1中断程序
{
	unsigned char Res;
	
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //
	{
		Res =USART_ReceiveData(USART1);	//读取接收到的字节
		
		if((USART_RX_STA&0x8000)==0)    //接收未完成
		{
			if(USART_RX_STA&0x4000)       //接收到了0x0d
			{
				if(Res!=0x0a)USART_RX_STA=0;   //接收错误，重新开始
				else 
				{
					USART_RX_STA|=0x8000;	       //接收完成
					USART_RX_BUF[USART_RX_STA&0X3FFF]='\0';   //最后一个字节放'0’，方便判断
				}
			}
			else //还没收到0x0D
			{	
				if(Res==0x0d)USART_RX_STA|=0x4000;
				else
				{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=Res;
					USART_RX_STA++;
					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;  //接收错误，重新开始  
				}		 
			}
		}
  }
}

void uart3_init(unsigned long bound1)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // 使能时钟（UART3在APB1，GPIOB在APB2）
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    // UART3_TX -> PB10 复用推挽输出
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // UART3_RX -> PB11 浮空输入
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // 配置NVIC中断
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  // 抢占优先级3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;          // 子优先级3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // USART3参数配置
    USART_InitStructure.USART_BaudRate = bound1;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART3, &USART_InitStructure);

    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);  // 使能接收中断
    USART_Cmd(USART3, ENABLE);                      // 使能串口3
}

void USART3_IRQHandler(void) {
    if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) {
        uint8_t Res = USART_ReceiveData(USART3);
			
        if (rx_index < 71) {
            USART_RX_BUF2[rx_index++] = Res;
					
        } else {
            rx_index = 0;
        }
        
        if (Res == 0x0D) {
          USART_RX_BUF2[rx_index] = '\0';  
					rx_complete_flag = 1;          // 标记接收完成
          rx_index = 0; 					// 重置索引
        commander_uart();
				}
				 
    }
}

void commander_uart()//需要调试
{
	if (rx_complete_flag == 1)
	{
		switch(USART_RX_BUF2[1])
			{ 
				
					case 0X01: 
					 	for(jj=2;jj<=65;jj++)
						{
							M1.force1[jj] = USART_RX_BUF2[jj];	
							//printf("%02X ", M1.force1[jj]);
						}
						actual_pressure(M1.force1);
						find_max(&max_idx);
						break;
					case 0X02: 
						for(jj=2;jj<=65;jj++)
						{
							M1.force2[jj] = USART_RX_BUF2[jj];
							printf("%02X ", M1.force2[jj]);
						}
						break;
					case 0X03:  
						for(jj=0;jj<=80;jj++)
						{
							M1.force3[jj] = USART_RX_BUF2[jj+2];
							//printf("M1.force1=%.4f\r\n", M1.force3[0]);
						}
						break;
					}

	}
			rx_complete_flag=0;
}

// 返回最大值，并通过指针参数返回最大值所在的索引
float find_max(int *max_index) {
    float max_val = voltage0[0];
    *max_index = 0; 
    for (jj = 0; jj < 31; jj++) {
        if (voltage0[jj] > max_val) {
            max_val = voltage0[jj];
            *max_index = jj;
					max_indexs = jj;
        }
    }
    printf("max_value: %.4f, index: %d", max_val, *max_index);
    return max_val;
}

void actual_pressure(uint8_t data[128])
{
    int16_t raw_value[32];
    const float scale_factor = 10.0f / 32767.0f;

    for(ii = 0; ii < 32; ii++) {
        int byte_index = 2 + ii * 2;
        raw_value[ii] = (data[byte_index-1] << 8) | data[byte_index];
        voltage0[ii] = raw_value[ii] * scale_factor;
				//printf("%.4f", voltage0[ii]);
    }
}

/***************************************************************************/


