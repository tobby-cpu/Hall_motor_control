
#include "stm32f10x.h"
#include <string.h>
#include <stdlib.h>
#include "MyProject.h"
#include "user.h"
#include "finger.h"
/******************************************************************************/
//Dalian University of Technology Yunxin Zhao (Add an angle control loop and three-motor coordinated control to the existing closed-loop control system.)
/******************************************************************************/
#define LED_blink    GPIOC->ODR^=(1<<13)
#define PI 3.14159265358979323846
/*****************************************************************************************************************************************************/
MOTORController M1,M2;
long timecntr_pre=0;
long time_cntr=0;

void commander_run(void);
void commander_spi(void);
int kk;
/******************************************************************************/
//us计时，每71.5分钟溢出循环一次
uint32_t timecount(void)
{
	uint32_t  diff,now_us;
	
	now_us = _micros();    //0xFFFFFFFF=4294967295 us=71.5分钟
	if(now_us>=timecntr_pre)diff = now_us - timecntr_pre;   //us
	else
		diff = 0xFFFFFFFF - timecntr_pre + now_us;
	timecntr_pre = now_us;
	
	return diff;
}
/******************************************************************************/
void GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO, ENABLE);//使能GPIOA,GPIOB,GPIOC,AFIO;
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;         //PC13是LED
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   //推挽输出	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;  //速度
	GPIO_Init(GPIOC, &GPIO_InitStructure);             //对选中管脚初始化
	GPIO_ResetBits(GPIOC,GPIO_Pin_13);                 //上电点亮LED
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_9;          //使能,PB9是motor1,PB3是motor2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOB,GPIO_Pin_3|GPIO_Pin_9);                  //低电平解除,Motor_init()中使能
}
/******************************************************************************/
int main(void)
{
	GPIO_Config();
	uart_init(115200);
	uart3_init(460800);
	strcpy(M1.str, "M1");
	strcpy(M2.str, "M2");
	TIM2_PWM_Init();    //M1_PWM
	TIM3_PWM_Init();    //M2_PWM
	systick_CountInit();      
	printf("Double HALL F1\r\n");
	
	delay_ms(1000);               //Wait for the system to stabilize
	//InlineCurrentSense(&M1,0.001,50,ADC_Channel_3,ADC_Channel_4,NOT_SET);    //SimpleMotor//采样电阻阻值，运放倍数，A相，B相，C相
	//InlineCurrentSense(&M2,0.001,50,ADC_Channel_5,ADC_Channel_9,NOT_SET);
	//InlineCurrentSense_Init(&M1); //ADC初始化和偏置电压校准
	//InlineCurrentSense_Init(&M2);
	configureADC();
	LPF_init(&M1);                //LPF参数初始化，可进入函数初始化参数
	LPF_init(&M2);
	PID_init(&M1);                //PID参数初始化，可进入函数初始化参数
	PID_init(&M2);
	SPI2_Slave_Init();
	voltage_power_supply=24;      //V,两个电机的共同参数
	
	M1.pole_pairs=2;              //电机极对数，磁铁的对数
	M1.voltage_sensor_align=2.5;  
	M1.voltage_limit=10;          
	M1.velocity_limit=20;         
	M1.current_limit=50;          
	M1.torque_controller=Type_voltage;  //Type_dc_current;//  Type_foc_current;  //Type_voltage;
	M1.controller=Type_angle;  //Type_torque;  //Type_velocity;  //Type_angle; 
	M1.PID_d.P=0.6;               
	M1.PID_d.I=0.5;                 
	M1.PID_q.P=0.6;
	M1.PID_q.I=0.5;
	M1.PID_vel.P=0.1;             
	M1.PID_vel.I=0.6;
	M1.P_ang.P=20;                
	M1.PID_vel.output_ramp=50;    
	M1.LPF_vel.Tf=0.05;           
	M1.target=0;
	
	M2.pole_pairs=2;              //电机极对数
	M2.voltage_sensor_align=1.5;    //V alignSensor() use it，大功率电机设置的值小一点比如0.5-1，小电机设置的大一点比如2-3
	M2.voltage_limit=13;           //V，最大值需小于12/1.732=6.9
	M2.velocity_limit=20;         
	M2.current_limit=50;          
	M2.torque_controller=Type_voltage;  //Type_dc_current;//  Type_foc_current;  //Type_voltage;
	//M2.controller=Type_torque;  //Type_torque;  //Type_velocity;  //Type_angle; 
	M2.PID_d.P=0.6;             
	M2.PID_d.I=0.5;                
	M2.PID_q.P=0.6;
	M2.PID_q.I=0.5;
	M2.PID_vel.P=0.1;             
	M2.PID_vel.I=0.6;
	M2.P_ang.P = 20;							
	M2.PID_vel.output_ramp=50;    
	M2.LPF_vel.Tf=0.05;         
	M2.target=0;
	
	Hall_init();                  
	Motor_init(&M1);
	Motor_init(&M2);
	Motor_initFOC(&M1,4.188,CW);
	Motor_initFOC(&M2, 2.094, CW); //(&M2, 2.094, CW); 
  printf("Motor ready.\r\n");
	//***************************************************//


	//**********************************************************//
	while(1)
	{
		
		time_cntr +=timecount();
		if(time_cntr>=5000)  //us
		{
			time_cntr=0;
			LED_blink;
		
		}
		move(&M1, M1.target);
		loopFOC(&M1,ADC_Channel_3);
		move(&M2, M2.target);
		loopFOC(&M2,ADC_Channel_4);
		commander_run();
		printf("index: %d",max_indexs);
		//main_calculate(*max_index, *max_index, *max_index, 30, M1.shaft_angle_new, M2.shaft_angle_new, 60);//初始化一个新的ADC//将max_value变为外部变量//调试的时候不要delay
		//delay_ms(50);
  }

}

void commander_run(void)
{
	if((USART_RX_STA&0x8000)!=0)
	{
		switch(USART_RX_BUF[0])
			{ 
			case 'Z'://切换模式
				switch(USART_RX_BUF[1])
				{
					case'A':
						M1.controller = Type_torque;
						M1.target = 0;
						printf("Type_torque!\r\n");
						break;
					case 'B':
						M1.controller = Type_angle_openloop;
						M1.target = 0;
						printf("Type_angle_openloop!\r\n");
						break;
				}
			  break;
			case 'H':
				printf("Hello World!\r\n");
				break;
			case 'A':   //A6.28
				M1.target=atof((const char *)(USART_RX_BUF+1));
				printf("A=%.4f\r\n", M1.target);
				break;
			case 'B':   //B6.28
				M2.target=atof((const char *)(USART_RX_BUF+1));
				printf("B=%.4f\r\n", M2.target);
				break;
			case 'T':   //T6.28
				M1.target=atof((const char *)(USART_RX_BUF+1));
				M2.target=M1.target;
				printf("T=%.4f\r\n", M1.target);
				break;
			
			case 'M':  
				switch(USART_RX_BUF[1])
				{
					case 'P':   //设置电流环的P参数,MP1
						M1.PID_d.P=atof((const char *)(USART_RX_BUF+2));
					  M1.PID_q.P=M1.PID_d.P;
					  printf("M1.current.P=%.4f\r\n", M1.PID_d.P);
					  break;
					case 'I':   //设置电流环的I参数,MI0.02
						M1.PID_d.I=atof((const char *)(USART_RX_BUF+2));
					  M1.PID_q.I=M1.PID_d.I;
					  printf("M1.current.I=%.4f\r\n", M1.PID_d.I);
					  break;
					case 'V':   //MV  读实时速度
					  printf("M1.vel=%.2f\r\n", M1.shaft_velocity);
						break;
					case 'A':   //MA  读绝对角度
					  printf("M1.ang=%.2f\r\n", M1.shaft_angle);
						break;
					case 'G':  //MG读取角度传感器的值
						printf("M1.new_ang=%.2f\r\n", M1.shaft_angle_new);
						break;
				}
				break;
			case 'N':  //M2
				switch(USART_RX_BUF[1])
				{
					case 'P':
						M2.PID_d.P=atof((const char *)(USART_RX_BUF+2));
					  M2.PID_q.P=M2.PID_d.P;
					  printf("M2.current.P=%.4f\r\n", M2.PID_d.P);
					  break;
					case 'I':
						M2.PID_d.I=atof((const char *)(USART_RX_BUF+2));
					  M2.PID_q.I=M2.PID_d.I;
					  printf("M2.current.I=%.4f\r\n", M2.PID_d.I);
					  break;
					case 'V':   //MV  读实时速度
					  printf("M2.vel=%.2f\r\n", M2.shaft_velocity);
						break;
					case 'A':   //MA  读绝对角度
					  printf("M2.ang=%.2f\r\n", M2.shaft_angle);
						break;
				}
				break;
		}
		USART_RX_STA=0;
	}
}

void commander_spi(void) {
    if((SPI_RX_STA & 0x8000) != 0) { // 接收完成标志位检查[1]
        switch(SPI_RX_BUF[0])
			{ 
				case 'Z'://切换模式
					switch(SPI_RX_BUF[1])
					{
						
						case'A':
							M1.controller = Type_torque;
							M1.target = 0;
							printf("Type_torque!\r\n");
						break;
						
						case 'B':
							M1.controller = Type_angle_openloop;
							M1.target = 0;
							printf("Type_angle_openloop!\r\n");
						break;
					
					}
			  break;
			case 'H':
				printf("Hello World!\r\n");
				break;
			case 'A':   //A6.28
				M1.target=atof((const char *)(SPI_RX_BUF+1));
				printf("A=%.4f\r\n", M1.target);
				break;
			case 'B':   //B6.28
				M2.target=atof((const char *)(SPI_RX_BUF+1));
				printf("B=%.4f\r\n", M2.target);
				break;
			case 'T':   //T6.28
				M1.target=atof((const char *)(SPI_RX_BUF+1));
				M2.target=M1.target;
				printf("T=%.4f\r\n", M1.target);
				break;
			
			case 'M': 
				switch(USART_RX_BUF[1])
				{
					case 'P':   //设置电流环的P参数,MP1
						M1.PID_d.P=atof((const char *)(SPI_RX_BUF+2));
					  M1.PID_q.P=M1.PID_d.P;
					  printf("M1.current.P=%.4f\r\n", M1.PID_d.P);
					  break;
					case 'I':   //设置电流环的I参数,MI0.02
						M1.PID_d.I=atof((const char *)(SPI_RX_BUF+2));
					  M1.PID_q.I=M1.PID_d.I;
					  printf("M1.current.I=%.4f\r\n", M1.PID_d.I);
					  break;
					case 'V':   //MV  读实时速度
					  printf("M1.vel=%.2f\r\n", M1.shaft_velocity);
						break;
					case 'A':   //MA  读绝对角度
					  printf("M1.ang=%.2f\r\n", M1.shaft_angle);
						break;
					case 'G':  //MG读取角度传感器的值
						printf("M1.new_ang=%.2f\r\n", M1.shaft_angle_new);
						break;
				}
				break;
			case 'N':  //M2
				switch(SPI_RX_BUF[1])
				{
					case 'P':
						M2.PID_d.P=atof((const char *)(SPI_RX_BUF+2));
					  M2.PID_q.P=M2.PID_d.P;
					  printf("M2.current.P=%.4f\r\n", M2.PID_d.P);
					  break;
					case 'I':
						M2.PID_d.I=atof((const char *)(SPI_RX_BUF+2));
					  M2.PID_q.I=M2.PID_d.I;
					  printf("M2.current.I=%.4f\r\n", M2.PID_d.I);
					  break;
					case 'V':   //MV  读实时速度
					  printf("M2.vel=%.2f\r\n", M2.shaft_velocity);
						break;
					case 'A':   //MA  读绝对角度
					  printf("M2.ang=%.2f\r\n", M2.shaft_angle);
						break;
				}
				break;
		}
		SPI_RX_STA=0;
       
    }
}


