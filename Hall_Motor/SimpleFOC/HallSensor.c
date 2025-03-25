
#include "MyProject.h"


/************************************************
SCL1(PB6)、SDA1(PB7)和PA12接hall信号，三个信号随便接，不用区分
SCL2(PB10)、SDA2(PB11)和SWD(PA13)接hall信号，三个信号之间随便接，不用区分
=================================================
使用教程：https://blog.csdn.net/loop222/article/details/129819526
         《SimpleFOC移植STM32(八) —— 霍尔电机》
创建日期：20230321
作    者：loop222 @郑州
************************************************/
/***************************************************************************/
// seq 1 > 5 > 4 > 6 > 2 > 3 > 1     000 001 010 011 100 101 110 111
signed char const ELECTRIC_SECTORS[8] = { -1,  0,  4,  5,  2,  1,  3 , -1 };
/***************************************************************************/
/***************************************************************************/
void updateState(HallSensor *H);
/***************************************************************************/
void Hall_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO,ENABLE); //开启时钟
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13; //PA13--SWDIO引脚，使用此引脚做为编码器输入。注意配置后SWD接口失效，只能用串口下载代码
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;  //上拉
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_SetBits(GPIOA, GPIO_Pin_12|GPIO_Pin_13);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_10|GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;  //上拉
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_SetBits(GPIOB, GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_10|GPIO_Pin_11);
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource12);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource6);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource7);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource13);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource10);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource11);
	
	EXTI_InitStructure.EXTI_Line = EXTI_Line6|EXTI_Line7|EXTI_Line12 | EXTI_Line10|EXTI_Line11|EXTI_Line13;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; //双边触发
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn; //使能外部中断
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn; //使能外部中断
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	M1.Hall.cpr=M1.pole_pairs*6;
	M2.Hall.cpr=M2.pole_pairs*6;
	
	M1.Hall.A_active= GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_6);   //初始化时 主动读一次角度
	M1.Hall.B_active= GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7);
	M1.Hall.C_active= GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_12);
	updateState(&M1.Hall);
	M2.Hall.A_active= GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_10);
	M2.Hall.B_active= GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11);
	M2.Hall.C_active= GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_13);
	updateState(&M2.Hall);
}
/***************************************************************************/
void EXTI15_10_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line12) == SET)
	{
		EXTI_ClearITPendingBit(EXTI_Line12); //清除中断标志位
		M1.Hall.C_active= GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_12);
		updateState(&M1.Hall);
	}
	
	if(EXTI_GetITStatus(EXTI_Line10) == SET)
	{
		EXTI_ClearITPendingBit(EXTI_Line10);
		M2.Hall.A_active= GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_10);
		updateState(&M2.Hall);
	}
	
	if(EXTI_GetITStatus(EXTI_Line11) == SET)
	{
		EXTI_ClearITPendingBit(EXTI_Line11);
		M2.Hall.B_active= GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11);
		updateState(&M2.Hall);
	}
	
	if(EXTI_GetITStatus(EXTI_Line13) == SET)
	{
		EXTI_ClearITPendingBit(EXTI_Line13); //PA13--SWDIO引脚
		M2.Hall.C_active= GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_13);
		updateState(&M2.Hall);
	}
}
/***************************************************************************/
void EXTI9_5_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line6) == SET)
	{
		EXTI_ClearITPendingBit(EXTI_Line6);//清除中断标志位
		M1.Hall.A_active= GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_6);
		updateState(&M1.Hall);
	}
	
	if(EXTI_GetITStatus(EXTI_Line7) == SET)
	{
		EXTI_ClearITPendingBit(EXTI_Line7);//清除中断标志位
		M1.Hall.B_active= GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7);
		updateState(&M1.Hall);
	}
}
/***************************************************************************/
void updateState(HallSensor *H)
{
	signed char new_electric_sector;
	long  new_pulse_timestamp = _micros();
	uint8_t new_hall_state = H->C_active + (H->B_active << 1) + (H->A_active << 2);
	
	if(new_hall_state == H->hall_state)return;
  H->hall_state = new_hall_state;
	new_electric_sector = ELECTRIC_SECTORS[H->hall_state];  //根据hall状态判断所在扇区
	
  if (new_electric_sector - H->electric_sector > 3) {
    //underflow
    H->direction = CCW;
    H->electric_rotations += H->direction;
  } else if (new_electric_sector - H->electric_sector < (-3)) {
    //overflow
    H->direction = CW;
    H->electric_rotations += H->direction;
  } else {
    H->direction = (new_electric_sector > H->electric_sector)? CW : CCW;
  }
  H->electric_sector = new_electric_sector;
	
  // glitch avoidance #2 changes in direction can cause velocity spikes.  Possible improvements needed in this area
  if(H->direction == H->old_direction)
	{
    // not oscilating or just changed direction
		H->pulse_diff = new_pulse_timestamp - H->pulse_timestamp;
  }
	else
	{
		H->pulse_diff = 0;
  }
	
  H->pulse_timestamp = new_pulse_timestamp;
  //total_interrupts++;   //没有用到，所以屏蔽
  H->old_direction = H->direction;
  //if (onSectorChange != nullptr) onSectorChange(electric_sector);
}
/***************************************************************************/
/*
float hall_getSensorAngle(void)
{
	return getAngle();
}*/
/***************************************************************************/
/*
float hall_getMechanicalAngle(void)
{
  return ((float)((electric_rotations * 6 + electric_sector) % cpr) / (float)cpr) * _2PI ;
}*/
/***************************************************************************/
float getVelocity(MOTORController *M)
{
  if(M->Hall.pulse_diff == 0 || ((long)(_micros() - M->Hall.pulse_timestamp) > M->Hall.pulse_diff) ) { // last velocity isn't accurate if too old
    return 0;
  }
	else {
    float vel = M->Hall.direction * (_2PI / (float)M->Hall.cpr) / (M->Hall.pulse_diff / 1000000.0f);
    // quick fix https://github.com/simplefoc/Arduino-FOC/issues/192
    if(vel < -1000 || vel > 1000)  vel = 0.0f;   //if velocity is out of range then make it zero
    return vel;
  }
}
/***************************************************************************/
float getAngle(MOTORController *M)
{
  return ((float)(M->Hall.electric_rotations * 6 + M->Hall.electric_sector) / (float)M->Hall.cpr) * _2PI ;
}

float getAngle_new(void)
{
	float Angle = getPhaseAngle();
	return  Angle  ;
}
/***************************************************************************/
