

#ifndef MYPROJECT_H
#define MYPROJECT_H

/* Includes ------------------------------------------------------------------*/

#include "usart.h"
#include "delay.h"
#include "timer.h"
#include "adc.h"

#include "foc_utils.h"
#include "FOCMotor.h"
#include "BLDCmotor.h" 
#include "lowpass_filter.h" 
#include "pid.h"
#include "InlineCurrentSense.h"
#include "CurrentSense.h"
#include "HallSensor.h" 

//SCL1(PB6)、SDA1(PB7)和PA12接hall信号，三个信号之间随便接，不用区分
//SCL2(PB10)、SDA2(PB11)和SWD(PA13)接hall信号，三个信号之间随便接，不用区分
/******************************************************************************/
#define M1_Enable    GPIO_SetBits(GPIOB,GPIO_Pin_9)          //高电平使能
#define M1_Disable   GPIO_ResetBits(GPIOB,GPIO_Pin_9)        //低电平失能
#define M2_Enable    GPIO_SetBits(GPIOB,GPIO_Pin_3)          //高电平使能
#define M2_Disable   GPIO_ResetBits(GPIOB,GPIO_Pin_3)        //低电平失能
/******************************************************************************/

#endif

