
#ifndef STM32_HALLSENSOR_H
#define STM32_HALLSENSOR_H

#include "user.h"

/******************************************************************************/
void Hall_init(void);
float getAngle(MOTORController *M);
float getVelocity(MOTORController *M);
float getAngle_new(void);
/******************************************************************************/


#endif
