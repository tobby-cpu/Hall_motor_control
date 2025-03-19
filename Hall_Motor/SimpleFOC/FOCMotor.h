#ifndef FOCMOTOR_H
#define FOCMOTOR_H

#include "foc_utils.h" 


/******************************************************************************/
float shaftAngle(MOTORController *M);
float shaftVelocity(MOTORController *M);
float electricalAngle(MOTORController *M);
float shaftAngle_new(MOTORController *M);
/******************************************************************************/

#endif

