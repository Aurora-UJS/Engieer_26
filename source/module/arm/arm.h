#ifndef __ARM_H__
#define __ARM_H__
#include "motor_DM.h"

extern DM_motor_t *DM_motor_8009_Down, *DM_motor_8009_Up, *DM_motor_6220;
extern DJI_motor_t *DJ_motor_2006_Left, DJ_motor_2006_Right;
void DM_motor_init();
void motor_Reset();
void motor_8009_down_movement();
void motor_8009_up_movement();
void motor_spin_movement();
void motor_Zero();



#endif 