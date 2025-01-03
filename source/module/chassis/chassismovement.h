#ifndef __CHASSISMOVEMENT_H__
#define __CHASSISMOVEMENT_H__

#include "motor_DJI.h"

extern DJI_motor_t *DJMotor_chassis;

void DJI_motor_init();
void chassis_pid_init(void);
void chassis_move(void);



#endif 