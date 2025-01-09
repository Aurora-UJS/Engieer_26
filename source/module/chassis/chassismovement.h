#ifndef __CHASSISMOVEMENT_H__
#define __CHASSISMOVEMENT_H__

#include "motor_DJI.h"

#define MOTOR_COUNT 4


extern DJI_motor_t *DJMotor_chassis;
extern pid_type_def pid_chassis[4];

void DJI_motor_init();
void chassis_pid_init(void);
void chassis_move(void);



#endif 