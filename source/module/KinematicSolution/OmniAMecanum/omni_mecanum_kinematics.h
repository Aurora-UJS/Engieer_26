#ifndef __CHASSIS_CACULATION_H__
#define __CHASSIS_CACULATION_H__

#include "main.h"
#include "motion_state.h"

#define CHASSIS_WZ_SET_SCALE (0.0f)

#define MOTOR_DISTANCE_TO_CENTER (1.2f) 

void omni_mecanum_kinematics(basic_vector_t *motion_vector,float *motor_speed_out);


#endif 
