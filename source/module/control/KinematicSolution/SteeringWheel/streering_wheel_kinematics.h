#ifndef STREERING_WHEEL_KINEMATICS_H
#define STREERING_WHEEL_KINEMATICS_H

#include "main.h"
#include "motion_state.h"

basic_angleV_t steering_wheel_kinematics(basic_vector_t *motion_vector, basic_angleV_t *motor_angleV_out);

#endif // !STREERING_WHEEL_KINEMATICS_H