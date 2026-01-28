#ifndef ARM_CONTROL_DRV_H
#define ARM_CONTROL_DRV_H

#include "motor_DM.h"

static inline float Motor_Get_Radian(const DM_motor_t *motor) {
  return motor->motor_msg.motor_angle;
}

#endif
