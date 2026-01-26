#ifndef ARM_STATE_MACHINE_H
#define  ARM_STATE_MACHINE_H

#include "joint_control_drv.h"
#include "ee_control_drv.h"

typedef enum {
  Arm_IDLE_Mode = 0,
  Arm_Custom_Controller_Follow_Mode,
  Arm_Frozen_Mode,
  Arm_Set_Radian,
  Arm_TRANSITION_Mode
} arm_control_mode_t;

void Gripper_Control_Mode_Mangner(endEffector_t *EndEffector);

void Joint_Control_Mode_Mangner(Joint_t *Joint);
#endif
