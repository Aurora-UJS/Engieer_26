#ifndef ARM_STATE_MACHINE_TASK_H
#define ARM_STATE_MACHINE_TASK_H

#include "DBusSys.h"
#include "ee_control_drv.h"
#include "cmsis_os2.h"
#include "stm32h7xx_hal_def.h"
#include <stdint.h>

typedef enum {
  SW_DEFAULT,
  SW1_UP,
  SW1_DOWN,
  SW1_MID,
  SW2_UP,
  SW2_DOWN,
  SW2_MID,
} rc_input_id_t;

typedef enum {
  Arm_IDLE_Mode = 0,
  Arm_Custom_Controller_Follow_Mode,
  Arm_Frozen_Mode,
  Arm_Set_Radian,
  Arm_TRANSITION_Mode
} arm_control_mode_t;
#endif
