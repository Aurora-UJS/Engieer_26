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

#endif
