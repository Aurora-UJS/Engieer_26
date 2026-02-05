#ifndef TRAJECTORY_PUBLISHER_TASK_H
#define TRAJECTORY_PUBLISHER_TASK_H

#include "Trajectory_Data.h"
#include "arm_state_machine.h"
#include "cmsis_os2.h"
#include "ee_control_drv.h"
#include "jointFollowAngle.h"
#include "joint_control_drv.h"
#include "stm32h7xx_hal_def.h"
#include "trajectory_publisher_drv.h"

void Trajectory_Timer_Init(void);


#endif
