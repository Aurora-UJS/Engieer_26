#ifndef TRAJECTORY_PUBLISHER_TASK_H
#define TRAJECTORY_PUBLISHER_TASK_H

#include "cmsis_os2.h"
#include "joint_control_drv.h"
#include "Trajectory_Data.h"

#define TRAJECTORY_LEN 3000
#define TRAJ_START_FLAG (1U << 0)

void Trajectory_Timer_Init(void);
#endif
