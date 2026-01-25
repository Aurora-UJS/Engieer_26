#ifndef JOINTFOLLOWANGLE
#define JOINTFOLLOWANGLE

#include "arm_state_machine_task.h"
#include "cmsis_os2.h"
#include "ee_control_drv.h"
#include "joint_control_drv.h"
#include <string.h>
#include <stdbool.h>


/** @brief 任务jointFollowAngle */
void jointFollowAngle(void *argument);

#endif
