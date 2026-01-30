#ifndef JOINTFOLLOWANGLE
#define JOINTFOLLOWANGLE

#include "cmsis_os2.h"
#include "ee_control_drv.h"
#include "joint_control_drv.h"
#include <string.h>
#include <stdbool.h>
#include "arm_state_machine.h"
#include "DBusSys.h"

/** @brief 任务jointFollowAngle */
void jointFollowAngle(void *argument);

#endif
