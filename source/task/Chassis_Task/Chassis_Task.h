#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "cmsis_os2.h"
#include "chassis_config.h"

#include <stdint.h>

void Chassis_Task(void *argument);

void Chassis_ForcePowerOff(uint8_t enable);
uint8_t Chassis_IsForcePowerOff(void);

#endif // !CHASSIS_TASK_H
