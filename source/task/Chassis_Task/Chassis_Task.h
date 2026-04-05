#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "cmsis_os2.h"
#include "chassis_config.h"

#include <stdint.h>

typedef enum
{
    CHASSIS_MODE_STATE_PowerOff = 0,
    CHASSIS_MODE_STATE_Normal = 1,
    CHASSIS_MODE_STATE_Rising = 2,
} Chassis_Mode_State_t;

typedef enum
{
    CHASSIS_CONTROL_SOURCE_STATE_DBUS = 0,
    CHASSIS_CONTROL_SOURCE_STATE_Keyboard = 1,
} Chassis_Control_Source_State_t;

typedef enum
{
    CHASSIS_RISING_BEHAVIOR_STATE_SingleLift = 0,
    CHASSIS_RISING_BEHAVIOR_STATE_DoubleLift = 1,
} Chassis_Rising_Behavior_State_t;

extern volatile Chassis_Mode_State_t g_chassis_mode_state;
extern volatile Chassis_Control_Source_State_t g_chassis_control_source_state;
extern volatile Chassis_Rising_Behavior_State_t g_chassis_rising_behavior_state;

void Chassis_Task(void *argument);

void Chassis_ForcePowerOff(uint8_t enable);
uint8_t Chassis_IsForcePowerOff(void);
void Chassis_SetControlSourceState(Chassis_Control_Source_State_t source_state);
void Chassis_SetModeState(Chassis_Mode_State_t mode_state);
void Chassis_SetRisingBehaviorState(Chassis_Rising_Behavior_State_t behavior_state);
void Chassis_HandleRisingKeyPressed(void);
Chassis_Mode_State_t Chassis_GetModeState(void);
Chassis_Control_Source_State_t Chassis_GetControlSourceStatePublic(void);
Chassis_Rising_Behavior_State_t Chassis_GetRisingBehaviorState(void);

#endif // !CHASSIS_TASK_H
