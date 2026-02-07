#ifndef REFEREE_TASK_H
#define REFEREE_TASK_H

#include "cmsis_os2.h"
#include "referee_protocol.h"
#include "arm_math.h"
#include <stdint.h>

#define CHASSIS_CTRL_MODE_Normal 0
#define CHASSIS_CTRL_MODE_Rising 1
#define ARM_CTRL_MODE_CustomCtrl 0
#define ARM_CTRL_MODE_AutoCtrl 1
#define ARM_CTRL_MODE_Rising 2
#define CTRL_LOGIC_MODE_DBUS 0
#define CTRL_LOGIC_MODE_Keyboard 1

void Referee_Task(void *argument);

void Referee_KeyboardEdgeDetect(const keyboard_t *kb);

void Referee_OnKeyboardKeyPressed(uint8_t key);
typedef struct
{
    uint8_t Chassis_Ctrl_Mode;
    uint8_t Arm_Ctrl_Mode;
    uint8_t Ctrl_Logic_Mode;
} Engineer_Mode_t;

extern Engineer_Mode_t Engineer_Mode;

#endif
