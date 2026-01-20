#ifndef JOINT_CONTROL_DRV
#define JOINT_CONTROL_DRV

#include "motor_DM.h"
#include "cmsis_os2.h"
#include "tool.h"
#include "arm_math.h"
#include <stdio.h>
#include "DBusSys.h"
#define JOINT_NUM 7

/** @brief 电机初始化*/
void joint_motor_init(void);
/** @brief 电机信息更新 */
void Joint_Motor_Refresh(void);
/** @brief 电机使能 */
void Joint_Motor_Enable(void);

extern uint8_t CtrllerData[24];
extern rc_info_t remoter;
#endif
