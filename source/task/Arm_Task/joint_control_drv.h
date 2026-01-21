#ifndef JOINT_CONTROL_DRV
#define JOINT_CONTROL_DRV

#include "motor_DM.h"
#include "cmsis_os2.h"
#include "tool.h"
#include "arm_math.h"
#include <stdio.h>
#include "DBusSys.h"
#include "can_struct.h"

#define JOINT_NUM 6
#define JOINT_DEFAULT_VELOCITY 0.5f

typedef enum{
   JOINT_DOF_ROLL,
   JOINT_DOF_YAW,
   JOINT_DOF_PITCH
} joint_dof_t;

static const can_port_t can_port_map[JOINT_NUM] = {
    CAN3_PORT,
    CAN2_PORT,
    CAN2_PORT,
    CAN2_PORT,
    CAN2_PORT,
    CAN2_PORT,
};
static const joint_dof_t joint_dof_map[JOINT_NUM] = {
    JOINT_DOF_YAW,
    JOINT_DOF_PITCH,
    JOINT_DOF_PITCH,
    JOINT_DOF_ROLL,
    JOINT_DOF_PITCH,
    JOINT_DOF_ROLL
};

typedef struct Joint_t{
    DM_motor_t *joint_motor;
    joint_dof_t dof;
    float target_radian;
    float current_radian;
}Joint_t ;

extern uint8_t CtrllerData[24];
extern rc_info_t remoter;

void Joint_Motor_PosSpeed_Ctrl(Joint_t *Joint,float target_radian,float velocity);
void Joint_Pos_Ctrl(Joint_t *Joint,float target_radian);
/** @brief 关节初始化*/
void joint_init(Joint_t *Joint);


/** @brief 关节自由度信息初始化 */
void joint_dof_init(Joint_t *Joint);

/** @brief 电机初始化*/
void joint_motor_init(Joint_t *Joint);

/** @brief 电机信息更新 */
void Joint_Motor_Refresh(Joint_t *Joint);

/** @brief 电机使能 */
void Joint_Motor_Enable(Joint_t *Joint);
#endif
