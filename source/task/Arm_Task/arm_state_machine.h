#ifndef ARM_STATE_MACHINE_H
#define  ARM_STATE_MACHINE_H

#include "DBusSys.h"
#include "joint_control_drv.h"
#include "ee_control_drv.h"
#include "trajectory_publisher_task.h"

typedef enum {
  Arm_IDLE_Mode = 0,
  Arm_Custom_Controller_Follow_Mode,
  Arm_Frozen_Mode,
  Arm_Set_Radian,
  Arm_Transition_Mode,
  Arm_Traj_Mode,
  Arm_Rising_Mode
} arm_control_mode_t;

extern float Ctrller_Joint_Radian[6];
extern DM_motor_t *Joint_Motor[JOINT_NUM];
extern float Mannal_Joint_Radian[6];
extern target_point_t Target_Point[6];
extern float Target_Joint_Radian[6];
extern rc_info_t remoter;

/** @brief 夹爪状态机管理器 */
void Gripper_Control_Mode_Mangner(endEffector_t *EndEffector);
/** @brief 关节状态机管理器 */
void Joint_Control_Mode_Mangner(Joint_t *Joint);
#endif
