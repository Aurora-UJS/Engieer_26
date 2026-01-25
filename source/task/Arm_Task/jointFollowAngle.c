// jointFollowAngle.c 关节跟随角度运动处理函数

#include "jointFollowAngle.h"
#include "joint_control_drv.h"
#include <stdbool.h>

#define Angle_Epsilon 0.005f
extern float Ctrller_Joint_Radian[6];
extern DM_motor_t *Joint_Motor[JOINT_NUM];
float Mannal_Joint_Radian[6] = {0};
float Target_Joint_Radian[6] = {0};

// 夹爪和臂状态引用
extern arm_control_mode_t Arm_Current_Control_Mode;

extern gripper_control_mode_t Gripper_Current_Control_Mode;

static inline float Float_Abs(float num) { return (num >= 0.0f) ? num : -num; }

static inline float Delat(float current, float target) {
  return current - target;
}

static inline float Error_Calc(float current, float target) {
  return Float_Abs(Delat(current, target));
}

// float at_angle_test;
static inline bool Joint_At_Target(Joint_t *Joint, float target_radian,
                                   float epsilon) {
  // at_angle_test = Error_Calc(Joint->joint_motor->motor_msg.motor_angle,
  // target_radian);
  if (Error_Calc(Joint->joint_motor->motor_msg.motor_angle, target_radian) <=
      epsilon) {
    return true;
  }
  return false;
}

bool Arm_Transition_Handle(Joint_t *Joint, const float *transition_radian) {
  for (int joint_index = 1; joint_index < JOINT_NUM; joint_index++) {
    /* if (-0.005 <= ((Joint[joint_index].joint_motor->motor_msg.motor_angle) -
                   (transition_radian[joint_index])) &&
        ((Joint[joint_index].joint_motor->motor_msg.motor_angle) -
         (transition_radian[joint_index])) <= 0.005) {
        transition_done_flag = 1;
    }
    else {
      transition_done_flag = 0;
      break;
    } */
    if (false == Joint_At_Target(&Joint[joint_index],
                                 transition_radian[joint_index],
                                 Angle_Epsilon)) {
      return false;
    }
  }

  return true;
}

/**
 * @brief 夹爪状态机
 *
 * @param EndEffector
 */
void Gripper_Control_Mode_Mangner(endEffector_t *EndEffector) {
  switch (Gripper_Current_Control_Mode) {
  case GRIPPER_IDLE_MODE:
    Gripper_Current_Control_Mode = GRIPPER_OPEN_MODE;
    break;
  case GRIPPER_OPEN_MODE:
    Gripper_Open(EndEffector);
    break;
  case GRIPPER_CLOSE_MODE:
    Gripper_Close(EndEffector);
    break;
  }
}
bool test_arm_handle;
/**
 * @brief 关节状态机
 *
 * @param Joint
 */
void Joint_Control_Mode_Mangner(Joint_t *Joint) {
  switch (Arm_Current_Control_Mode) {
  case Arm_IDLE_Mode:
    Arm_Current_Control_Mode = Arm_Custom_Controller_Follow_Mode;
    break;
  case Arm_TRANSITION_Mode:

    break;
  case Arm_Custom_Controller_Follow_Mode:

    Parse_ControllerData_To_CtrllerRadian(CtrllerData, Ctrller_Joint_Radian);

    CtrllerData_To_InputRadian_Converter(Ctrller_Joint_Radian);

    memcpy(Target_Joint_Radian, Ctrller_Joint_Radian,
           sizeof(Ctrller_Joint_Radian));
    // Joint_Custom_State_Motor_Ctrl(Joint, Ctrller_Joint_Radian);
    test_arm_handle = Arm_Transition_Handle(Joint, Mannal_Joint_Radian);
    break;
  case Arm_Frozen_Mode:
    break;

  case Arm_Set_Radian:
    Joint_Mannal_State_Motor_Ctrl(Joint, Mannal_Joint_Radian);
    break;
  }
}

void jointFollowAngle(void *argument) {
  Joint_t Joint[JOINT_NUM];
  endEffector_t EndEffector;

  UNUSED(argument);
  joint_init(Joint);
  endEffector_init(&EndEffector);

  osDelay(100);
  Joint_Motor_Enable(Joint); // 使能所有关节电机
  EndEffector_Motor_Enable(&EndEffector);
  while (1) {

    Joint_Motor_Refresh(Joint);

    EndEffector_Motor_Refresh(&EndEffector);

    Gripper_Control_Mode_Mangner(&EndEffector);

    Joint_Control_Mode_Mangner(Joint);

    Joint_Move(Joint, Target_Joint_Radian);
    osDelay(1);
  }
}
