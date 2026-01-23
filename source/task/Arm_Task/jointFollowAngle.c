// jointFollowAngle.c 关节跟随角度运动处理函数

#include "ee_control_drv.h"
#include "joint_control_drv.h"
#include "arm_state_machine_task.h"

extern float joint_radian[6];
extern DM_motor_t *Joint_Motor[JOINT_NUM];
float Mannal_Joint_Radian[6] = {0};

extern arm_control_mode_t Arm_Current_Control_Mode;

void jointFollowAngle(void *argument) {
  Joint_t Joint[JOINT_NUM];
  endEffector_t EndEffector;

  UNUSED(argument);
  joint_init(Joint);
  endEffector_init(&EndEffector);

  gripper_state_t GRIPPER_STATE = GRIPPER_STATE_IDLE;
  osDelay(100);
  Joint_Motor_Enable(Joint); // 使能所有关节电机
  EndEffector_Motor_Enable(&EndEffector);

  while (1) {

    Joint_Motor_Refresh(Joint);

    EndEffector_Motor_Refresh(&EndEffector);

    switch (Arm_Current_Control_Mode) {
      case Arm_IDLE_Mode:
        Arm_Current_Control_Mode = Arm_Custom_Controller_Follow_Mode;
        break;
      case Arm_Custom_Controller_Follow_Mode:
        Parse_ControllerData_To_JointRadian(CtrllerData, joint_radian);
        Joint_Custom_State_Motor_Ctrl(Joint, joint_radian);
        break;
      case Arm_Frozen_Mode:
    }

    switch (GRIPPER_STATE) {
    case GRIPPER_STATE_IDLE:
      GRIPPER_STATE = GRIPPER_STATE_OPEN;
      break;
    case GRIPPER_STATE_OPEN:
      Gripper_Open(&EndEffector);
      break;
    case GRIPPER_STATE_CLOSE:
      Gripper_Close(&EndEffector);
      break;
    }
  }
}
