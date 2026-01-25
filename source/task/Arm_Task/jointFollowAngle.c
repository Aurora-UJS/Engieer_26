// jointFollowAngle.c 关节跟随角度运动处理函数

#include "arm_state_machine_task.h"
#include "cmsis_os2.h"
#include "ee_control_drv.h"
#include "joint_control_drv.h"

extern float Ctrller_Joint_Radian[6];
extern DM_motor_t *Joint_Motor[JOINT_NUM];
float Mannal_Joint_Radian[6] = {0};


extern arm_control_mode_t Arm_Current_Control_Mode;

extern gripper_control_mode_t Gripper_Current_Control_Mode;

void Gripper_Control_Mode_Mangner(endEffector_t *EndEffector) {
  switch (Gripper_Current_Control_Mode) {
  case GRIPPER_IDLE_MODE:
    Gripper_Current_Control_Mode = GRIPPER_CLOSE_MODE;
    break;
  case GRIPPER_OPEN_MODE:
    Gripper_Open(EndEffector);
    break;
  case GRIPPER_CLOSE_MODE:
    Gripper_Close(EndEffector);
    break;
  }
}
void Joint_Control_Mode_Mangner(Joint_t *Joint) {
  switch (Arm_Current_Control_Mode) {
  case Arm_IDLE_Mode:
    Arm_Current_Control_Mode = Arm_Custom_Controller_Follow_Mode;
    break;
  case Arm_Custom_Controller_Follow_Mode:

    Parse_ControllerData_To_CtrllerRadian(CtrllerData, Ctrller_Joint_Radian);

    CtrllerData_To_InputRadian_Converter(Ctrller_Joint_Radian);

    // Joint_Custom_State_Motor_Ctrl(Joint, Ctrller_Joint_Radian);
    Joint_Move(Joint,  Ctrller_Joint_Radian);

    break;
  case Arm_Frozen_Mode:
    Joint_Custom_State_Motor_Ctrl(Joint, Ctrller_Joint_Radian);
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

    osDelay(1);
  }
}
