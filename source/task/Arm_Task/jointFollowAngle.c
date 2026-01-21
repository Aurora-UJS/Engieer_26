// jointFollowAngle.c 关节跟随角度运动处理函数

#include "ee_control_drv.h"
#include "joint_control_drv.h"

extern float joint_radian[6];
extern DM_motor_t *Joint_Motor[JOINT_NUM];

// ch1 右摇杆 左右 左-右+
// ch2 右摇杆 前后 前+后-
// ch3 左摇杆 左右 左-右+
// ch4 左摇杆 前后 前+后-
// sw1 左拨码开关 前1 中3 后2
// sw2 右拨码开关 前1 中3 后2

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

    Parse_ControllerData_To_JointRadian(CtrllerData, joint_radian);

    Joint_Custom_State_Motor_Ctrl(Joint, joint_radian);

    switch (GRIPPER_STATE) {

    case GRIPPER_STATE_IDLE:

    case GRIPPER_STATE_OPEN:
      PosSpeed_CtrlMotorDM(EndEffector.endEffector_motor, 0, 0.5); // 夹爪控制
      osDelay(1);
      break;
    case GRIPPER_STATE_CLOSE:
      PosSpeed_CtrlMotorDM(EndEffector.endEffector_motor, 1.2, 0.5); // 夹爪控制
      osDelay(1);
      break;
    }
  }
}
