// jointFollowAngle.c 关节跟随角度运动处理函数

#include "jointFollowAngle.h"
#include "DBusSys.h"
#include "arm_state_machine.h"

extern float Ctrller_Joint_Radian[6];
// extern DM_motor_t *Joint_Motor[JOINT_NUM];
float Mannal_Joint_Radian[6] = {0};
target_point_t Target_Point[6];
float Target_Joint_Radian[6] = {0};

void target_point_init(target_point_t *Target_Point) {
  for (int joint_index = 0; joint_index < JOINT_NUM; joint_index++) {
    Target_Point[joint_index].target_joint_radian = 0;
    Target_Point[joint_index].velocity = JOINT_DEFAULT_VELOCITY;
  }
}

static inline float Motor_Get_Radian(const DM_motor_t *motor) {
  return motor->motor_msg.motor_angle;
}
float Current_Radian[6] = {0};
void Joint_Get_Radian(Joint_t Joint[],float rad[]){
  for (int joint_index=0; joint_index<JOINT_NUM; joint_index++) {
    rad[joint_index]=Motor_Get_Radian(Joint[joint_index].joint_motor);
  }
}
Joint_t Joint[JOINT_NUM];
void jointFollowAngle(void *argument) {
  endEffector_t EndEffector;

  UNUSED(argument);

  joint_init(Joint);

  target_point_init(Target_Point);

  endEffector_init(&EndEffector);

  osDelay(100);

  Joint_Motor_Enable(Joint); // 使能所有关节电机

  EndEffector_Motor_Enable(&EndEffector);

  while (1) {

    if (remoter.sw1 == 1) {
      Arm_Current_Control_Mode = Arm_Rising_Mode;
    }
    if (remoter.sw1 != 1) {
      Arm_Current_Control_Mode = Arm_Zero_Mode;
    }
    Joint_Move(Joint, Target_Point);

    Joint_Get_Radian(Joint,Current_Radian);

    Joint_Motor_Refresh(Joint);

    EndEffector_Motor_Refresh(&EndEffector);

    Gripper_Control_Mode_Manager(&EndEffector);

    Joint_Control_Mode_Manager(Joint);

    osDelay(1);
  }
}
