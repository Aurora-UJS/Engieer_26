// jointFollowAngle.c 关节跟随角度运动处理函数

#include "jointFollowAngle.h"
#include "joint_control_drv.h"
#include "arm_control_drv.h"

extern float Ctrller_Joint_Radian[6];
extern DM_motor_t *Joint_Motor[JOINT_NUM];
float Mannal_Joint_Radian[6] = {0};
target_point_t Target_Point[6];
float Target_Joint_Radian[6] = {0};


void target_point_init(target_point_t *Target_Point) {
  for (int joint_index = 0; joint_index < JOINT_NUM; joint_index++) {
    Target_Point[joint_index].target_joint_radian = 0;
    Target_Point[joint_index].velocity = JOINT_DEFAULT_VELOCITY;
  }
}

float Current_Radian[6] = {0};
void Joint_Get_Radian(Joint_t *Joint){
  for (int joint_index=0; joint_index<JOINT_NUM; joint_index++) {
    Current_Radian[joint_index]=Motor_Get_Radian(Joint[joint_index].joint_motor);
  }
}
void jointFollowAngle(void *argument) {
  Joint_t Joint[JOINT_NUM];
  endEffector_t EndEffector;

  UNUSED(argument);

  joint_init(Joint);

  target_point_init(Target_Point);

  endEffector_init(&EndEffector);

  osDelay(100);

  Joint_Motor_Enable(Joint); // 使能所有关节电机

  EndEffector_Motor_Enable(&EndEffector);

  while (1) {

    Joint_Motor_Refresh(Joint);

    EndEffector_Motor_Refresh(&EndEffector);
    // Gripper_Control_Mode_Mangner(&EndEffector);
    Joint_Motor_Disable(Joint);
    // Joint_Control_Mode_Mangner(Joint);
    Joint_Get_Radian(Joint);
    // Joint_Move_byPoint(Joint, Target_Point);

    osDelay(1);
  }
}
