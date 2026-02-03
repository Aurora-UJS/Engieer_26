#include "trajectory_publisher_task.h"
#include "Trajectory_Data.h"
#include "arm_state_machine.h"
#include "ee_control_drv.h"
#include "cmsis_os2.h"
#include "jointFollowAngle.h"
#include "joint_control_drv.h"
#include "stm32h7xx_hal_def.h"

static osTimerId_t traj_timer_id;
extern target_point_t Target_Point[JOINT_NUM];
extern arm_control_mode_t Arm_Current_Control_Mode;
extern gripper_control_mode_t Gripper_Current_Control_Mode;
static int traj_point_index = 0;

extern const float Trajectory[Traj_Num][JOINT_NUM];

extern const float Traj_Vel[Traj_Num][JOINT_NUM];

extern const target_point_t Left_Rotation_Point[6];


void Trajectory_Publisher_right(const float Trajectory[][JOINT_NUM],int point_index){
for (int joint_index=0; joint_index<JOINT_NUM; joint_index++) {
    if (joint_index == 0){
      Target_Point[joint_index].target_joint_radian= (-1.0f)*Trajectory[point_index][joint_index];
    }
    else {
      Target_Point[joint_index].target_joint_radian= Trajectory[point_index][joint_index];
    }
    Target_Point[joint_index].velocity = Traj_Vel[point_index][joint_index]*(-2.0f);
  }
}
void Trajectory_Publisher(const float Trajectory[][JOINT_NUM],int point_index){
for (int joint_index=0; joint_index<JOINT_NUM; joint_index++) {
    Target_Point[joint_index].target_joint_radian= Trajectory[point_index][joint_index];
    Target_Point[joint_index].velocity = Traj_Vel[point_index][joint_index]*2.0f;
  }
}
void getLeft(void){

  if (traj_point_index<=1000) {
    Target_Point[0].target_joint_radian= -0.79f;
    Target_Point[0].velocity = 0.5f;
    Target_Point[4].target_joint_radian = 0.0f;
    Target_Point[4].velocity = 0.5f;
  }
  else if (traj_point_index >=4192) {
    target_point_init(Target_Point);
  }
  else {
    Trajectory_Publisher(Trajectory, traj_point_index-1000);
  }
  if (traj_point_index == 3000) {
    Gripper_Current_Control_Mode = GRIPPER_CLOSE_MODE;
  }
  traj_point_index++;
}
void getRight(void){
  if (traj_point_index<=1000) {
    Target_Point[0].target_joint_radian= 0.79f;
    Target_Point[0].velocity = 0.5f;
    Target_Point[4].target_joint_radian = 0.0f;
    Target_Point[4].velocity = 0.5f;
  }
  else if (traj_point_index >=4192) {
    target_point_init(Target_Point);
  }
  else {
    Trajectory_Publisher_right(Trajectory, traj_point_index-1000);
  }
  if (traj_point_index == 3000) {
    Gripper_Current_Control_Mode = GRIPPER_CLOSE_MODE;
  }
  traj_point_index++;
}
void Trajectory_Timer_Callback(void *argument){
  UNUSED(argument);
  getRight();
  if (traj_point_index >= Traj_Num) {
        osTimerStop(traj_timer_id);  // 停定时器
        traj_point_index = 0;
        Arm_Current_Control_Mode = Arm_Frozen_Mode;
    }
}

void Trajectory_Timer_Init(void)
{
    osTimerAttr_t timer_attr = {0};
    timer_attr.name = "traj_timer";

    traj_timer_id = osTimerNew(Trajectory_Timer_Callback, osTimerPeriodic, NULL, &timer_attr);
}

void Trajectory_Publisher_Task(void *argument) {
  UNUSED(argument);
  osThreadFlagsWait(TRAJ_START_FLAG, osFlagsWaitAll, osWaitForever);
  osTimerStart(traj_timer_id, 7);
  for (;;) {
    osDelay(100);
  }
  // for (int point_index = 0; point_index < 1598; point_index++) {
  //   Trajectory_Publisher(Trajectory,  point_index);
  //   osDelay(10);
  // }
  // Arm_Current_Control_Mode = Arm_Frozen_Mode;
}
