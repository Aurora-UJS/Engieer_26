#include "trajectory_publisher_task.h"
#include "Trajectory_Data.h"
#include "arm_state_machine.h"
#include "cmsis_os2.h"
#include "joint_control_drv.h"
#include "stm32h7xx_hal_def.h"

static osTimerId_t traj_timer_id;
extern target_point_t Target_Point[JOINT_NUM];
extern arm_control_mode_t Arm_Current_Control_Mode;
static int traj_point_index = 0;

extern const float Trajectory[Traj_Num][JOINT_NUM];


extern const float Traj_Vel[Traj_Num][JOINT_NUM];



void Trajectory_Publisher(const float Trajectory[][JOINT_NUM],int point_index){
for (int joint_index=0; joint_index<JOINT_NUM; joint_index++) {
    Target_Point[joint_index].target_joint_radian= Trajectory[point_index][joint_index];
    Target_Point[joint_index].velocity = Traj_Vel[point_index][joint_index];
  }
}
void Trajectory_Timer_Callback(void *argument){
  UNUSED(argument);
  Trajectory_Publisher(Trajectory, traj_point_index);
  traj_point_index++;
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
  osTimerStart(traj_timer_id, 13);
  for (;;) {
    osDelay(100);
  }
  // for (int point_index = 0; point_index < 1598; point_index++) {
  //   Trajectory_Publisher(Trajectory,  point_index);
  //   osDelay(10);
  // }
  // Arm_Current_Control_Mode = Arm_Frozen_Mode;
}
