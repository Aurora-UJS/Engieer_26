#include "trajectory_publisher_task.h"

static osTimerId_t traj_timer_id;
int traj_point_index = 0;

void Trajectory_Timer_Callback(void *argument) {
  UNUSED(argument);
  // placeLeft();
  // placeRight();
  getLeft();
  // Debug_set_pos();
  // getRight();
  // if (traj_point_index >= (ARM_TOTOAL_TRAJECTORY_TIME_TICKS)) {
  if (traj_point_index >= ARM_TOTOAL_TRAJECTORY_TIME_TICKS) {
    osTimerStop(traj_timer_id); // 停定时器
    traj_point_index = 0;
    Arm_Current_Control_Mode = Arm_Frozen_Mode;
  }
}

void Trajectory_Timer_Init(void) {
  osTimerAttr_t timer_attr = {0};
  timer_attr.name = "traj_timer";
  traj_timer_id =
      osTimerNew(Trajectory_Timer_Callback, osTimerPeriodic, NULL, &timer_attr);
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
