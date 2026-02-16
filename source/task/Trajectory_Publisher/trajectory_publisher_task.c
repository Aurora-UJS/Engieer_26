#include "trajectory_publisher_task.h"

static osTimerId_t traj_timer_id;
int traj_point_index = 0;
static int demo_index = 0;

void Trajectory_Timer_Callback(void *argument) {
  UNUSED(argument);
  // placeLeft();
  placeRight();
  // getLeft();
  // Debug_set_pos();
  // getRight();
  // if (traj_point_index >= (ARM_TOTOAL_TRAJECTORY_TIME_TICKS)) {
  if (traj_point_index >= 4000) {
    osTimerStop(traj_timer_id); // 停定时器
    traj_point_index = 0;
    Arm_Current_Control_Mode = Arm_Frozen_Mode;
  }
}

void Trajectory_demo_Timer_Callback(void *argument) {
  UNUSED(argument);

  if (demo_index == 0) {
    getRight();
    if (traj_point_index >= 3000) {
      traj_point_index = 0;
      demo_index++;
    }
  }

  else if (demo_index == 1) {
    placeLeft();
    if (traj_point_index >= 4000) {
      traj_point_index = 0;
      demo_index++;
    }
  }

  else if (demo_index == 2) {
    getLeft();
    if (traj_point_index >= 3000) {
      traj_point_index = 0;
      demo_index++;
    }
  }

  else if (demo_index == 3) {
    placeRight();
    if (traj_point_index >= 4000) {
      traj_point_index = 0;
      demo_index++;
    }
  }

  else {
    osTimerStop(traj_timer_id);
    demo_index = 0;
    traj_point_index = 0;
    Arm_Current_Control_Mode = Arm_Frozen_Mode;
  }
}
void Trajectory_Timer_Init(void) {
  osTimerAttr_t timer_attr = {0};
  timer_attr.name = "traj_timer";
  traj_timer_id =
      // osTimerNew(Trajectory_Timer_Callback, osTimerPeriodic, NULL, &timer_attr);
    osTimerNew(Trajectory_demo_Timer_Callback, osTimerPeriodic, NULL, &timer_attr);
}

void Trajectory_Publisher_Task(void *argument) {
  UNUSED(argument);
  osThreadFlagsWait(TRAJ_START_FLAG, osFlagsWaitAll, osWaitForever);
  osTimerStart(traj_timer_id, 7);
  for (;;) {
    osDelay(100);
  }
}
