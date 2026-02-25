#include "trajectory_publisher_task.h"
#include "arm_state_machine.h"
#include "trajectory_publisher_drv.h"

static osTimerId_t traj_timer_id;
int traj_point_index = 0;
static int demo_index = 0;

typedef enum Command_Place_And_Get {
  Command_placeRight,
  Command_placeLeft,
  Command_getRight,
  Command_getLeft
} Command_Place_And_Get_t;
Command_Place_And_Get_t cmd_place_get;

void Command_Place_And_Get_Manager(Command_Place_And_Get_t cmd) {
  switch (cmd) {
  case Command_placeLeft:
    placeLeft();
    break;
  case Command_placeRight:
    placeLeft();
    break;
  case Command_getLeft:
    getLeft();
    break;
  case Command_getRight:
    getRight();
    break;
  default:
    getRight();
  }
}
void Trajectory_Timer_Callback(void *argument) {
  UNUSED(argument);
  Command_Place_And_Get_Manager(cmd_place_get);
  // placeLeft();
  // getLeft();
  // Debug_set_pos();
  // getRight();
  // if (traj_point_index >= (ARM_TOTOAL_TRAJECTORY_TIME_TICKS)) {
  if (traj_point_index >= 4000) {
    osTimerStop(traj_timer_id); // 停定时器
    traj_point_index = 0;
    Arm_Current_Control_Mode = Arm_Custom_Controller_Follow_Mode;
  }
}
void demo(void) {
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

#define DURATION_STAGE_1   100
#define DURATION_STAGE_2   250
#define DURATION_STAGE_3   150
#define DURATION_STAGE_4   100

typedef enum {
    STAGE_PREPARE,
    STAGE_1,
    STAGE_2,
    STAGE_3,
    STAGE_4,
    STAGE_DONE
} Stage_t;

const int stage_duration[] = {
    DURATION_STAGE_1,
    DURATION_STAGE_2,
    DURATION_STAGE_3,
    DURATION_STAGE_4
};
Stage_t get_stage(int t)
{
    if (t < 0)
        return STAGE_PREPARE;

    int acc = 0;

    for (int i = 0; i < 4; i++)
    {
        acc += stage_duration[i];
        if (t < acc)
            return (Stage_t)(i + 1);
    }

    return STAGE_DONE;
}
void newGetRight(void)
{
    int t = traj_point_index - ARM_PREPARE_TIME_TICKS;
    Stage_t stage = get_stage(t);

    switch (stage)
    {
        case STAGE_PREPARE:
            Target_Point[0].target_joint_radian = 0.76f;
            Target_Point[0].velocity = 1.5f;
            break;

        case STAGE_1:
            Target_Point[2].target_joint_radian = -0.15f;
            Target_Point[2].velocity = 1.0f;
            break;

        case STAGE_2:
            Target_Point[1].target_joint_radian = 0.7f;
            Target_Point[1].velocity = 1.0f;

            Target_Point[4].target_joint_radian = 0.78f;
            Target_Point[4].velocity = 1.0f;

            Target_Point[2].target_joint_radian = 0.1f;
            Target_Point[2].velocity = 0.1f;
            break;

        case STAGE_3:
            Target_Point[4].target_joint_radian = 1.0f;
            Target_Point[4].velocity = 1.0f;

            Target_Point[2].target_joint_radian = 0.7f;
            Target_Point[2].velocity = 1.0f;
            break;

        case STAGE_4:
            Target_Point[0].target_joint_radian = 0.0f;
            Target_Point[0].velocity = 1.5f;
            break;

        case STAGE_DONE:
            target_point_init(Target_Point);
            break;
    }

    /* 夹爪逻辑 */
    if (t >= (DURATION_STAGE_1 + DURATION_STAGE_2 + 50))
        Gripper_Current_Control_Mode = GRIPPER_CLOSE_MODE;

    traj_point_index++;
}
void newGetLeft(void)
{
    int t = traj_point_index - ARM_PREPARE_TIME_TICKS;
    Stage_t stage = get_stage(t);

    switch (stage)
    {
        case STAGE_PREPARE:
            Target_Point[0].target_joint_radian = -0.76f;
            Target_Point[0].velocity = 1.5f;
            break;

        case STAGE_1:
            Target_Point[2].target_joint_radian = -0.15f;
            Target_Point[2].velocity = 1.0f;
            break;

        case STAGE_2:
            Target_Point[1].target_joint_radian = 0.7f;
            Target_Point[1].velocity = 1.0f;

            Target_Point[4].target_joint_radian = 0.78f;
            Target_Point[4].velocity = 1.0f;

            Target_Point[2].target_joint_radian = 0.1f;
            Target_Point[2].velocity = 0.1f;
            break;

        case STAGE_3:
            Target_Point[4].target_joint_radian = 1.0f;
            Target_Point[4].velocity = 1.0f;

            Target_Point[2].target_joint_radian = 0.7f;
            Target_Point[2].velocity = 1.0f;
            break;

        case STAGE_4:
            Target_Point[0].target_joint_radian = 0.0f;
            Target_Point[0].velocity = 1.5f;
            break;

        case STAGE_DONE:
            target_point_init(Target_Point);
            break;
    }

    /* 夹爪逻辑 */
    if (t >= (DURATION_STAGE_1 + DURATION_STAGE_2 + 50))
        Gripper_Current_Control_Mode = GRIPPER_CLOSE_MODE;

    traj_point_index++;
}
void getRight_Callback(void *argument) {
  UNUSED(argument);
  newGetRight();
  if (traj_point_index >=3000) {
    traj_point_index = 0;
    osTimerStop(traj_timer_id);
    Arm_Current_Control_Mode = Arm_Frozen_Mode;
  }
}

void Trajectory_demo_Timer_Callback(void *argument) {
  UNUSED(argument);
  demo();
}
void Trajectory_Timer_Init(void) {
  osTimerAttr_t timer_attr = {0};
  timer_attr.name = "traj_timer";
  traj_timer_id =
      osTimerNew(getRight_Callback, osTimerPeriodic, NULL, &timer_attr);
}

void Trajectory_Publisher_Task(void *argument) {
  UNUSED(argument);
  osThreadFlagsWait(TRAJ_START_FLAG, osFlagsWaitAll, osWaitForever);
  osTimerStart(traj_timer_id, 5);
  for (;;) {
    osDelay(100);
  }
}
