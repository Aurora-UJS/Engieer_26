#include "trajectory_publisher_task.h"
#include "arm_state_machine.h"
#include "trajectory_publisher_drv.h"

static osTimerId_t traj_timer_id;
int traj_point_index = 0;
static int demo_index = 0;

Command_Place_And_Get_t cmd_place_get = Command_getRight;

void Command_Place_And_Get_Manager(Command_Place_And_Get_t cmd) {
  switch (cmd) {
  case Command_placeLeft:
    newPlaceLeft();
    break;
  case Command_placeRight:
    newPlaceRigth();
    break;
  case Command_getLeft:
    Get(Command_getLeft);
    break;
  case Command_getRight:
    Get(Command_getRight);
    // getRight();
    break;
  default:
    Get(Command_getRight);
  }
}
void Trajectory_Timer_Callback(void *argument) {
  UNUSED(argument);
  Command_Place_And_Get_Manager(cmd_place_get);
  if (traj_point_index >= 4000) {
    osTimerStop(traj_timer_id); // 停定时器
    traj_point_index = 0;
    Arm_Current_Control_Mode = Arm_Frozen_Mode;
  }
}
void demo(void) {
  if (demo_index == 0) {
    Get(Command_getRight);
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
    Get(Command_getLeft);
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

#define DURATION_STAGE_1 100
#define DURATION_STAGE_2 250
#define DURATION_STAGE_3 180
#define DURATION_STAGE_4 100

#define PL_LEFT_STAGE0_DURATION 300  // 阶段0持续时间
#define PL_LEFT_STAGE1_DURATION 500 // 阶段1持续时间
#define PL_LEFT_STAGE2_DURATION 800  // 阶段2持续时间
#define PL_LEFT_STAGE3_DURATION 500  // 阶段3持续时间
#define PL_LEFT_STAGE4_DURATION 200  // 阶段4持续时间，可根据需要0

typedef enum {
  STAGE_PREPARE,
  STAGE_1,
  STAGE_2,
  STAGE_3,
  STAGE_4,
  STAGE_DONE
} Get_Stage_t;
typedef enum {
  PUT_STAGE_PREPARE,
  PUT_STAGE_YAW,
  PUT_STAGE_PUT,
  PUT_STAGE_BACK,
  PUT_STAGE_DOWN
} Put_Stage_t;
const int stage_duration[] = {DURATION_STAGE_1, DURATION_STAGE_2,
                              DURATION_STAGE_3, DURATION_STAGE_4};
Get_Stage_t get_stage(int t) {
  if (t < 0)
    return STAGE_PREPARE;

  int acc = 0;

  for (int i = 0; i < 4; i++) {
    acc += stage_duration[i];
    if (t < acc)
      return (Get_Stage_t)(i + 1);
  }

  return STAGE_DONE;
}

void newGetRight(void) {
  int t = traj_point_index - ARM_PREPARE_TIME_TICKS;
  Get_Stage_t stage = get_stage(t);

  switch (stage) {
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
void newGetLeft(void) {
  int t = traj_point_index - ARM_PREPARE_TIME_TICKS;
  Get_Stage_t stage = get_stage(t);
  switch (stage) {
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

void Get(Command_Place_And_Get_t cmd_get) {
  int direct = (cmd_get == Command_getRight) ? (1) : (-1);
  int t = traj_point_index - ARM_PREPARE_TIME_TICKS;
  Get_Stage_t stage = get_stage(t);

  switch (stage) {
  case STAGE_PREPARE:
    Target_Point[0].target_joint_radian = 0.76f * direct;
    Target_Point[0].velocity = 1.5f;
    break;

  case STAGE_1:
    Target_Point[2].target_joint_radian = -0.18f;
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

void newPlaceRigth(void) {
  if (traj_point_index <= 1300) {
    Gripper_Current_Control_Mode = GRIPPER_CLOSE_MODE;
  } else {
    Gripper_Current_Control_Mode = GRIPPER_OPEN_MODE;
  }
  static int stage = 0;     // 当前阶段
  int t = traj_point_index; // 当前时间点
  int acc = 0;              // 累计时间，用于判断阶段切换

  switch (stage) {
  case 0:
    Target_Point[4].velocity = 2.5f;
    Target_Point[1].target_joint_radian = 0.49139f;
    Target_Point[2].target_joint_radian = 0.45f;
    Target_Point[3].target_joint_radian = 0.0f;
    Target_Point[4].target_joint_radian = 0.28f;
    Target_Point[5].target_joint_radian = 0.0f;

    Gripper_Current_Control_Mode = GRIPPER_CLOSE_MODE;

    acc = PL_LEFT_STAGE0_DURATION;
    if (t >= acc)
      stage++;
    break;

  case 1:
    Target_Point[0].target_joint_radian = 0.73f;
    Target_Point[0].velocity = 1.5f;

    acc = PL_LEFT_STAGE0_DURATION + PL_LEFT_STAGE1_DURATION;
    if (t >= acc)
      stage++;
    break;

  case 2:
    Target_Point[1].target_joint_radian = 0.55f;
    Target_Point[2].target_joint_radian = 0.10f;
    Target_Point[2].velocity = 0.5f;

    acc = PL_LEFT_STAGE0_DURATION + PL_LEFT_STAGE1_DURATION +
          PL_LEFT_STAGE2_DURATION;
    if (t >= acc)
      stage++;
    break;

  case 3:
    Target_Point[1].target_joint_radian = 0.0f;
    Target_Point[1].velocity = 0.2f;

    Target_Point[2].target_joint_radian = -0.05f;
    Target_Point[2].velocity = 0.6f;

    acc = PL_LEFT_STAGE0_DURATION + PL_LEFT_STAGE1_DURATION +
          PL_LEFT_STAGE2_DURATION + PL_LEFT_STAGE3_DURATION;
    if (t >= acc)
      stage++;
    break;

  case 4:
    Target_Point[0].target_joint_radian = 0.0f;
    Target_Point[0].velocity = 1.5f;

    break;
  }

  traj_point_index++;
}
void newPlaceLeft(void) {
  if (traj_point_index <= 1300) {
    Gripper_Current_Control_Mode = GRIPPER_CLOSE_MODE;
  } else {
    Gripper_Current_Control_Mode = GRIPPER_OPEN_MODE;
  }
  static int stage = 0;     // 当前阶段
  int t = traj_point_index; // 当前时间点
  int acc = 0;              // 累计时间，用于判断阶段切换

  switch (stage) {
  case 0:
    Target_Point[4].velocity = 2.5f;
    Target_Point[1].target_joint_radian = 0.49139f;
    Target_Point[2].target_joint_radian = 0.40f;
    Target_Point[3].target_joint_radian = 0.0f;
    Target_Point[4].target_joint_radian = 0.28f;
    Target_Point[5].target_joint_radian = 0.0f;

    Gripper_Current_Control_Mode = GRIPPER_CLOSE_MODE;

    acc = PL_LEFT_STAGE0_DURATION;
    if (t >= acc)
      stage++;
    break;

  case 1:
    Target_Point[0].target_joint_radian = -0.79f;
    Target_Point[0].velocity = 1.5f;

    acc = PL_LEFT_STAGE0_DURATION + PL_LEFT_STAGE1_DURATION;
    if (t >= acc)
      stage++;
    break;

  case 2:
    Target_Point[2].target_joint_radian = 0.26f;
    Target_Point[2].velocity = 0.5f;

    acc = PL_LEFT_STAGE0_DURATION + PL_LEFT_STAGE1_DURATION +
          PL_LEFT_STAGE2_DURATION;
    if (t >= acc)
      stage++;
    break;

  case 3:
    Target_Point[1].target_joint_radian = 0.0f;
    Target_Point[1].velocity = 0.2f;

    Target_Point[2].target_joint_radian = -0.05f;
    Target_Point[2].velocity = 0.6f;

    acc = PL_LEFT_STAGE0_DURATION + PL_LEFT_STAGE1_DURATION +
          PL_LEFT_STAGE2_DURATION + PL_LEFT_STAGE3_DURATION;
    if (t >= acc)
      stage++;
    break;

  case 4:
    Target_Point[0].target_joint_radian = 0.0f;
    Target_Point[0].velocity = 1.5f;

    break;
  }

  traj_point_index++;
}
void Trajectory_demo_Timer_Callback(void *argument) {
  UNUSED(argument);
  demo();
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
  osTimerStart(traj_timer_id, 5);
  for (;;) {
    osDelay(100);
  }
}
