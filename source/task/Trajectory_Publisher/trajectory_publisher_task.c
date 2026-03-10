#include "trajectory_publisher_task.h"
#include "arm_state_machine.h"
#include "jointFollowAngle.h"
#include "joint_control_drv.h"
#include "trajectory_publisher_drv.h"

static osTimerId_t traj_timer_id;
int traj_point_index = 0;
static int demo_index = 0;

Command_Place_And_Get_t cmd_place_get = Command_getLeft;

#define PB_STAGE_A_DURATION 200
#define PB_STAGE_B_DURATION 400
#define PB_STAGE_C_DURATION 500
#define PB_STAGE_D_DURATION 500
#define PB_STAGE_E_DURATION 500
#define PB_STAGE_F_DURATION 500
#define PB_STAGE_G_DURATION 500
#define PB_STAGE_H_DURATION 500

#define GB_DURATION_PREPARE   200
#define GB_DURATION_1         500
#define GB_DURATION_2         500
#define GB_DURATION_3         100
#define GB_DURATION_4         500
#define GB_DURATION_5         500
#define GB_DURATION_6         300
#define GB_TOTAL_TIME (GB_DURATION_PREPARE + GB_DURATION_1 + GB_DURATION_2 + GB_DURATION_3 + GB_DURATION_4 + GB_DURATION_5 + GB_DURATION_6)

const int place_back_duration[] = {
    PB_STAGE_A_DURATION,
    PB_STAGE_B_DURATION,
    PB_STAGE_C_DURATION,
    PB_STAGE_D_DURATION,
    PB_STAGE_E_DURATION,
    PB_STAGE_F_DURATION,
    PB_STAGE_G_DURATION,
    PB_STAGE_H_DURATION
};

void Command_Place_And_Get_Manager(Command_Place_And_Get_t cmd) {
  switch (cmd) {
  case Command_Error:
    Arm_Current_Control_Mode = Arm_Frozen_Mode;
    break;
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
void Get_Back_Timer_Callback(void *argument){
  UNUSED(argument);
  Get(Command_getLeft);
}

void Trajectory_Timer_Callback(void *argument) {
  UNUSED(argument);
  Command_Place_And_Get_Manager(cmd_place_get);
  if (traj_point_index >= 4000) {
    osTimerStop(traj_timer_id); // 停定时器
    traj_point_index = 0;
    Arm_Current_Control_Mode = Arm_IDLE_Mode;
  }
}

typedef enum {
    PB_STAGE_A,
    PB_STAGE_B,
    PB_STAGE_C,
    PB_STAGE_D,
    PB_STAGE_E,
    PB_STAGE_F,
    PB_STAGE_G,
    PB_STAGE_H,
    PB_STAGE_DONE
} Place_Back_Stage_t;


Place_Back_Stage_t place_back_stage(int t)
{
    int acc = 0;

    for(int i = 0; i < 8; i++)
    {
        acc += place_back_duration[i];

        if(t < acc)
            return (Place_Back_Stage_t)i;
    }

    return PB_STAGE_DONE;
}

void place_back(void)
{
    int t = traj_point_index;

    Place_Back_Stage_t stage = place_back_stage(t);

    switch(stage)
    {

    case PB_STAGE_A:
        Gripper_Current_Control_Mode = GRIPPER_CLOSE_MODE;
        Target_Point[0].target_joint_radian = 0.0f;
        Target_Point[1].target_joint_radian = 0.0f;
        Target_Point[2].target_joint_radian = 0.58f;
        Target_Point[4].target_joint_radian = 0.0f;
        Target_Point[5].target_joint_radian = 0.0f;
        break;

    case PB_STAGE_B:
        Target_Point[0].velocity = 1.5f;
        Target_Point[0].target_joint_radian = -2.25f;
        break;

    case PB_STAGE_C:
        Gripper_Current_Control_Mode = GRIPPER_OPEN_MODE;
        Target_Point[1].target_joint_radian = 0.24f;
        Target_Point[2].target_joint_radian = 0.25f;
        Target_Point[4].target_joint_radian = 0.0f;
        break;

    case PB_STAGE_D:
        Target_Point[1].target_joint_radian = 0.0f;
        Target_Point[2].target_joint_radian = -0.1f;
        break;

    case PB_STAGE_E:
        Target_Point[4].target_joint_radian = 0.6f;
        break;

    case PB_STAGE_F:
        Target_Point[0].target_joint_radian = 0.0f;
        break;

    case PB_STAGE_G:
        Target_Point[4].target_joint_radian = 0.0f;
        break;

    case PB_STAGE_H:
        target_point_init(Target_Point);
        break;

    case PB_STAGE_DONE:
        osTimerStop(traj_timer_id);
        traj_point_index = 0;
        Arm_Current_Control_Mode = Arm_IDLE_Mode;
        return;
    }

    traj_point_index++;
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

#define DURATION_STAGE_INIT 500
#define DURATION_STAGE_PREPARE 300
#define DURATION_STAGE_1 300
#define DURATION_STAGE_2 250
#define DURATION_STAGE_3 180
#define DURATION_STAGE_4 100

#define PL_LEFT_STAGE0_DURATION 300  // 阶段0持续时间
#define PL_LEFT_STAGE1_DURATION 500 // 阶段1持续时间
#define PL_LEFT_STAGE2_DURATION 800  // 阶段2持续时间
#define PL_LEFT_STAGE3_DURATION 500  // 阶段3持续时间
#define PL_LEFT_STAGE4_DURATION 200  // 阶段4持续时间，可根据需要0


typedef enum {
  STAGE_INIT,
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

const int stage_duration[] = {
    DURATION_STAGE_INIT,
    DURATION_STAGE_PREPARE,
    DURATION_STAGE_1,
    DURATION_STAGE_2,
    DURATION_STAGE_3,
    DURATION_STAGE_4
};

Get_Stage_t get_stage(int t) {

  int acc = 0;

for (int i = 0; i < 6; i++)
{
    acc += stage_duration[i];
    if (t < acc)
        return (Get_Stage_t)i;
}

  return STAGE_DONE;
}

typedef enum {
  GB_STAGE_PREPARE,
  GB_STAGE_1,
  GB_STAGE_2,
  GB_STAGE_3,
  GB_STAGE_4,
  GB_STAGE_5,
  GB_STAGE_6,
  GB_STAGE_DONE
} Get_Back_Stage_t;


const int get_back_duration[] = {
  GB_DURATION_PREPARE,
  GB_DURATION_1,
  GB_DURATION_2,
  GB_DURATION_3,
  GB_DURATION_4,
  GB_DURATION_5,
  GB_DURATION_6
};
Get_Back_Stage_t get_back_stage(int t)
{
  int acc = 0;

  for (int i = 0; i < 7; i++)
  {
    acc += get_back_duration[i];

    if (t < acc)
      return (Get_Back_Stage_t)i;
  }

  return GB_STAGE_DONE;
}



void Get_B()
{
  int t = traj_point_index;
  Get_Back_Stage_t stage = get_back_stage(t);

  switch(stage)
  {

  case GB_STAGE_PREPARE:
    target_point_init(Target_Point);
    Target_Point[4].target_joint_radian = 0.6f;
    Target_Point[4].velocity = 1.0f;
    break;

  case GB_STAGE_1:
    Target_Point[0].target_joint_radian = -2.27f;
    Target_Point[0].velocity = 1.5f;
    break;

  case GB_STAGE_2:
    Target_Point[1].target_joint_radian = -0.1;
    Target_Point[4].target_joint_radian = 0.0f;
    break;

  case GB_STAGE_3:
    Target_Point[1].target_joint_radian = 0.27f;
    Target_Point[2].target_joint_radian = 0.1f;
    break;

  case GB_STAGE_4:
    Target_Point[1].target_joint_radian = 0.34f;
    Target_Point[2].target_joint_radian = 0.15f;
    Target_Point[4].target_joint_radian = 0.2f;
    break;

  case GB_STAGE_5:

    Target_Point[2].target_joint_radian = 0.55f;
    break;

  case GB_STAGE_6:
    Gripper_Current_Control_Mode = GRIPPER_CLOSE_MODE;
    Target_Point[0].velocity = 1.5f;
    Target_Point[0].target_joint_radian = 0.0f;
    break;

  case GB_STAGE_DONE:
    target_point_init(Target_Point);
    break;
  }

  traj_point_index++;
}
void Get(Command_Place_And_Get_t cmd_get) {
  int direct = (cmd_get == Command_getRight) ? (1) : (-1);
  int t = traj_point_index ;
  Get_Stage_t stage = get_stage(t);

  switch (stage) {
  case STAGE_INIT:
    target_point_init(Target_Point);
    break;
  case STAGE_PREPARE:
    Target_Point[0].target_joint_radian = 0.76f * direct;
    Target_Point[0].velocity = 1.5f;
    Target_Point[2].target_joint_radian = 0.5f;
    Target_Point[2].velocity = 1.0f;
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
if (t >= (DURATION_STAGE_INIT +
          DURATION_STAGE_PREPARE +
          DURATION_STAGE_1 +
          DURATION_STAGE_2 + 50))
{
    Gripper_Current_Control_Mode = GRIPPER_CLOSE_MODE;
}

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
      // osTimerNew(Trajectory_Timer_Callback, osTimerPeriodic, NULL, &timer_attr);
      osTimerNew(Get_Back_Timer_Callback, osTimerPeriodic, NULL, &timer_attr);
}

void Trajectory_Publisher_Task(void *argument) {
  UNUSED(argument);
  osThreadFlagsWait(TRAJ_START_FLAG, osFlagsWaitAll, osWaitForever);
  osTimerStart(traj_timer_id, 5);
  for (;;) {
    osDelay(100);
  }
}
