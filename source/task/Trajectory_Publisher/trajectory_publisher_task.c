#include "trajectory_publisher_task.h"
#include "Trajectory_Data.h"
#include "arm_state_machine.h"
#include "cmsis_os2.h"
#include "ee_control_drv.h"
#include "jointFollowAngle.h"
#include "joint_control_drv.h"
#include "stm32h7xx_hal_def.h"

static osTimerId_t traj_timer_id;
extern target_point_t Target_Point[JOINT_NUM];
extern arm_control_mode_t Arm_Current_Control_Mode;
extern gripper_control_mode_t Gripper_Current_Control_Mode;
static int traj_point_index = 0;

extern const float Trajectory[TRAJECTORY_NUM][JOINT_NUM];

extern const float Traj_Vel[TRAJECTORY_NUM][JOINT_NUM];

#define ARM_PREPARE_TIME_TICKS (500)
#define ARM_PICK_TIME_TICKS (TRAJECTORY_NUM)
#define ARM_RETURN_TIME_TICKS (1000)
#define ARM_TOTOAL_TRAJECTORY_TIME_TICKS                                       \
  (ARM_PREPARE_TIME_TICKS + ARM_PICK_TIME_TICKS + ARM_RETURN_TIME_TICKS)

#define ENDEFFECTOR_CLOSE_TIME_TICKS (1550)
#define ENDEFFECTOR_TOTOAL_TIME_TICKS                                          \
  (ARM_PREPARE_TIME_TICKS + ENDEFFECTOR_CLOSE_TIME_TICKS)

void Trajectory_Publisher_right(const float Trajectory[][JOINT_NUM],
                                int point_index) {
  for (int joint_index = 0; joint_index < JOINT_NUM; joint_index++) {
    if (joint_index == 0) {
      Target_Point[joint_index].target_joint_radian =
          (-1.0f) * Trajectory[point_index][joint_index];
    } else {
      if (point_index >= 1500 && joint_index == 4) {
        Target_Point[joint_index].target_joint_radian =
            Trajectory[point_index][4] + 0.2f;
      } else {
        Target_Point[joint_index].target_joint_radian =
            Trajectory[point_index][joint_index];
      }
    }
    Target_Point[joint_index].velocity =
        Traj_Vel[point_index][joint_index] * (-2.0f);
  }
}
void Trajectory_Publisher(const float Trajectory[][JOINT_NUM],
                          int point_index) {
  for (int joint_index = 0; joint_index < JOINT_NUM; joint_index++) {
    Target_Point[joint_index].target_joint_radian =
        Trajectory[point_index][joint_index];
    Target_Point[joint_index].velocity =
        Traj_Vel[point_index][joint_index] * 2.0f;
  }
}
void getLeft(void) {

  if (traj_point_index <= ARM_PREPARE_TIME_TICKS) {
    Target_Point[0].target_joint_radian = -0.79f;
    Target_Point[0].velocity = 0.5f;
    Target_Point[4].target_joint_radian = 0.0f;
    Target_Point[4].velocity = 0.5f;
  } else if (traj_point_index >= 2742) {
    target_point_init(Target_Point);
  } else {
    Trajectory_Publisher(Trajectory, traj_point_index - ARM_PREPARE_TIME_TICKS);
  }
  if (traj_point_index == ENDEFFECTOR_CLOSE_TIME_TICKS) {
    Gripper_Current_Control_Mode = GRIPPER_CLOSE_MODE;
  }
  traj_point_index++;
}
void getRight(void) {
  if (traj_point_index <= ARM_PREPARE_TIME_TICKS) {
    Target_Point[0].target_joint_radian = 0.79f;
    Target_Point[0].velocity = 0.5f;
    Target_Point[4].target_joint_radian = 0.0f;
    Target_Point[4].velocity = 0.5f;
  } else if (traj_point_index >= (ARM_PREPARE_TIME_TICKS+ARM_PICK_TIME_TICKS) ) {
    target_point_init(Target_Point);
  } else {
    Trajectory_Publisher_right(Trajectory, traj_point_index - ARM_PREPARE_TIME_TICKS);
  }
  if (traj_point_index == ENDEFFECTOR_CLOSE_TIME_TICKS) {
    Gripper_Current_Control_Mode = GRIPPER_CLOSE_MODE;
  }
  traj_point_index++;
}
void Debug_set_pos(void){
  Target_Point[0].target_joint_radian = -0.79;
  Target_Point[1].target_joint_radian = 0.46139;
  // Target_Point[2].target_joint_radian = 0.2790;
  Target_Point[2].target_joint_radian = 0.40;
  Target_Point[3].target_joint_radian = 0.0;
  Target_Point[4].target_joint_radian = 0.004;
  Target_Point[5].target_joint_radian = 0.000;
  Target_Point[0].velocity = 0.2;
  for (int joint_index= 1; joint_index < JOINT_NUM; joint_index++) {
    Target_Point[joint_index].velocity = JOINT_DEFAULT_VELOCITY;
  }
}
void placeLeft(void){
// if (traj_point_index >= 500 && traj_point_index <= 2500) {
//   Gripper_Current_Control_Mode = GRIPPER_CLOSE_MODE;
// }
// else {
//   Gripper_Current_Control_Mode = GRIPPER_OPEN_MODE;
// }

if (traj_point_index <=2500) {
  Gripper_Current_Control_Mode = GRIPPER_CLOSE_MODE;
}
else {
  Gripper_Current_Control_Mode = GRIPPER_OPEN_MODE;
}
if (traj_point_index >=1000 && traj_point_index <=2000) {
  Target_Point[0].target_joint_radian = -0.79;
  Target_Point[1].target_joint_radian = 0.46139;
  // Target_Point[2].target_joint_radian = 0.2790;
  Target_Point[2].target_joint_radian = 0.45;
  Target_Point[3].target_joint_radian = 0.0;
  Target_Point[4].target_joint_radian = 0.2;
  Target_Point[5].target_joint_radian = 0.00;
  Target_Point[0].velocity = 0.2;
  for (int joint_index= 1; joint_index < JOINT_NUM; joint_index++) {
    Target_Point[joint_index].velocity = JOINT_DEFAULT_VELOCITY;
  }
}
else if (traj_point_index > 2000 && traj_point_index <2800) {
  Target_Point[2].velocity = 0.2;
  Target_Point[2].target_joint_radian = 0.26;
}
else if (traj_point_index >= 2800 && traj_point_index <=3300) {

  Target_Point[1].velocity = 0.2;
  Target_Point[1].target_joint_radian = 0;
}
else if (traj_point_index > 3300) {
  Target_Point[0].target_joint_radian = 0;
  Target_Point[0].velocity = 0.3;
}
traj_point_index ++;
}

void placeRight(void){
if (traj_point_index <=2500) {
  Gripper_Current_Control_Mode = GRIPPER_CLOSE_MODE;
}
else {
  Gripper_Current_Control_Mode = GRIPPER_OPEN_MODE;
}
// if (traj_point_index >= 500 && traj_point_index <= 2500) {
//   Gripper_Current_Control_Mode = GRIPPER_CLOSE_MODE;
// }
// else {
//   Gripper_Current_Control_Mode = GRIPPER_OPEN_MODE;
// }
if (traj_point_index >=1000 && traj_point_index <=2000) {
  Target_Point[0].target_joint_radian = 0.79;
  Target_Point[1].target_joint_radian = 0.46139;
  // Target_Point[2].target_joint_radian = 0.2790;
  Target_Point[2].target_joint_radian = 0.45;
  Target_Point[3].target_joint_radian = 0.0;
  Target_Point[4].target_joint_radian = 0.2;
  Target_Point[5].target_joint_radian = 0.00;
  Target_Point[0].velocity = 0.2;
  for (int joint_index= 1; joint_index < JOINT_NUM; joint_index++) {
    Target_Point[joint_index].velocity = JOINT_DEFAULT_VELOCITY;
  }
}
else if (traj_point_index > 2000 && traj_point_index <2800) {
  Target_Point[2].velocity = 0.2;
  Target_Point[2].target_joint_radian = 0.26;
}
else if (traj_point_index >= 2800 && traj_point_index <=3300) {

  Target_Point[1].velocity = 0.2;
  Target_Point[1].target_joint_radian = 0;
}
else if (traj_point_index > 3300) {
  Target_Point[0].target_joint_radian = 0;
  Target_Point[0].velocity = 0.3;
}
traj_point_index ++;
}
void Trajectory_Timer_Callback(void *argument) {
  UNUSED(argument);
  // placeRight();
  getLeft();
  // getLeft();
  // Debug_set_pos();
  // placeRight();
  // if (traj_point_index >= (ARM_TOTOAL_TRAJECTORY_TIME_TICKS)) {
  if (traj_point_index >= 5000) {
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
