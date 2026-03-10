#ifndef TRAJECTORY_PUBLISHER_DRV_H
#define TRAJECTORY_PUBLISHER_DRV_H

#include "Trajectory_Data.h"
#include "arm_state_machine.h"
#include "cmsis_os2.h"
#include "ee_control_drv.h"
#include "jointFollowAngle.h"
#include "joint_control_drv.h"
#include "stm32h7xx_hal_def.h"

#define TRAJECTORY_LEN (3000)
#define TRAJ_START_FLAG (1U << 0)

#define INIT_STAGE_TICKS (200)
#define ARM_PREPARE_TIME_TICKS (500)
#define ARM_PICK_TIME_TICKS (TRAJECTORY_NUM)
#define ARM_RETURN_TIME_TICKS (1000)
#define ARM_TOTOAL_TRAJECTORY_TIME_TICKS                                       \
  (ARM_PREPARE_TIME_TICKS + ARM_PICK_TIME_TICKS + ARM_RETURN_TIME_TICKS)

#define ENDEFFECTOR_CLOSE_TIME_TICKS (1550)
#define ENDEFFECTOR_TOTOAL_TIME_TICKS                                          \
  (INIT_STAGE_TICKS + ARM_PREPARE_TIME_TICKS + ENDEFFECTOR_CLOSE_TIME_TICKS)

  
typedef enum Command_Place_And_Get {
  Command_placeRight,
  Command_placeLeft,
  Command_getRight,
  Command_getLeft,
  Command_getBackLeft,
  Command_getBackRight,
  Command_placeBackLeft,
  Command_placeBackRight,
  Command_Error
} Command_Place_And_Get_t;

/** @brief 左取矿轨迹发布*/
void Trajectory_Publisher_right(const float Trajectory[][JOINT_NUM],
                                int point_index);

/** @brief 右取矿轨迹发布 */
void Trajectory_Publisher(const float Trajectory[][JOINT_NUM], int point_index);

void Get(Command_Place_And_Get_t cmd_get);

void newPlaceRigth(void);

void newPlaceLeft(void);

void Get_B(void);

void place_back(void);

extern Command_Place_And_Get_t cmd_place_get;
extern const float Trajectory[TRAJECTORY_NUM][JOINT_NUM];
extern const float Traj_Vel[TRAJECTORY_NUM][JOINT_NUM];
extern target_point_t Target_Point[JOINT_NUM];
extern int traj_point_index;
#endif
