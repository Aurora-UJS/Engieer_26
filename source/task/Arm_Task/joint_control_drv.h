#ifndef JOINT_CONTROL_DRV
#define JOINT_CONTROL_DRV

#include "DBusSys.h"
#include "arm_math.h"
#include "can_struct.h"
#include "cmsis_os2.h"
#include "motor_DM.h"
#include "tool.h"
#include <stdio.h>

#define JOINT_NUM 6
#define JOINT_DEFAULT_VELOCITY 0.5f

#define JOINT_POS_MAX 3.2f
#define JOINT_POS_MIN -3.2f
#define  NEGAVTIVE  (-1.0f)
#define  PROSITIVE  (1.0f)
typedef enum { JOINT_DOF_ROLL = 0, JOINT_DOF_YAW, JOINT_DOF_PITCH } joint_dof_t;

static const float joint_pos_limit_max_map[JOINT_NUM] = {
    2.0f, 1.5f, 3.0f, JOINT_POS_MAX, 1.5f, JOINT_POS_MAX,
};
static const float joint_pos_limit_min_map[JOINT_NUM] = {
    -2.0f, 0, 0, JOINT_POS_MIN, -1.5f, JOINT_POS_MIN};
static const float joint_polarity_map[JOINT_NUM] = {
    PROSITIVE, PROSITIVE, NEGAVTIVE, NEGAVTIVE, NEGAVTIVE, NEGAVTIVE};
static const can_port_t can_port_map[JOINT_NUM] = {
    CAN3_PORT, CAN2_PORT, CAN2_PORT, CAN2_PORT, CAN2_PORT, CAN2_PORT,
};
static const joint_dof_t joint_dof_map[JOINT_NUM] = {
    JOINT_DOF_YAW,  JOINT_DOF_PITCH, JOINT_DOF_PITCH,
    JOINT_DOF_ROLL, JOINT_DOF_PITCH, JOINT_DOF_ROLL};

typedef struct Joint_t {
  DM_motor_t *joint_motor;
  joint_dof_t dof;
  float target_radian;
  float current_radian;
} Joint_t;

extern uint8_t CtrllerData[24];
extern rc_info_t remoter;

void Parse_ControllerData_To_JointRadian(const uint8_t *CtrllerData,
                                         float *joint_radian);
void Joint_Motor_PosSpeed_Ctrl(Joint_t *Joint, float target_radian,
                               float velocity);

void Joint_Custom_State_Motor_Ctrl(Joint_t *Joint, const float *input_radian);

/** @brief 关节初始化*/
void joint_init(Joint_t *Joint);

/** @brief 关节自由度信息初始化 */
void joint_dof_init(Joint_t *Joint);

/** @brief 电机初始化*/
void joint_motor_init(Joint_t *Joint);

/** @brief 电机信息更新 */
void Joint_Motor_Refresh(Joint_t *Joint);

/** @brief 电机使能 */
void Joint_Motor_Enable(Joint_t *Joint);
#endif
