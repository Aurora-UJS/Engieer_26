#ifndef JOINT_CONTROL_DRV
#define JOINT_CONTROL_DRV

#include "DBusSys.h"
#include "arm_math.h"
#include "can_struct.h"
#include "cmsis_os2.h"
#include "motor_DM.h"
#include "tool.h"
#include <stdio.h>
#include "referee_api.h"

#define JOINT_NUM 6
#define JOINT_DEFAULT_VELOCITY 0.5f

#define JOINT_POS_MAX 3.2f
#define JOINT_POS_MIN -3.2f
#define NEGAVTIVE (-1.0f)
#define PROSITIVE (1.0f)
#define Angle_Epsilon 0.005f

typedef enum { JOINT_DOF_ROLL = 0, JOINT_DOF_YAW, JOINT_DOF_PITCH } joint_dof_t;

static const float joint_pos_limit_max_map[JOINT_NUM] = {
    2.0f, 1.5f, 3.0f, JOINT_POS_MAX, 1.5f, 1.5f,
};
static const float joint_pos_limit_min_map[JOINT_NUM] = {
    -2.0f, 0, 0, JOINT_POS_MIN, -1.5f, -1.5f};

static const float joint_custom_polarity_map[JOINT_NUM] = {
    PROSITIVE, PROSITIVE, NEGAVTIVE, NEGAVTIVE, NEGAVTIVE, NEGAVTIVE};

static const float joint_mannal_polarity_map[JOINT_NUM] = {
    PROSITIVE, PROSITIVE, PROSITIVE, PROSITIVE, PROSITIVE, PROSITIVE};

static const can_port_t can_port_map[JOINT_NUM] = {
    CAN3_PORT, CAN3_PORT, CAN2_PORT, CAN2_PORT, CAN2_PORT, CAN2_PORT,
};
static const joint_dof_t joint_dof_map[JOINT_NUM] = {
    JOINT_DOF_YAW,  JOINT_DOF_PITCH, JOINT_DOF_PITCH,
    JOINT_DOF_ROLL, JOINT_DOF_PITCH, JOINT_DOF_ROLL};

typedef struct Joint_t {
  DM_motor_t *joint_motor;
  joint_dof_t dof;
} Joint_t;

typedef struct target_point_t {
  float target_joint_radian;
  float velocity;
} target_point_t;

extern uint8_t CtrllerData[CtrllerData_Length];
extern rc_info_t remoter;

/** @brief 关节电机常规控制（未使用） */
void Joint_Mannal_State_Motor_Ctrl(Joint_t *Joint, float *input_radian);

void Parse_ControllerData_To_CtrllerRadian(const uint8_t *CtrllerData,
                                           float *joint_radian);

void Joint_Custom_State_Motor_Ctrl(Joint_t *Joint, float *input_radian);

/**
 * @brief 关节电机位置速度模式控制
 *
 * @param Joint 关节
 * @param target_radian 目标弧度
 * @param velocity 速度
 */
static inline void
Joint_Motor_PosSpeed_Ctrl(Joint_t *Joint, float target_radian, float velocity) {
  PosSpeed_CtrlMotorDM(Joint->joint_motor, target_radian, velocity);
}

/**
 * @brief 常规极性调整
 *
 * @param input_radian
 * @param joint_index
 * @return float
 */
static inline float Joint_Apply_Mannal_polarity(float input_radian,
                                                int joint_index) {
  return joint_mannal_polarity_map[joint_index] * input_radian;
}
/**
 * @brief 自定义控制器关节电机极性调整
 *
 * @param input_radian
 * @param joint_index
 * @return float
 */
static inline float Joint_Apply_Polarity(float input_radian, int joint_index) {
  return joint_custom_polarity_map[joint_index] * input_radian;
}

/**
 * @brief 关节限位
 *
 * @param input_radian
 * @param joint_index
 * @return float
 */
static inline float Joint_Pos_Limit(float input_radian, int joint_index) {
  return limit(input_radian, joint_pos_limit_min_map[joint_index],
               joint_pos_limit_max_map[joint_index]);
}

static inline float Float_Abs(float num) { return (num >= 0.0f) ? num : -num; }

static inline float Delta(float current, float target) {
  return current - target;
}

static inline float Error_Calc(float current, float target) {
  return Float_Abs(Delta(current, target));
}

// float at_angle_test;
static inline bool Joint_At_Target(Joint_t *Joint, float target_radian,
                                   float epsilon) {
  // at_angle_test = Error_Calc(Joint->joint_motor->motor_msg.motor_angle,
  // target_radian);
  if (Error_Calc(Joint->joint_motor->motor_msg.motor_angle, target_radian) <=
      epsilon) {
    return true;
  }
  return false;
}

static inline bool Arm_At_Target(Joint_t *Joint,
                                 const float *transition_radian) {
  for (int joint_index = 1; joint_index < JOINT_NUM; joint_index++) {

    if (false == Joint_At_Target(&Joint[joint_index],
                                 transition_radian[joint_index],
                                 Angle_Epsilon)) {
      return false;
    }
  }
  return true;
}

/** @brief 发布点数据 */
void Point_Publisher(target_point_t *Target_Point, const float *joint_radian,
                     const float *velocity);

/** @brief 关节根据点移动函数 */
void Joint_Move_byPoint(Joint_t *Joint, target_point_t *target_point);

/** @brief 关节数据转换输入弧度 */
void CtrllerData_To_InputRadian_Converter(float *joint_radian);

/** @brief 关节默认速度移动（未使用） */
void Joint_Move_defaultyVel(Joint_t *Joint, float *target_radian);

/** @brief 关节初始化*/
void joint_init(Joint_t *Joint);

/** @brief 关节自由度信息初始化 */
void joint_dof_init(Joint_t *Joint);

/** @brief 电机初始化 */
void joint_motor_init(Joint_t *Joint);

/** @brief 电机信息更新 */
void Joint_Motor_Refresh(Joint_t *Joint);

/** @brief 电机使能 */
void Joint_Motor_Enable(Joint_t *Joint);
#endif
