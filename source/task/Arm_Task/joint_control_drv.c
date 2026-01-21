// joint_control_drv.c: 关节控制驱动

#include "joint_control_drv.h"
#include "motor_DM.h"
#include "stm32h7xx_hal_def.h"
#include "tool.h"

float joint_radian[6] = {0};

DM_motor_t *Joint_Motor[JOINT_NUM];

void Parse_ControllerData_To_JointRadian(const uint8_t *CtrllerData,
                                         float *joint_radian){
  for (int i = 0; i < 6; i++) {
    int tmp = 0;
    // 每个关节弧度占 4 个字符
    sscanf((const char *)&CtrllerData[i * 4], "%04d", &tmp);
    joint_radian[i] = tmp / 1000.0f;
    joint_radian[i] -= PI; // 偏移 PI
  }
}
/**
 * @brief 关节电机信息更新
 *
 * @param Joint 关节
 */
void Joint_Motor_Refresh(Joint_t *Joint) {
  for (int joint_index = 0; joint_index < JOINT_NUM; joint_index++) {
    Motor_DM_Refresh(Joint[joint_index].joint_motor);
  }
}

/**
 * @brief 关节电机使能模块
 *
 * @param Joint 关节
 */
void Joint_Motor_Enable(Joint_t *Joint) {
  for (int joint_index = 0; joint_index < JOINT_NUM; joint_index++) {
    Motor_DM_Enable(Joint[joint_index].joint_motor);
  }
}
void joint_motor_init(Joint_t *Joint) {
  for (int joint_index = 0; joint_index < JOINT_NUM; joint_index++) {

    Joint[joint_index].joint_motor = pvPortMalloc(sizeof(DM_motor_t));
    // Slave id 初始化
    Joint[joint_index].joint_motor->can_cfg.id = 0x01 + joint_index;
    // Master id 初始化
    Joint[joint_index].joint_motor->motor_msg.can_msg.id = 0x11 + joint_index;

    // PMAX,VMAX,TMAX 初始化
    Joint[joint_index].joint_motor->tmp.PMAX = 12.5f;
    Joint[joint_index].joint_motor->tmp.VMAX = 3.0f;
    Joint[joint_index].joint_motor->tmp.TMAX = 1.0f;

    // CAN Port初始化
    Joint[joint_index].joint_motor->can_cfg.port = can_port_map[joint_index];
  }

  // 初始化电机
  for (int joint_index = 0; joint_index < JOINT_NUM; joint_index++) {
    Motor_DM_Init(Joint[joint_index].joint_motor);
  }
}
void joint_dof_init(Joint_t *Joint) {
  for (int joint_index = 0; joint_index < JOINT_NUM; joint_index++) {
    Joint[joint_index].dof = joint_dof_map[joint_index];
  }
}
void Joint_Motor_PosSpeed_Ctrl(Joint_t *Joint, float target_radian,
                               float velocity) {
  PosSpeed_CtrlMotorDM(Joint->joint_motor, target_radian, velocity);
}

/**
 * @brief 关节电机极性调整
 * 
 * @param input_radian 
 * @param joint_index 
 * @return float 
 */
static inline float Joint_Apply_Polarity(float input_radian, int joint_index) {
  return joint_polarity_map[joint_index] * input_radian;
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

/**
 * @brief 关节初始化设置
 * 
 * @param Joint 
 */
void joint_init(Joint_t *Joint) {

  // Joint 自由度类型初始化
  joint_dof_init(Joint);

  // 关节电机初始化
  joint_motor_init(Joint);
}

/**
* @brief 接收角度并转动电机
* 
* @param Joint 
* @param input_radian 
*/
void Joint_Custom_State_Motor_Ctrl(Joint_t *Joint, const float *input_radian) {
  for (int joint_index = 0; joint_index < JOINT_NUM; joint_index++) {
    PosSpeed_CtrlMotorDM(
        Joint[joint_index].joint_motor,
        Joint_Pos_Limit(Joint_Apply_Polarity(input_radian[joint_index], joint_index),
                        joint_index),
        JOINT_DEFAULT_VELOCITY);
  }
}
