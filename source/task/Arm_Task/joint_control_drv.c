// joint_control_drv.c: 关节控制驱动

#include "joint_control_drv.h"
#include "cmsis_os2.h"
#include "motor_DM.h"
#include "stm32h7xx_hal_def.h"
#include "tool.h"

float Ctrller_Joint_Radian[6] = {0};
DM_motor_t *Joint_Motor[JOINT_NUM];

void Point_Generator(target_point_t *Target_Point, const float *joint_radian,
                     const float *velocity) {
  for (int joint_index = 0; joint_index < JOINT_NUM; joint_index++) {
    Target_Point[joint_index].target_joint_radian = joint_radian[joint_index];
    Target_Point[joint_index].velocity = velocity[joint_index];
  }
}
/**
 * @brief 关节角度解算
 *
 * @param CtrllerData 控制器数据
 * @param joint_radian 弧度数组
 */
void Parse_ControllerData_To_CtrllerRadian(const uint8_t *CtrllerData,
                                           float *joint_radian) {
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
/**
 * @brief 电机模块初始化
 *
 * @param Joint
 */
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
/**
 * @brief 关节自由度初始化
 *
 * @param Joint
 */
void joint_dof_init(Joint_t *Joint) {
  for (int joint_index = 0; joint_index < JOINT_NUM; joint_index++) {
    Joint[joint_index].dof = joint_dof_map[joint_index];
  }
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
 * @brief 自定义控制器接收角度并转动电机
 *
 * @param Joint
 * @param input_radian
 */
void Joint_Custom_State_Motor_Ctrl(Joint_t *Joint, float *input_radian) {
  for (int joint_index = 0; joint_index < JOINT_NUM; joint_index++) {
    osDelay(1);
    Joint_Motor_PosSpeed_Ctrl(
        &Joint[joint_index],
        Joint_Pos_Limit(
            Joint_Apply_Polarity(input_radian[joint_index], joint_index),
            joint_index),
        JOINT_DEFAULT_VELOCITY);
  }
}

void Joint_Mannal_State_Motor_Ctrl(Joint_t *Joint, float *input_radian) {
  for (int joint_index = 0; joint_index < JOINT_NUM; joint_index++) {
    osDelay(1);
    Joint_Motor_PosSpeed_Ctrl(
        &Joint[joint_index],
        Joint_Pos_Limit(
            Joint_Apply_Mannal_polarity(input_radian[joint_index], joint_index),
            joint_index),
        JOINT_DEFAULT_VELOCITY);
  }
}

static inline float Radian_Input_To_Target(float input_radian,
                                           int joint_index) {
  return Joint_Pos_Limit(input_radian, joint_index);
}
/**
 * @brief 将CtrllerData 转换成输入弧度组，主要功能是调整极性
 *
 * @param joint_radian 关节数组
 */
void CtrllerData_To_InputRadian_Converter(float *joint_radian) {
  for (int joint_index = 0; joint_index < JOINT_NUM; joint_index++) {
    joint_radian[joint_index] =
        joint_radian[joint_index] * joint_custom_polarity_map[joint_index];
  }
}

// /**
//  * @brief 关节电机控制
//  *
//  * @param Joint 关节数组
//  * @param input_radian 输入弧度
//  */
// void Joint_Motor_Ctrl(Joint_t *Joint, float input_radian) {
//   for (int joint_index = 0; joint_index < JOINT_NUM; joint_index++) {
//     osDelay(1);
//     Joint_Motor_PosSpeed_Ctrl(&Joint[joint_index],
//                               Radian_Input_To_Target(input_radian,
//                               joint_index), JOINT_DEFAULT_VELOCITY);
//   }
// }

void Joint_Move_defaultyVel(Joint_t *Joint, float *target_radian) {
  for (int joint_index = 0; joint_index < JOINT_NUM; joint_index++) {
    osDelay(1);
    Joint_Motor_PosSpeed_Ctrl(
        &Joint[joint_index],
        Radian_Input_To_Target(target_radian[joint_index], joint_index),
        JOINT_DEFAULT_VELOCITY);
  }
}

void Joint_Move_byPoint(Joint_t *Joint, target_point_t *target_point) {
  for (int joint_index = 0; joint_index < JOINT_NUM; joint_index++) {
    osDelay(1);
    Joint_Motor_PosSpeed_Ctrl(&Joint[joint_index],
                              target_point[joint_index].target_joint_radian,
                              target_point[joint_index].velocity);
  }
}
