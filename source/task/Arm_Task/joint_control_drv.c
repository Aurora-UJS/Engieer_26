// joint_control_drv.c: 关节控制驱动

#include "joint_control_drv.h"
#include "cmsis_os2.h"
#include "ee_control_drv.h"
#include "motor_DM.h"

float Ctrller_Joint_Radian[6] = {0};
DM_motor_t *Joint_Motor[JOINT_NUM];

void Point_Publisher(target_point_t *Target_Point, const float *joint_radian,
                     const float *velocity) {
  for (int joint_index = 0; joint_index < JOINT_NUM; joint_index++) {
    Target_Point[joint_index].target_joint_radian = joint_radian[joint_index];
    Target_Point[joint_index].velocity = velocity[joint_index];
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

  // 关节电机初始化
  joint_motor_init(Joint);
}


void Joint_Move(Joint_t Joint[],target_point_t Target_Point[]){
  
    Joint_Motor_PosSpeed_Ctrl(&Joint[1],Target_Point[1]);

    Joint_Motor_PosSpeed_Ctrl(&Joint[4],Target_Point[4]);

    osDelay(1);
    Joint_Motor_PosSpeed_Ctrl(&Joint[2],Target_Point[2]);

    Joint_Motor_PosSpeed_Ctrl(&Joint[0],Target_Point[0]);

    osDelay(1);
    Joint_Motor_PosSpeed_Ctrl(&Joint[3],Target_Point[3]);

    osDelay(1);
    Joint_Motor_PosSpeed_Ctrl(&Joint[5],Target_Point[5]);
    osDelay(1);
}

__attribute__((deprecated))
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
