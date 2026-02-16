#include "ee_control_drv.h"

/**
 * @brief 末端执行器信息更新
 *
 * @param endeffector 末端执行器
 */

void EndEffector_Motor_Refresh(endEffector_t *endeffector) {
  Motor_DM_Refresh(endeffector->endEffector_motor);
}

/**
 * @brief 末端执行器电机使能模块
 *
 * @param endeffector 末端执行器
 */
void EndEffector_Motor_Enable(endEffector_t *endeffector) {
  Motor_DM_Enable(endeffector->endEffector_motor);
}

/**
 * @brief 末端执行器参数
 *
 * @param endeffector
 */
void endEffector_init(endEffector_t *endeffector) {
  endEffector_motor_init(endeffector);
}
void Gripper_Open(endEffector_t *endeffector) {
  osDelay(1);
  PosSpeed_CtrlMotorDM(endeffector->endEffector_motor, GRIPPER_OPEN_RADIAN,
                       GRIPPER_VEL);
}
void Gripper_Close(endEffector_t *endEffector) {
  osDelay(1);
  PosSpeed_CtrlMotorDM(endEffector->endEffector_motor, GRIPPER_CLOSE_RADION,
                       GRIPPER_VEL);
}
/**
 * @brief 末端执行器初始化
 *
 * @param endeffector
 */
void endEffector_motor_init(endEffector_t *endeffector) {
  endeffector->endEffector_motor = pvPortMalloc(sizeof(DM_motor_t));
  endeffector->endEffector_motor->can_cfg.id = 0x07;
  endeffector->endEffector_motor->motor_msg.can_msg.id = 0x07;
  endeffector->endEffector_motor->can_cfg.port = CAN2_PORT;

  Motor_DM_Init(endeffector->endEffector_motor);
}
