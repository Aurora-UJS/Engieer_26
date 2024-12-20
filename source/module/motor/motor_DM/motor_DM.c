#include "motor_DM.h"
#include "string.h"
static Data_Enable[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};    // 达妙电机使能命令
static Data_Failure[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};   // 电机失能命令
static Data_Save_zero[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE}; // 电机保存零点命令

void Motor_DM_Init(DM_motor_t *motor)
{
    // 初始化电机参数
    motor->motor_msg.can_msg.port = motor->can_cfg.port;
    can_msg_add_item(&motor->motor_msg.can_msg);
}

void Motor_DM_Refresh(DM_motor_t *motor)
{
    // 刷新电机状态
    motor->motor_msg.motor_angle = (motor->motor_msg.can_msg.data[1] << 8) | motor->motor_msg.can_msg.data[2];
    motor->motor_msg.motor_speed = (motor->motor_msg.can_msg.data[3] << 4) | (motor->motor_msg.can_msg.data[4] >> 4);
    motor->motor_msg.torque_current = (motor->motor_msg.can_msg.data[4] << 4) | motor->motor_msg.can_msg.data[5];
    motor->motor_msg.temp = motor->motor_msg.can_msg.data[6] > motor->motor_msg.can_msg.data[7] ? motor->motor_msg.can_msg.data[6] : motor->motor_msg.can_msg.data[7];
}

void Motor_DM_Enable(DM_motor_t *motor)
{
    // 使能电机
    memcpy(motor->can_cfg.data, Data_Enable, 8);

    motor->can_cfg.len = FDCAN_DLC_BYTES_8;

    can_msg_send_classical(&motor->can_cfg);
}

void Motor_DM_Disable(DM_motor_t *motor)
{
    // 失能电机
    memcpy(motor->can_cfg.data, Data_Failure, 8);
    motor->can_cfg.len = FDCAN_DLC_BYTES_8;

    can_msg_send_classical(&motor->can_cfg);
}   

void Motor_DM_Save_Zero(DM_motor_t *motor)
{
    // 保存零点
    memcpy(motor->can_cfg.data, Data_Save_zero, 8);
    motor->can_cfg.len = FDCAN_DLC_BYTES_8;

    can_msg_send_classical(&motor->can_cfg);
}

/**
 * @brief  达妙电机位置速度模式控下控制帧
 * @param  hcan   CAN的句柄
 * @param  ID     数据帧的ID
 * @param  _pos   位置给定
 * @param  _vel   速度给定
 */
void PosSpeed_CtrlMotorDM(DM_motor_t *motor, float _pos, float _vel)
{

    uint8_t *pbuf, *vbuf;
    pbuf = (uint8_t *)&_pos;
    vbuf = (uint8_t *)&_vel;                 

    memcpy(motor->can_cfg.data, pbuf, 4);
    memcpy(motor->can_cfg.data + 4, vbuf, 4);

    motor->can_cfg.len = FDCAN_DLC_BYTES_8;

    can_msg_send_classical(&motor->can_cfg);
}