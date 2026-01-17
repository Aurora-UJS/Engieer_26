#include "motor_DM.h"
#include "string.h"
#include "uint_float_convert.h"
static uint8_t Data_Enable[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};    // 达妙电机使能命令
static uint8_t Data_Failure[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};   // 电机失能命令
static uint8_t Data_Save_zero[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE}; // 电机保存零点命令
static uint8_t Data_Clear_Error[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB}; // 电机清除错误命令



/**
************************************************************************
* @brief:      	Motor_DM_Init: 电机初始化函数
* @param[in]:   motor: 		指向DM_motor_t结构体的指针，用于指定电机
* @retval:     	void
* @details:    	初始化电机参数
************************************************************************
**/
void Motor_DM_Init(DM_motor_t *motor)
{
    // 初始化电机参数
    motor->motor_msg.can_msg.port = motor->can_cfg.port;
    can_msg_add_item(&motor->motor_msg.can_msg);
}

/**
************************************************************************
* @brief:      	Motor_DM_Refresh: 电机刷新函数
* @param[in]:   motor: 		指向DM_motor_t结构体的指针，用于指定电机
* @retval:     	void
* @details:    	通过CAN总线接收电机反馈数据
************************************************************************
**/
void Motor_DM_Refresh(DM_motor_t *motor)
{
    //开始解算
    uint16_t motor_angle, motor_speed, torque_current;

    motor_angle = (motor->motor_msg.can_msg.data[1] << 8) | motor->motor_msg.can_msg.data[2];
    motor_speed = (motor->motor_msg.can_msg.data[3] << 4) | (motor->motor_msg.can_msg.data[4] >> 4);
    torque_current = ((motor->motor_msg.can_msg.data[4]&0xF) << 8) | motor->motor_msg.can_msg.data[5];

    // 刷新电机状态
    motor->error_code = (motor->motor_msg.can_msg.data[0]>>4 & 0x0F);
    motor->motor_msg.temp = motor->motor_msg.can_msg.data[6] > motor->motor_msg.can_msg.data[7] ? motor->motor_msg.can_msg.data[6] : motor->motor_msg.can_msg.data[7];
    motor->motor_msg.motor_angle = uint_to_float(motor_angle, -motor->tmp.PMAX, motor->tmp.PMAX, 16);    // (-12.5,12.5)
    motor->motor_msg.motor_speed = uint_to_float(motor_speed , -motor->tmp.VMAX, motor->tmp.VMAX, 12);    // (-45.0,45.0)
    motor->motor_msg.torque_current = uint_to_float(torque_current, -motor->tmp.TMAX, motor->tmp.TMAX, 12); // (-18.0,18.0)
}

/**
************************************************************************
* @brief:      	Motor_DM_Enable: 电机使能函数
* @param[in]:   motor: 		指向DM_motor_t结构体的指针，用于指定电机
* @retval:     	void
* @details:    	通过CAN总线向电机发送使能命令
************************************************************************
**/
void Motor_DM_Enable(DM_motor_t *motor)
{
    // 使能电机
    memcpy(motor->can_cfg.data, Data_Enable, 8);

    motor->can_cfg.len = FDCAN_DLC_BYTES_8;

    can_msg_send_classical(&motor->can_cfg);
}

/**
************************************************************************
* @brief:      	Motor_DM_Disable: 电机失能函数
* @param[in]:   motor: 		指向DM_motor_t结构体的指针，用于指定电机
* @retval:     	void
* @details:    	通过CAN总线向电机发送失能命令
************************************************************************
**/
void Motor_DM_Disable(DM_motor_t *motor)
{
    // 失能电机
    memcpy(motor->can_cfg.data, Data_Failure, 8);
    motor->can_cfg.len = FDCAN_DLC_BYTES_8;

    can_msg_send_classical(&motor->can_cfg);
}

/**
************************************************************************
* @brief:      	Motor_DM_Save_Zero: 电机保存零点函数
* @param[in]:   motor: 		指向DM_motor_t结构体的指针，用于指定电机
* @retval:     	void
* @details:    	通过CAN总线向电机发送保存零点命令
************************************************************************
**/
void Motor_DM_Save_Zero(DM_motor_t *motor)
{
    // 保存零点
    memcpy(motor->can_cfg.data, Data_Save_zero, 8);
    motor->can_cfg.len = FDCAN_DLC_BYTES_8;

    can_msg_send_classical(&motor->can_cfg);
}

/**
************************************************************************
* @brief:      	Motor_DM_Clear_Error: 电机清除错误函数
* @param[in]:   motor: 		指向DM_motor_t结构体的指针，用于指定电机
* @retval:     	void
* @details:    	通过CAN总线向电机发送清除错误命令
************************************************************************
**/
void Motor_DM_Clear_Error(DM_motor_t *motor)
{
    // 清除错误
    memcpy(motor->can_cfg.data, Data_Clear_Error, 8);
    motor->can_cfg.len = FDCAN_DLC_BYTES_8;

    can_msg_send_classical(&motor->can_cfg);
}
/**
************************************************************************
* @brief:      	speed_ctrl: 速度控制函数
* @param[in]:   hcan: 		指向CAN_HandleTypeDef结构的指针，用于指定CAN总线
* @param[in]:   motor_id: 电机ID，指定目标电机
* @param[in]:   vel: 			速度给定值
* @retval:     	void
* @details:    	通过CAN总线向电机发送速度控制命令
************************************************************************
**/
void Speed_CtrlMotorDM(DM_motor_t *motor, float vel)
{
    motor->can_cfg.id += SPD_MODE;
    uint8_t *vbuf;
    vbuf = (uint8_t *)&vel;
    motor->can_cfg.id = motor->can_cfg.id;

    memcpy(motor->can_cfg.data, vbuf, 4);
    motor->can_cfg.len = FDCAN_DLC_BYTES_4;

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
    motor->can_cfg.id += POS_MODE;
    uint8_t *pbuf, *vbuf;
    pbuf = (uint8_t *)&_pos;
    vbuf = (uint8_t *)&_vel;

    memcpy(motor->can_cfg.data, pbuf, 4);
    memcpy(motor->can_cfg.data + 4, vbuf, 4);
    motor->can_cfg.len = FDCAN_DLC_BYTES_8;

    can_msg_send_classical(&motor->can_cfg);
}

/**
************************************************************************
* @brief:      	mit_ctrl: MIT模式下的电机控制函数
* @param[in]:   hcan:			指向CAN_HandleTypeDef结构的指针，用于指定CAN总线
* @param[in]:   motor_id:	    电机ID，指定目标电机
* @param[in]:   pos:			位置给定值
* @param[in]:   vel:			速度给定值
* @param[in]:   kp:				位置比例系数
* @param[in]:   kd:				位置微分系数
* @param[in]:   torq:			转矩给定值
* @retval:     	void
* @details:    	通过CAN总线向电机发送MIT模式下的控制帧。
************************************************************************
**/
void MIT_CtrlMotorDM(DM_motor_t *motor, float pos, float vel, float kp, float kd, float tor)
{
    motor->can_cfg.id += MIT_MODE;
    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;

    pos_tmp = float_to_uint(pos, -motor->tmp.PMAX, motor->tmp.PMAX, 16);
    vel_tmp = float_to_uint(vel, -motor->tmp.VMAX, motor->tmp.VMAX, 12);
    tor_tmp = float_to_uint(tor, -motor->tmp.TMAX, motor->tmp.TMAX, 12);
    kp_tmp = float_to_uint(kp, KP_MIN, KP_MAX, 12);
    kd_tmp = float_to_uint(kd, KD_MIN, KD_MAX, 12);

    motor->can_cfg.data[0] = (pos_tmp >> 8);
    motor->can_cfg.data[1] = pos_tmp;
    motor->can_cfg.data[2] = (vel_tmp >> 4);
    motor->can_cfg.data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
    motor->can_cfg.data[4] = kp_tmp;
    motor->can_cfg.data[5] = (kd_tmp >> 4);
    motor->can_cfg.data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
    motor->can_cfg.data[7] = tor_tmp;

    motor->can_cfg.len = FDCAN_DLC_BYTES_8;

    can_msg_send_classical(&motor->can_cfg);
}
