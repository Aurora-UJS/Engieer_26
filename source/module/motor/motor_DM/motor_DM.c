#include "motor_DM.h"
#include "string.h"
static uint8_t Data_Enable[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};    // 达妙电机使能命令
static uint8_t Data_Failure[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};   // 电机失能命令
static uint8_t Data_Save_zero[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE}; // 电机保存零点命令

/**
************************************************************************
* @brief:      	float_to_uint: 浮点数转换为无符号整数函数
* @param[in]:   x_float:	待转换的浮点数
* @param[in]:   x_min:		范围最小值
* @param[in]:   x_max:		范围最大值
* @param[in]:   bits: 		目标无符号整数的位数
* @retval:     	无符号整数结果
* @details:    	将给定的浮点数 x 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个指定位数的无符号整数
************************************************************************
**/
int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
    /* Converts a float to an unsigned int, given range and number of bits */
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x_float - offset) * ((float)((1 << bits) - 1)) / span);
}

float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	/* converts unsigned int to float, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}
void Motor_DM_Init(DM_motor_t *motor)
{
    // 初始化电机参数
    motor->motor_msg.can_msg.port = motor->can_cfg.port;
    motor->tmp.PMAX		= 12.5f;
	motor->tmp.VMAX		= 30.0f;
	motor->tmp.TMAX		= 10.0f;
    can_msg_add_item(&motor->motor_msg.can_msg);
}

void Motor_DM_Refresh(DM_motor_t *motor)
{
    //开始解算
    int16_t motor_angle, motor_speed, torque_current;

    motor_angle = (motor->motor_msg.can_msg.data[1] << 8) | motor->motor_msg.can_msg.data[2];
    motor_speed = (motor->motor_msg.can_msg.data[3] << 4) | (motor->motor_msg.can_msg.data[4] >> 4);
    torque_current = (motor->motor_msg.can_msg.data[4] << 4) | motor->motor_msg.can_msg.data[5];

    // 刷新电机状态
    motor->motor_msg.temp = motor->motor_msg.can_msg.data[6] > motor->motor_msg.can_msg.data[7] ? motor->motor_msg.can_msg.data[6] : motor->motor_msg.can_msg.data[7];
    motor->motor_msg.motor_angle = uint_to_float(motor_angle, -motor->tmp.PMAX, motor->tmp.PMAX, 16);    // (-12.5,12.5)
    motor->motor_msg.motor_speed = uint_to_float(motor_speed , -motor->tmp.VMAX, motor->tmp.VMAX, 12);    // (-45.0,45.0)
    motor->motor_msg.torque_current = uint_to_float(torque_current, -motor->tmp.TMAX, motor->tmp.TMAX, 12); // (-18.0,18.0)
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
* @param[in]:   motor_id:	电机ID，指定目标电机
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
