
#include "DbusSys.h"
#include "motor_DJI.h"
#include "stm32h7xx_hal_fdcan.h"
#include "can_api.h"

void set_motor_voltage(DJI_motor_t *msg,int16_t v1,int16_t v2,int16_t v3,int16_t v4)
{
	msg->can_cfg.len=FDCAN_DLC_BYTES_8;
	msg->can_cfg.data[0]=v1>>8;
	msg->can_cfg.data[1]=v1;
	msg->can_cfg.data[2]=v2>>8;
	msg->can_cfg.data[3]=v2;
	msg->can_cfg.data[4]=v3>>8;
	msg->can_cfg.data[5]=v3;
	msg->can_cfg.data[6]=v4>>8;
	msg->can_cfg.data[7]=v4;                	 //完善数据帧
	
	can_msg_send_classical(&msg->can_cfg);
	
}

void Motor_DJI_Init(DJI_motor_t *motor)
{
    // 初始化电机参数
    motor->motor_msg[0].can_msg.port = motor->can_cfg.port;
    can_msg_add_item(&motor->motor_msg[0].can_msg);
	motor->motor_msg[1].can_msg.port = motor->can_cfg.port;
    can_msg_add_item(&motor->motor_msg[1].can_msg);
	motor->motor_msg[2].can_msg.port = motor->can_cfg.port;
	can_msg_add_item(&motor->motor_msg[2].can_msg);
	motor->motor_msg[3].can_msg.port = motor->can_cfg.port;
	can_msg_add_item(&motor->motor_msg[3].can_msg);
}

void Motor_DJI_Refresh(DJI_motor_t *motor)
{

    motor->motor_msg[0].motor_angle = (motor->motor_msg[0].can_msg.data[0] << 8) | motor->motor_msg[0].can_msg.data[1];
    motor->motor_msg[0].motor_speed = (motor->motor_msg[0].can_msg.data[2] << 8) | (motor->motor_msg[0].can_msg.data[3]);
    motor->motor_msg[0].torque_current = (motor->motor_msg[0].can_msg.data[4] << 8) | motor->motor_msg[0].can_msg.data[5];
    motor->motor_msg[0].temp = motor->motor_msg[0].can_msg.data[6]; 

	motor->motor_msg[1].motor_angle = (motor->motor_msg[1].can_msg.data[0] << 8) | motor->motor_msg[1].can_msg.data[1];
    motor->motor_msg[1].motor_speed = (motor->motor_msg[1].can_msg.data[2] << 8) | (motor->motor_msg[1].can_msg.data[3]);
    motor->motor_msg[1].torque_current = (motor->motor_msg[1].can_msg.data[4] << 8) | motor->motor_msg[1].can_msg.data[5];
    motor->motor_msg[1].temp = motor->motor_msg[1].can_msg.data[6]; 

	motor->motor_msg[2].motor_angle = (motor->motor_msg[2].can_msg.data[0] << 8) | motor->motor_msg[2].can_msg.data[1];
    motor->motor_msg[2].motor_speed = (motor->motor_msg[2].can_msg.data[2] << 8) | (motor->motor_msg[2].can_msg.data[3]);
    motor->motor_msg[2].torque_current = (motor->motor_msg[2].can_msg.data[4] << 8) | motor->motor_msg[2].can_msg.data[5];
    motor->motor_msg[2].temp = motor->motor_msg[2].can_msg.data[6]; 

	motor->motor_msg[3].motor_angle = (motor->motor_msg[3].can_msg.data[0] << 8) | motor->motor_msg[3].can_msg.data[1];
    motor->motor_msg[3].motor_speed = (motor->motor_msg[3].can_msg.data[2] << 8) | (motor->motor_msg[3].can_msg.data[3]);
    motor->motor_msg[3].torque_current = (motor->motor_msg[3].can_msg.data[4] << 8) | motor->motor_msg[3].can_msg.data[5];
    motor->motor_msg[3].temp = motor->motor_msg[3].can_msg.data[6]; 
}