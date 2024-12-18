
#include "DbusSys.h"
#include "motor_DJI.h"
#include "stm32h7xx_hal_fdcan.h"
#include "can_api.h"

void set_motor_voltage(DJI_motor_t *msg,uint8_t Port,uint16_t ID,int16_t v1,int16_t v2,int16_t v3,int16_t v4)
{
		
	msg->can_cfg.port=Port;
    msg->can_cfg.id=ID;
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
    motor->motor_msg.can_msg.port = motor->can_cfg.port;
    can_msg_add_item(&motor->motor_msg.can_msg);
}

void Motor_DJI_Refresh(DJI_motor_t *motor)
{

    motor->motor_msg.motor_angle = (motor->motor_msg.can_msg.data[0] << 8) | motor->motor_msg.can_msg.data[1];
    motor->motor_msg.motor_speed = (motor->motor_msg.can_msg.data[2] << 8) | (motor->motor_msg.can_msg.data[3]);
    motor->motor_msg.torque_current = (motor->motor_msg.can_msg.data[4] << 8) | motor->motor_msg.can_msg.data[5];
    motor->motor_msg.temp = motor->motor_msg.can_msg.data[6]; 

}