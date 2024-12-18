#ifndef MOTOR_DJI_H
#define MOTOR_DJI_H

#include "motor_basic.h"
#include "can_struct.h"


typedef struct
{
    can_motor_t motor_msg;//电机数据包
    can_msg_t can_cfg; //CAN发送数据配置
    uint8_t error_code; //电机错误码
    uint8_t mode; //电机模式

}DJI_motor_t;

void set_motor_voltage(DJI_motor_t *msg,uint8_t Port,uint16_t ID,int16_t v1,int16_t v2,int16_t v3,int16_t v4);
void Motor_DJI_Init(DJI_motor_t *motor);
void Motor_DJI_Refresh(DJI_motor_t *motor);
#endif // MOTOR_DJI_H