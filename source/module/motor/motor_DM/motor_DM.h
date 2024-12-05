#ifndef __MOTOR_DM_H__
#define __MOTOR_DM_H__

#include "main.h"
#include "motor_basic.h"

typedef struct
{
    can_motor_t motor_msg;//电机数据包
    can_msg_t can_cfg; //CAN发送数据配置
    uint8_t error_code; //电机错误码
    uint8_t mode; //电机模式

}DM_motor_t;


#endif /* __MOTOR_DM_H__ */