#ifndef __MOTOR_BASIC_H__
#define __MOTOR_BASIC_H__

#include "main.h"
#include "bsp.h"
#include <stdbool.h>

typedef struct
{
    float motor_speed;
    float motor_angle;
    float torque_current;
    int8_t  temp;
    can_msg_t can_msg; //can消息指针

} can_motor_t;





#endif /* __MOTOR_COM_H__ */ 