#ifndef __MOTOR_BASIC_H__
#define __MOTOR_BASIC_H__

#include "main.h"
#include "bsp.h"
#include <stdbool.h>

typedef struct
{
    float motor_speed;    // 电机速度
    float motor_angle;    // 电机角度
    float torque_current; // 电机扭矩或电流
    int8_t temp;          // 电机温度
    can_msg_t can_msg;    // can消息指针

} can_motor_t;

#endif /* __MOTOR_COM_H__ */