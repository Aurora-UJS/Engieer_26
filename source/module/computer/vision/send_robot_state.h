#ifndef SEND_ROBOT_STATE_H
#define SEND_ROBOT_STATE_H

#include "main.h"

void send_robot_state(); // 发送机器人状态
void robot_state_msg_init(UART_HandleTypeDef *huart);
void set_robot_angle(float *yaw, float *pitch); // 绑定机器人角度
void set_robot_id(uint16_t id); // 设置机器人id
void change_robot_target(); // 改变机器人目标

#endif // !SEND_ROBOT_STATE_H