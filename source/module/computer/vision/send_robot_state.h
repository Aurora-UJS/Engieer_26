#ifndef SEND_ROBOT_STATE_H
#define SEND_ROBOT_STATE_H

#include "main.h"

void send_robot_state(); // 发送机器人状态
void robot_state_msg_init(UART_HandleTypeDef *huart);

#endif // !SEND_ROBOT_STATE_H