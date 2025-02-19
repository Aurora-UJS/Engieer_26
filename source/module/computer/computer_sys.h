#ifndef COMPUTER_SYS_H
#define COMPUTER_SYS_H

#define COMPUTER_BUFFER_SIZE 96

#include "main.h"
#include "config.h"
#include "send_robot_state.h"
#include "radarTool.h"
#include "vision_solution.h"
#include "vision_receive.h"

void computer_uart_init(UART_HandleTypeDef *huart);
void computer_receive_solve();

#endif // !COMPUTER_RECEIVE_H