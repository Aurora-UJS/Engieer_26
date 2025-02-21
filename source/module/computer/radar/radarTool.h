#ifndef _RADARTOOL_H_
#define _RADARTOOL_H_
#include "main.h"
#include "stdint.h"
#include "stdbool.h"
#include "radarTool.h"
#include "computer_pack.h"


#define RADAR_BUFFER_SIZE 100


#define CRC_Check_False   0
#define CRC_Check_True    1
typedef unsigned char u8;

extern uint8_t rader_rx_buffer[RADAR_BUFFER_SIZE];
extern ReceivedPacketTwist_t radarTwist; 
extern ReceivedPacketRobotControl_t robotControl;

/* 雷达/云台目标运动状态获取宏组 ----------------------------------------------------*/

/**
 * @brief 获取目标在X轴方向的线速度
 * @return 目标X轴线速度值，单位：米/秒(m/s)
 */
#define get_target_x() (radarTwist.linear_x)

/**
 * @brief 获取目标在Y轴方向的线速度
 * @return 目标Y轴线速度值，单位：米/秒(m/s) 
 */
#define get_target_x() (radarTwist.linear_x)

/**
 * @brief 获取目标在Z轴方向的线速度
 * @return 目标Z轴线速度值，单位：弧度/秒(rad/s)
 */
#define get_target_z() (radarTwist.angular_z)

/* 雷达/云台自身角速度获取宏组 ------------------------------------------------------*/

/**
 * @brief 获取雷达/云台绕X轴的旋转角速度
 * @return X轴角速度值，单位：弧度/秒(rad/s)
 */
#define get_angular_x() (radarTwist.angular_x)

/**
 * @brief 获取雷达/云台绕Y轴的旋转角速度
 * @return Y轴角速度值，单位：弧度/秒(rad/s)
 */
#define get_angular_y() (radarTwist.angular_y)

/**
 * @brief 获取雷达/云台绕Z轴的旋转角速度
 * @return Z轴角速度值，单位：弧度/秒(rad/s)
 */
#define get_angular_z() (radarTwist.angular_z)

/* 机器人控制状态获取宏组 ----------------------------------------------------------*/

/**
 * @brief 获取云台扫描停止状态
 * @return 停止扫描标志位，布尔类型：true-停止扫描，false-允许扫描
 */
#define get_gimbal_stop() (robotControl.stop_gimbal_scan)

/**
 * @brief 获取底盘旋转速度
 * @return 底盘旋转速度值，单位：弧度/秒(rad/s)
 * @note 正值表示顺时针旋转，负值表示逆时针旋转
 */
#define get_chassis_spin() (robotControl.chassis_spin_vel)

void receiveRadarData(uint8_t *rx_buffer);
void radar_msg_init(UART_HandleTypeDef *huart);
void sendAllRobotHPData(SendPacketAllRobotHP_t *sendAllRobotHP);
void sendGameStatusData(SendPacketGameStatus_t *sendGameStatus);
void sendRobotStatusData(SendPacketRobotStatus_t *sendRobotStatus);


#endif // !_RADARTOOL_H_



