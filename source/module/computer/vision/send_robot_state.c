/* send_robot_state.c - 机器人状态数据打包及UART发送模块 */

#include "computer_pack.h"
#include "main.h"
#include "usart.h"
#include "uart_api.h"
#include "vision_solution.h"
#include "crc_api.h"

/* 全局变量声明 */
SendPacketVision_t robot_State_Data; // 视觉数据包结构体实例
static uint16_t robot_id = 0;        // 机器人唯一标识符
static float *yaw_angle_now;         // 当前偏航角（单位：度）
static float *pitch_angle_now;       // 当前俯仰角（单位：度）
static bool reset_flag = 0;          // 跟踪器重置标志
static uart_msg_t robot_state_msg;   // UART消息传输结构体

/**
 * @brief 初始化机器人状态UART消息结构体
 * @param huart 指向UART外设句柄的指针
 * @note 配置UART参数并关联数据缓冲区
 */
void robot_state_msg_init(UART_HandleTypeDef *huart)
{
  robot_state_msg.huart = huart;                          // 绑定UART硬件接口
  robot_state_msg.pBuffer = (uint8_t *)&robot_State_Data; // 设置数据缓冲区指针
  robot_state_msg.Len = sizeof(robot_State_Data);         // 计算数据包长度
}

/**
 * @brief 发送机器人状态数据包
 * @note 数据包结构：
 * - 包头：0x5A（固定标识）
 * - 颜色检测：0=未检测颜色模式
 * - 跟踪重置：0=保持当前跟踪状态
 * - 预留字段：保留未来扩展
 * - 姿态数据：当前偏航/俯仰角度
 * - 目标坐标：归一化后的三维坐标
 * - 自动添加CRC16校验
 */
void send_robot_state()
{
  /* 填充数据包头及状态字段 */
  robot_State_Data.header = 0x5A;                         // 固定包头标识
  robot_State_Data.detect_color = robot_id > 100 ? 0 : 1; // 自身颜色检测状态
  robot_State_Data.reset_tracker = reset_flag;            // 跟踪器保持运行
  robot_State_Data.reserved = 0;                          // 预留字段初始化
  robot_State_Data.roll = 0;                              // 横滚角暂未使用

  /* 更新实时姿态数据 */
  robot_State_Data.yaw = *yaw_angle_now;     // 当前偏航角
  robot_State_Data.pitch = *pitch_angle_now; // 当前俯仰角

  /* 填充目标坐标（需确保aim_x/y/z_n已正确定义） */
  robot_State_Data.aim_x = aim_x_n; // X轴归一化坐标
  robot_State_Data.aim_y = aim_y_n; // Y轴归一化坐标
  robot_State_Data.aim_z = aim_z_n; // Z轴归一化坐标

  /* 数据校验与发送 */
  Append_CRC16_Check_Sum((uint8_t *)&robot_State_Data, sizeof(robot_State_Data)); // 添加CRC校验
  HAL_UART_Transmit_IT(robot_state_msg.huart,
                       (uint8_t *)&robot_State_Data,
                       sizeof(robot_State_Data)); // 非阻塞式UART发送
}

// 设置机器人角度
void set_robot_angle(float *yaw, float *pitch)
{
  // 设置当前yaw角度
  yaw_angle_now = yaw;
  // 设置当前pitch角度
  pitch_angle_now = pitch;
}
// 设置机器人ID
void set_robot_id(uint16_t id)
{
  // 设置机器人ID
  robot_id = id;
}

// 函数：改变机器人目标
void change_robot_target()
{
  // 重置标志位
  reset_flag = 1;
  // 发送机器人状态
  send_robot_state();
  // 重置标志位
  reset_flag = 0;
}