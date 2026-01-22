#ifndef CHASSIS_DRIVE_H
#define CHASSIS_DRIVE_H

#include "chassis_config.h"

#include <stdint.h>

#include "DbusSys.h"
#include "LPF.h"
#include "PIDtool.h"
#include "arm_math_types.h"
#include "motor_DJI.h"

/**
 * @brief 初始化底盘轮系控制模块
 *
 * 初始化内容包括：电机句柄、PID、低通滤波器等。
 */
void Chassis_Drive_Init(void);

/**
 * @brief 关闭底盘轮电机输出（轮子停转）
 */
void Chassis_Stop(void);

/**
 * @brief 底盘普通模式控制
 *
 * @param remoter 遥控器数据指针
 */
void Chassis_Normal_Mode(const rc_info_t *remoter);

/**
 * @brief 底盘上楼模式控制
 *
 * @param remoter 遥控器数据指针
 */
void Chassis_Upstairs_Mode(const rc_info_t *remoter);

/**
 * @brief 初始化底盘轮电机（DJI 3508）
 *
 * 配置CAN相关参数并启动电机收发。
 *
 * @param Chassis_Motor 指向DJI电机结构体指针的指针（需要指向有效内存）
 */
void Chassis_Wheel_Init_DJI(DJI_motor_t **Chassis_Motor);

/**
 * @brief 初始化底盘轮速度PID
 *
 * @param pid PID数组（长度为4）
 */
void Chassis_3508_PID_Init(pid_type_def pid[]);

/**
 * @brief 计算底盘轮速度PID输出并进行低通滤波
 *
 * @param pid PID数组（长度为4）
 * @param target_speed 目标轮速数组（长度为4）
 * @param motor 电机反馈结构体
 * @param output 电流输出数组（长度为4）
 * @param lpf 低通滤波器数组（长度为4）
 */
void Chassis_3508_PID_Calculate(pid_type_def pid[], float32_t target_speed[],
                               DJI_motor_t *motor, int16_t output[], LowPassFilter lpf[]);

/**
 * @brief 遥控器通道映射为底盘运动速度，并通过麦轮运动学解算为四轮目标转速
 *
 * @param Target_Velocity 目标轮速数组（长度为4）
 * @param remoter 遥控器数据
 */
void Chassis_Motor_TargetVelocity(float32_t Target_Velocity[], rc_info_t remoter);

/**
 * @brief 发送底盘轮电机控制输出
 *
 * @param DJMotor DJI电机句柄
 * @param output 电流输出数组（长度为4）
 */
void Chassis_Motor_SendControl_DJI(DJI_motor_t *DJMotor, int16_t output[]);

/**
 * @brief 初始化底盘轮速度低通滤波器
 *
 * @param lpf 低通滤波器数组（长度为4）
 * @param alpha 低通滤波系数
 */
void Chassis_Wheel_LPF_Init(LowPassFilter lpf[4], float alpha);

#endif /* CHASSIS_DRIVE_H */
