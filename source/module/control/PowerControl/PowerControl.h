#ifndef POWERCONTROL_H
#define POWERCONTROL_H

#include "main.h"
#include "PIDtool.h"

/* 类型定义 */
typedef enum
{
    POWER_LOSS = 0, // 功率受限模式
    POWER_NORMAL,   // 电容常规支持模式
    POWER_BURST     // 电容爆发输出模式
} power_control_state_t;

typedef struct
{
    pid_type_def chassis_power_pid; // 底盘功率缓冲PID
    float *chassis_power_buffer;    // 指向功率缓冲值的指针
    float *chassis_power_MAX;       // 裁判系统最大允许功率
} total_power_control_t;

// 电机功率计算参数结构体
typedef struct
{
    float torque_coeff;  // 扭矩系数 (默认: 1.99688994e-6f)
    float k1;  // 机械损耗系数 (默认: 1.23e-07)
    float k2; // 铜损系数 (默认: 1.453e-07)
    float k3; // 恒定损耗 (默认: 4.081f)
} MotorPowerParams_t;

void PowerControl_Init(float *buffer_ptr,
                       float *cap_ptr,
                       float *max_power,
                       pid_type_def *pid_params, void (*hook)(float));
void PowerControl_Update(void);                        // 更新功率控制状态
void Set_PowerControlMode(power_control_state_t mode); // 设置功率控制模式
float calculate_discriminant(float motor_speed, MotorPowerParams_t params, float scaled_power);
float MotorPower_CalculateSingle(
    float pid_output,
    float speed_rpm,
    const MotorPowerParams_t *params);
#endif // !POWERCONTROL_H