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
    float chassis_power_MAX;        // 裁判系统最大允许功率
} total_power_control_t;

void PowerControl_Init(float *buffer_ptr,
                       float *cap_ptr,
                       float max_power,
                       pid_type_def *pid_params, void (*hook)(float));
void PowerControl_Update(void); // 更新功率控制状态
#endif // !POWERCONTROL_H