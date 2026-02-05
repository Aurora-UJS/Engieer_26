#include "PIDtool.h"
#include "stdint.h"
#include "string.h"
#include "tool.h"
// 定义PID控制器结构体
// pid_type_def pid;
/**
 * @brief 初始化PID控制器
 * 
 * @param pid pid控制器结构体
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 * @param max_out 输出最大值
 * @param max_iout 积分输出最大值
 */
void PID_Init(pid_type_def *pid, const float kp, const float ki, const float kd, float max_out, float max_iout)
{
    memset(pid, 0, sizeof(pid_type_def));
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;

}

/**
 * @brief 计算增量式PID控制器输出
 * 
 * @param pid pid控制器结构体
 * @param now 当前值
 * @param target 目标值
 * @return float 输出值
 */
float PID_Calc_Add(pid_type_def *pid, float now, float target)
{
    // 更新目标值和反馈值
    pid->set = target;
    pid->fdb = now;

    // 更新误差值
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->error[0] = target - now;

    // 计算比例、积分和微分输出
    pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
    pid->Iout += pid->Ki * pid->error[0];
    LimitMax(pid->Iout, pid->max_iout);

    // 更新微分项缓冲区
    pid->Dbuf[2] = pid->Dbuf[1];
    pid->Dbuf[1] = pid->Dbuf[0];
    pid->Dbuf[0] = pid->error[0] - 2.0f * pid->error[1] + pid->error[2];
    pid->Dout = pid->Kd * pid->Dbuf[0];

    // 计算最终输出并限制在最大输出范围内
    pid->out += pid->Pout + pid->Iout + pid->Dout;
    LimitMax(pid->out, pid->max_out);

    return pid->out;

}

/**
 * @brief 计算位置式PID控制器输出
 * 
 * @param pid pid控制器结构体
 * @param now 当前值
 * @param target 目标值
 * @return float 输出值
 */
float PID_Calc_Pos(pid_type_def *pid, float now, float target) 
{
    // 更新目标值和反馈值
    pid->set = target;
    pid->fdb = now;

    // 更新误差值
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->error[0] = target - now;

    // 计算比例、积分和微分输出
    pid->Pout = pid->Kp * pid->error[0];
    pid->Iout += pid->Ki * pid->error[0];
    LimitMax(pid->Iout, pid->max_iout);

    pid->Dbuf[2] = pid->Dbuf[1];
    pid->Dbuf[1] = pid->Dbuf[0];
    pid->Dbuf[0] = target;
    pid->Dout = pid->Kd * (pid->Dbuf[0] - pid->Dbuf[1]);

    // 计算最终输出并限制在最大输出范围内
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    LimitMax(pid->out, pid->max_out);

    return pid->out;
}


