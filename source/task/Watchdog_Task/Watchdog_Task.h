#ifndef WATCHDOG_TASK_H
#define WATCHDOG_TASK_H

#include <stdint.h>

#define WATCHDOG_MOTOR_OFFLINE_TIMEOUT_MS 500U
#define WATCHDOG_RC_NOCHANGE_TIMEOUT_MS   20000U

/**
 * @brief 看门狗任务
 *
 * 功能：
 * 1) 检测底盘 6 个 3508 电机（底盘 4 + 抬升 2）是否持续有 CAN 反馈。
 * 2) 检测抬升 2 个 DM 电机是否持续有反馈；若在线则读取 DM_motor_t 的 error_code 并分支处理。
 * 3) 检测遥控器数据是否变化，超过 20s 未变化则强制底盘进入 PowerOff。
 *
 * 使用方式：
 * - 该任务会在 freertos.c 中被创建并周期运行。
 * - 若你需要在“在线/离线/错误码”等情况下插入自己的处理逻辑，
 *   可以在任意 .c 文件里实现同名的弱函数回调（在 Watchdog_Task.c 内有列出回调函数名）。
 */
void Watchdog_Task(void *argument);

#endif /* WATCHDOG_TASK_H */
