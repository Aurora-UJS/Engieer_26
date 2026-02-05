#ifndef WATCHDOG_CONFIG_H
#define WATCHDOG_CONFIG_H

#include <stdint.h>

// ============================
// 看门狗任务配置项（可按需修改）
// 说明：
// - 本项目的“看门狗”是软件任务 Watchdog_Task（电机在线检测/遥控器检测/蜂鸣器报警）。
// - 这些宏的单位默认均为 ms（毫秒）或 Hz（赫兹）。
// - 建议只在本文件内修改这些宏；其它文件会自动读取。
// ============================

#ifndef WATCHDOG_ENABLE_DEFAULT
// 看门狗任务默认开关：1 启用 Watchdog_Task 逻辑；0 禁用（不检测、不强制掉电）。
#define WATCHDOG_ENABLE_DEFAULT (0U)
#endif

#ifndef WATCHDOG_MOTOR_OFFLINE_TIMEOUT_MS
// 电机离线判定超时（ms）：CAN 反馈计数在该时间内无变化，则判定离线。
#define WATCHDOG_MOTOR_OFFLINE_TIMEOUT_MS (500U)
#endif

#ifndef WATCHDOG_RC_NOCHANGE_TIMEOUT_MS
// 遥控器“无输入/无变化”超时（ms）：超过该时间仍无输入变化则判定遥控器异常。
#define WATCHDOG_RC_NOCHANGE_TIMEOUT_MS (60000U)
#endif

#ifndef WATCHDOG_TASK_START_DELAY_MS
// 看门狗任务启动延时（ms）：上电后延迟一段时间再开始检测，避免初始化阶段误判。
#define WATCHDOG_TASK_START_DELAY_MS (500U)
#endif

#ifndef WATCHDOG_TASK_LOOP_DELAY_MS
// 看门狗任务循环周期（ms）：越小检测越频繁，但占用 CPU 越多。
#define WATCHDOG_TASK_LOOP_DELAY_MS (1U)
#endif

#ifndef WATCHDOG_TASK_DISABLED_DELAY_MS
// 看门狗关闭时的循环延时（ms）：用于降低空转时 CPU 占用。
#define WATCHDOG_TASK_DISABLED_DELAY_MS (10U)
#endif

#ifndef WATCHDOG_ALARM_DURATION_MS
// 报警持续时间（ms）：进入报警后持续该时间，随后自动停止蜂鸣器报警。
#define WATCHDOG_ALARM_DURATION_MS (2000U)
#endif

#ifndef WATCHDOG_ALARM_MOTOR_STEP_MS
// 电机离线报警节拍（ms）：每隔该时间切换一次蜂鸣器频率（与离线数量相关）。
#define WATCHDOG_ALARM_MOTOR_STEP_MS (200U)
#endif

#ifndef WATCHDOG_BUZZER_MOTOR_BASE_FREQ_HZ
// 电机离线报警的基准蜂鸣器频率（Hz）。
#define WATCHDOG_BUZZER_MOTOR_BASE_FREQ_HZ (800U)
#endif

#ifndef WATCHDOG_BUZZER_MOTOR_STEP_FREQ_HZ
// 电机离线报警的频率步进（Hz）：离线数越多，频率越高。
#define WATCHDOG_BUZZER_MOTOR_STEP_FREQ_HZ (200U)
#endif

#ifndef WATCHDOG_BUZZER_RC_BASE_FREQ_HZ
// 遥控器离线报警的基准蜂鸣器频率（Hz）。
#define WATCHDOG_BUZZER_RC_BASE_FREQ_HZ (400U)
#endif

#ifndef WATCHDOG_BUZZER_RC_RAMP_FREQ_HZ
// 遥控器离线报警的频率爬升幅度（Hz）：报警期间从 base 线性爬升 base+ramp。
#define WATCHDOG_BUZZER_RC_RAMP_FREQ_HZ (1600U)
#endif

#ifndef WATCHDOG_DM_REENABLE_PERIOD_MS
// DM 电机失能后的重使能周期（ms）：在线且失能时，超过该时间则尝试重新使能。
#define WATCHDOG_DM_REENABLE_PERIOD_MS (200U)
#endif

#endif /* WATCHDOG_CONFIG_H */
