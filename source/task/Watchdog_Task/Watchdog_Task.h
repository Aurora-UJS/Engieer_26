#ifndef WATCHDOG_TASK_H
#define WATCHDOG_TASK_H

#include <stdint.h>

#include "watchdog_config.h"

#include "DBusSys.h"

// 3508 电机索引：用于回调与内部数组下标
typedef enum
{
    WD_3508_CHASSIS_ZQ = 0,
    WD_3508_CHASSIS_ZH,
    WD_3508_CHASSIS_YH,
    WD_3508_CHASSIS_YQ,
    WD_3508_RISING_L,
    WD_3508_RISING_R,
    WD_3508_COUNT
} watchdog_3508_index_t;

// DM 电机索引：用于回调与内部数组下标
typedef enum
{
    WD_DM_L = 0,
    WD_DM_R,
    WD_DM_COUNT
} watchdog_dm_index_t;

// 报警类型：用于蜂鸣器报警逻辑与状态机
typedef enum
{
    WD_ALARM_NONE = 0,
    WD_ALARM_MOTOR_OFFLINE,
    WD_ALARM_RC_OFFLINE,
} watchdog_alarm_type_t;

/**
 * @brief 看门狗运行时状态
 */
typedef struct
{
    // 3508 在线检测（通过 CAN cnt 判断）
    uint8_t last_cnt_3508[WD_3508_COUNT];
    uint32_t last_rx_ms_3508[WD_3508_COUNT];
    uint8_t online_3508[WD_3508_COUNT];

    // DM 在线检测 + 自动重使能
    uint8_t last_cnt_dm[WD_DM_COUNT];
    uint32_t last_rx_ms_dm[WD_DM_COUNT];
    uint8_t online_dm[WD_DM_COUNT];
    uint32_t dm_enable_last_ms[WD_DM_COUNT];

    // 遥控器变化检测（长时间无变化/无输入则判定异常）
    rc_info_t last_rc;
    uint32_t last_rc_change_ms;
    uint8_t rc_offline;

    // 报警状态（蜂鸣器）
    uint8_t alarm_active;
    uint8_t alarm_type;
    uint8_t alarm_motor_offline_cnt;
    uint32_t alarm_start_ms;
} Watchdog_State_t;

// 运行时开关：用于在代码层面启用/禁用 Watchdog_Task 逻辑
void Watchdog_SetEnabled(uint8_t enabled);
uint8_t Watchdog_IsEnabled(void);

// 弱函数回调声明：你可以在任意 .c 文件中实现同名函数来覆盖 Watchdog_Task.c 内的 __weak 空实现
void Watchdog_OnChassis3508_Online(uint8_t index);
void Watchdog_OnChassis3508_Offline(uint8_t index);
void Watchdog_OnDm_Online(uint8_t index);
void Watchdog_OnDm_Offline(uint8_t index);
void Watchdog_OnDmError(uint8_t index, uint32_t code);
void Watchdog_OnRc_Changed(void);
void Watchdog_OnRc_NoChangeTimeout(void);

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
