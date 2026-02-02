#include "Watchdog_Task.h"

#include "Chassis_Task.h"
#include "DBusSys.h"
#include "can_api.h"
#include "can_struct.h"
#include "chassis_config.h"
#include "cmsis_os2.h"
#include "rising_ctrl.h"

#include "buzzer_bsp.h"
#include "motor_DM.h"

#include <string.h>

#ifndef __weak
#if defined(__GNUC__)
#define __weak __attribute__((weak))
#else
#define __weak
#endif
#endif

/**
 * @brief 用户可重写的弱函数回调（在不同状态下插入自定义处理）
 *
 * 使用说明：
 * - 这些函数在本文件里用 __weak 定义（空实现）。
 * - 你可以在任意 .c 文件中实现同名函数来覆盖这里的弱实现。
 * - index 含义：
 *   - 3508：0~3 为底盘四轮，4~5 为抬升两路 3508
 *   - DM：0 为左 DM，1 为右 DM
 */
__weak void Watchdog_OnChassis3508_Online(uint8_t index) { (void)index; }
__weak void Watchdog_OnChassis3508_Offline(uint8_t index) { (void)index; }

__weak void Watchdog_OnDm_Online(uint8_t index) { (void)index; }
__weak void Watchdog_OnDm_Offline(uint8_t index) { (void)index; }

__weak void Watchdog_OnDmError(uint8_t index, uint32_t code)
{
    (void)index;
    (void)code;
}

__weak void Watchdog_OnRc_Changed(void) {}
__weak void Watchdog_OnRc_NoChangeTimeout(void) {}

/**
 * @brief 获取当前系统时间（ms）
 */
static uint32_t Watchdog_GetTickMs(void)
{
    uint32_t tick = osKernelGetTickCount();
    uint32_t freq = osKernelGetTickFreq();
    if (freq == 0U) {
        return tick;
    }
    return (tick * 1000U) / freq;
}

/**
 * @brief 通过 CAN 接收计数 cnt 判断“是否在线”
 *
 * 判定逻辑：
 * - cnt 有变化：认为在线，并刷新 last_rx_ms
 * - cnt 无变化：若距离 last_rx_ms 未超过超时时间，仍认为在线
 */
static uint8_t Watchdog_IsOnlineByCnt(const can_msg_t *msg,
                                      uint8_t *last_cnt,
                                      uint32_t *last_rx_ms,
                                      uint32_t now_ms)
{
    if (msg == NULL || last_cnt == NULL || last_rx_ms == NULL) {
        return 0U;
    }

    if (msg->cnt != *last_cnt) {
        *last_cnt = msg->cnt;
        *last_rx_ms = now_ms;
        return 1U;
    }

    if ((now_ms - *last_rx_ms) <= WATCHDOG_MOTOR_OFFLINE_TIMEOUT_MS) {
        return 1U;
    }

    return 0U;
}

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

typedef enum
{
    WD_DM_L = 0,
    WD_DM_R,
    WD_DM_COUNT
} watchdog_dm_index_t;

/**
 * @brief 看门狗运行时状态
 */
typedef struct
{
    uint8_t last_cnt_3508[WD_3508_COUNT];
    uint32_t last_rx_ms_3508[WD_3508_COUNT];
    uint8_t online_3508[WD_3508_COUNT];

    uint8_t last_cnt_dm[WD_DM_COUNT];
    uint32_t last_rx_ms_dm[WD_DM_COUNT];
    uint8_t online_dm[WD_DM_COUNT];
    uint32_t dm_enable_last_ms[WD_DM_COUNT];

    rc_info_t last_rc;
    uint32_t last_rc_change_ms;
    uint8_t rc_offline;

    uint8_t alarm_active;
    uint8_t alarm_type;
    uint8_t alarm_motor_offline_cnt;
    uint32_t alarm_start_ms;
} Watchdog_State_t;

typedef enum
{
    WD_ALARM_NONE = 0,
    WD_ALARM_MOTOR_OFFLINE,
    WD_ALARM_RC_OFFLINE,
} watchdog_alarm_type_t;

static const uint16_t s_3508_ids[WD_3508_COUNT] = {
    Chassis_Motor_3508_ZQ_id,
    Chassis_Motor_3508_ZH_id,
    Chassis_Motor_3508_YH_id,
    Chassis_Motor_3508_YQ_id,
    Rising_Motor_3508_Left_id,
    Rising_Motor_3508_Right_id,
};

static const can_port_t s_3508_ports[WD_3508_COUNT] = {
    CAN1_PORT,
    CAN1_PORT,
    CAN1_PORT,
    CAN1_PORT,
    CAN1_PORT,
    CAN1_PORT,
};

static const uint16_t s_dm_master_ids[WD_DM_COUNT] = {
    DM_l0010l_Master_ID_Left,
    DM_l0010l_Master_ID_Right,
};

static const can_port_t s_dm_ports[WD_DM_COUNT] = {
    CAN3_PORT,
    CAN3_PORT,
};

static void Watchdog_Check3508(Watchdog_State_t *st, uint32_t now_ms)
{
    if (st == NULL) {
        return;
    }

    for (uint8_t i = 0; i < (uint8_t)WD_3508_COUNT; i++) {
        can_msg_t *msg = can_msg_find_item(can_msg_find_list(s_3508_ports[i]), s_3508_ids[i]);

        const uint8_t is_online = Watchdog_IsOnlineByCnt(msg,
                                                         &st->last_cnt_3508[i],
                                                         &st->last_rx_ms_3508[i],
                                                         now_ms);

        if (is_online != st->online_3508[i]) {
            st->online_3508[i] = is_online;
        }

        if (is_online) {
            Watchdog_OnChassis3508_Online(i);
        } else {
            Watchdog_OnChassis3508_Offline(i);
        }
    }
}

static void Watchdog_CheckDm(Watchdog_State_t *st, uint32_t now_ms)
{
    if (st == NULL) {
        return;
    }

    DM_motor_t *dm_l = Rising_Get_DmMotor_L();
    DM_motor_t *dm_r = Rising_Get_DmMotor_R();
    DM_motor_t *dm_list[WD_DM_COUNT] = {dm_l, dm_r};

    for (uint8_t i = 0; i < (uint8_t)WD_DM_COUNT; i++) {
        can_msg_t *msg = can_msg_find_item(can_msg_find_list(s_dm_ports[i]), s_dm_master_ids[i]);
        const uint8_t is_online = Watchdog_IsOnlineByCnt(msg, &st->last_cnt_dm[i], &st->last_rx_ms_dm[i], now_ms);

        if (is_online != st->online_dm[i]) {
            st->online_dm[i] = is_online;
        }

        if (is_online) {
            Watchdog_OnDm_Online(i);
        } else {
            Watchdog_OnDm_Offline(i);
        }

        if (is_online && dm_list[i] != NULL) {
            const uint32_t code = (uint32_t)dm_list[i]->error_code;
            Watchdog_OnDmError(i, code);

            if ((dm_list[i]->error_code == Motor_DM_DISABLE) && ((now_ms - st->dm_enable_last_ms[i]) > 200U)) {
                Motor_DM_Enable(dm_list[i]);
                st->dm_enable_last_ms[i] = now_ms;
            }
        }
    }
}

static uint8_t Watchdog_RcHasInput(const rc_info_t *rc)
{
    if (rc == NULL) {
        return 0U;
    }

    if ((rc->ch1 != 0) || (rc->ch2 != 0) || (rc->ch3 != 0) || (rc->ch4 != 0)) {
        return 1U;
    }

    return 0U;
}

static void Watchdog_CheckRc(Watchdog_State_t *st, uint32_t now_ms)
{
    if (st == NULL) {
        return;
    }

    rc_info_t rc_snapshot = remoter;

    if (memcmp(&rc_snapshot, &st->last_rc, sizeof(rc_snapshot)) != 0) {
        st->last_rc = rc_snapshot;
        st->last_rc_change_ms = now_ms;
        st->rc_offline = 0U;
        Watchdog_OnRc_Changed();
    } else {
        if (Watchdog_RcHasInput(&rc_snapshot)) {
            st->last_rc_change_ms = now_ms;
            st->rc_offline = 0U;
            return;
        }

        if ((now_ms - st->last_rc_change_ms) > WATCHDOG_RC_NOCHANGE_TIMEOUT_MS) {
            st->rc_offline = 1U;
            Watchdog_OnRc_NoChangeTimeout();
        } else {
            st->rc_offline = 0U;
        }
    }
}

static uint8_t Watchdog_CountMotorOffline(const Watchdog_State_t *st)
{
    uint8_t cnt = 0U;
    if (st == NULL) {
        return 0U;
    }

    for (uint8_t i = 0; i < (uint8_t)WD_3508_COUNT; i++) {
        if (st->online_3508[i] == 0U) {
            cnt++;
        }
    }
    for (uint8_t i = 0; i < (uint8_t)WD_DM_COUNT; i++) {
        if (st->online_dm[i] == 0U) {
            cnt++;
        }
    }

    return cnt;
}

static void Watchdog_Buzzer_Off(void)
{
    buzzer_off();
}

static void Watchdog_Buzzer_On(uint32_t freq_hz)
{
    buzzer_on(freq_hz, 0.50f);
}

static void Watchdog_AlarmStart(Watchdog_State_t *st, uint8_t type, uint32_t now_ms, uint8_t offline_cnt)
{
    if (st == NULL) {
        return;
    }

    st->alarm_active = 1U;
    st->alarm_type = type;
    st->alarm_start_ms = now_ms;
    st->alarm_motor_offline_cnt = offline_cnt;
}

static void Watchdog_AlarmUpdate(Watchdog_State_t *st, uint32_t now_ms, uint8_t motor_offline_cnt)
{
    if (st == NULL) {
        return;
    }

    if (st->alarm_active == 0U) {
        if (motor_offline_cnt > 0U) {
            Watchdog_AlarmStart(st, (uint8_t)WD_ALARM_MOTOR_OFFLINE, now_ms, motor_offline_cnt);
        } else if (st->rc_offline != 0U) {
            Watchdog_AlarmStart(st, (uint8_t)WD_ALARM_RC_OFFLINE, now_ms, 0U);
        } else {
            Watchdog_Buzzer_Off();
        }
        return;
    }

    if ((now_ms - st->alarm_start_ms) >= 2000U) {
        st->alarm_active = 0U;
        st->alarm_type = (uint8_t)WD_ALARM_NONE;
        Watchdog_Buzzer_Off();
        return;
    }

    if (st->alarm_type == (uint8_t)WD_ALARM_MOTOR_OFFLINE) {
        const uint32_t t = now_ms - st->alarm_start_ms;
        const uint32_t step = t / 200U;
        if (step < (uint32_t)st->alarm_motor_offline_cnt) {
            const uint32_t freq = 800U + (step * 200U);
            Watchdog_Buzzer_On(freq);
        } else {
            Watchdog_Buzzer_Off();
        }
    } else if (st->alarm_type == (uint8_t)WD_ALARM_RC_OFFLINE) {
        const uint32_t t = now_ms - st->alarm_start_ms;
        const uint32_t freq = 400U + ((t * 1600U) / 2000U);
        Watchdog_Buzzer_On(freq);
    } else {
        Watchdog_Buzzer_Off();
    }
}

void Watchdog_Task(void *argument)
{
    (void)argument;

    osDelay(500);

    Watchdog_State_t st;
    memset(&st, 0, sizeof(st));
    st.last_rc_change_ms = Watchdog_GetTickMs();

    //buzzer_init();

    for (;;) {
        const uint32_t now_ms = Watchdog_GetTickMs();

        Watchdog_Check3508(&st, now_ms);
        Watchdog_CheckDm(&st, now_ms);
        Watchdog_CheckRc(&st, now_ms);

        const uint8_t motor_offline_cnt = Watchdog_CountMotorOffline(&st);
        if ((motor_offline_cnt > 0U) || (st.rc_offline != 0U)) {
            Chassis_ForcePowerOff(1U);
        } else {
            Chassis_ForcePowerOff(0U);
        }

        Watchdog_AlarmUpdate(&st, now_ms, motor_offline_cnt);

        osDelay(1);
    }
}
