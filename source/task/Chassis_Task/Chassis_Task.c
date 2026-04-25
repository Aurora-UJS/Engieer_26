#include "Chassis_Task.h"

#include "Referee_Task.h"
#include "arm_state_machine.h"
#include "chassis_drive.h"
#include "rising_ctrl.h"
#include "tool.h"

#include <stdint.h>

extern rc_info_t remoter;

volatile Chassis_Mode_State_t g_chassis_mode_state = CHASSIS_MODE_STATE_Normal;
volatile Chassis_Control_Source_State_t g_chassis_control_source_state = CHASSIS_CONTROL_SOURCE_STATE_DBUS;
volatile Chassis_Rising_Behavior_State_t g_chassis_rising_behavior_state =
    (Chassis_Rising_Behavior_State_t)CHASSIS_RISING_BEHAVIOR_DEFAULT;

static volatile uint8_t s_chassis_force_poweroff = 0U;
static volatile uint8_t s_chassis_rising_start_request = 0U;
static uint8_t s_chassis_dbus_rising_switch_prev_front = 0U;
static uint8_t s_chassis_dbus_left_not_front_right_front_prev_active = 0U;

typedef enum
{
    CHASSIS_RISING_RUNTIME_STATE_IDLE = 0,
    CHASSIS_RISING_RUNTIME_STATE_LIFTING,
    CHASSIS_RISING_RUNTIME_STATE_TRANSITION,
    CHASSIS_RISING_RUNTIME_STATE_DRIVING,
    CHASSIS_RISING_RUNTIME_STATE_DOUBLE_RISING_HOLD,
    CHASSIS_RISING_RUNTIME_STATE_DOUBLE_NORMAL_HOLD,
    CHASSIS_RISING_RUNTIME_STATE_FINISHED,
} Chassis_Rising_Runtime_State_t;

typedef struct
{
    Chassis_Rising_Runtime_State_t state;
    uint32_t state_start_tick;
} Chassis_Rising_Runtime_Ctx_t;

static uint8_t Chassis_IsPowerOffRequested(void);
static Chassis_Control_Source_State_t Chassis_GetControlSourceState(void);
static Chassis_Mode_State_t Chassis_GetRequestedModeState(void);
static void Chassis_SyncModeStateFromSource(void);
static void Chassis_SanitizeRisingBehaviorState(void);
static uint8_t Chassis_IsArmInRisingMode(void);
static uint8_t Chassis_IsDbusLeftNotFrontRightFront(void);
static uint8_t Chassis_IsDbusLeftMiddleRightFront(void);
static Chassis_Rising_Behavior_State_t Chassis_GetActiveRisingBehaviorState(void);
static uint8_t Chassis_TakeDbusRisingFrontEdge(void);
static uint8_t Chassis_TakeDbusLeftNotFrontRightFrontEdge(void);
static uint32_t Chassis_MsToTicks(uint32_t duration_ms);
static void Chassis_ResetRisingRuntimeCtxOnly(Chassis_Rising_Runtime_Ctx_t *ctx);
static void Chassis_ResetRisingRuntime(Chassis_Rising_Runtime_Ctx_t *ctx);
static void Chassis_UpdateRisingRuntime(Chassis_Rising_Runtime_Ctx_t *ctx);
static void Chassis_UpdateDbusLeftNotFrontRightFrontRuntime(Chassis_Rising_Runtime_Ctx_t *ctx);
static void Chassis_ExecuteNormalBySource(const keyboard_t *active_kb);
static void Chassis_ExecuteRisingRegularBySource(const keyboard_t *active_kb);
static void Chassis_ExecuteRisingStateMachine(const keyboard_t *active_kb,
                                              const Chassis_Rising_Runtime_Ctx_t *ctx);
static void Chassis_ExecuteDbusLeftNotFrontRightFrontStateMachine(const keyboard_t *active_kb,
                                                                  const Chassis_Rising_Runtime_Ctx_t *ctx);
static void Chassis_RunSequenceLift(int16_t chassis_ch2, int16_t rising_ch2);
static void Chassis_RunNormalHoldStop(void);
static void Chassis_RunNormalHoldDrive(int16_t chassis_ch2);
static void Chassis_RunNormalHoldBySource(const keyboard_t *active_kb);

/* 对外保留一个显式控制源入口。
 * 实际底盘任务每周期都会从 Engineer_Mode.Ctrl_Logic_Mode 同步，
 * 所以这里同时回写 Engineer_Mode，保证外部设置不会下一拍失效。
 */
void Chassis_SetControlSourceState(Chassis_Control_Source_State_t source_state)
{
    if ((uint8_t)source_state > (uint8_t)CHASSIS_CONTROL_SOURCE_STATE_Keyboard) {
        return;
    }

    g_chassis_control_source_state = source_state;
    Engineer_Mode.Ctrl_Logic_Mode =
        (source_state == CHASSIS_CONTROL_SOURCE_STATE_Keyboard) ? CTRL_LOGIC_MODE_Keyboard : CTRL_LOGIC_MODE_DBUS;
}

/* 对外提供底盘总状态设置接口。
 * 注意：真正持久生效的控制入口取决于当前控制源：
 * 1. DBUS源下，PowerOff/Normal/Rising 会映射为遥控器逻辑可识别的状态；
 * 2. Keyboard源下，底盘模式由 Engineer_Mode.Chassis_Ctrl_Mode 持续驱动。
 */
void Chassis_SetModeState(Chassis_Mode_State_t mode_state)
{
    switch (mode_state) {
        case CHASSIS_MODE_STATE_PowerOff:
            Chassis_ForcePowerOff(1U);
            Engineer_Mode.Chassis_Ctrl_Mode = CHASSIS_CTRL_MODE_Normal;
            break;

        case CHASSIS_MODE_STATE_Normal:
            Chassis_ForcePowerOff(0U);
            Engineer_Mode.Chassis_Ctrl_Mode = CHASSIS_CTRL_MODE_Normal;
            break;

        case CHASSIS_MODE_STATE_Rising:
            Chassis_ForcePowerOff(0U);
            Engineer_Mode.Chassis_Ctrl_Mode = CHASSIS_CTRL_MODE_Rising;
            break;

        default:
            return;
    }

    g_chassis_mode_state = mode_state;
}

/* 一级/二级抬升行为与总状态解耦，单独暴露接口，便于外部测试。 */
void Chassis_SetRisingBehaviorState(Chassis_Rising_Behavior_State_t behavior_state)
{
    if ((uint8_t)behavior_state > (uint8_t)CHASSIS_RISING_BEHAVIOR_STATE_DoubleLift) {
        return;
    }

    g_chassis_rising_behavior_state = behavior_state;
}

/* R 键功能统一放在底盘层处理：
 * 1. 不在 Rising 总模式时，R 只切换一级/二级子状态机；
 * 2. 在 Rising 总模式时，只有机械臂当前已进入 Arm_Rising_Mode，R 才允许启动当前选中的子状态机；
 * 3. 常规 Rising 与一级/二级流程解耦，未启动子状态机时始终走常规 Rising。
 */
void Chassis_HandleRisingKeyPressed(void)
{
    if (Chassis_GetRequestedModeState() == CHASSIS_MODE_STATE_Rising) {
        if (Chassis_IsArmInRisingMode() == 0U) {
            s_chassis_rising_start_request = 0U;
            return;
        }

        s_chassis_rising_start_request = 1U;
        return;
    }

    if (g_chassis_rising_behavior_state == CHASSIS_RISING_BEHAVIOR_STATE_SingleLift) {
        g_chassis_rising_behavior_state = CHASSIS_RISING_BEHAVIOR_STATE_DoubleLift;
    } else {
        g_chassis_rising_behavior_state = CHASSIS_RISING_BEHAVIOR_STATE_SingleLift;
    }
}

Chassis_Mode_State_t Chassis_GetModeState(void)
{
    return g_chassis_mode_state;
}

Chassis_Control_Source_State_t Chassis_GetControlSourceStatePublic(void)
{
    return g_chassis_control_source_state;
}

Chassis_Rising_Behavior_State_t Chassis_GetRisingBehaviorState(void)
{
    return g_chassis_rising_behavior_state;
}

void Chassis_ForcePowerOff(uint8_t enable)
{
    s_chassis_force_poweroff = (enable != 0U) ? 1U : 0U;
}

uint8_t Chassis_IsForcePowerOff(void)
{
    return s_chassis_force_poweroff;
}

static uint8_t Chassis_IsPowerOffRequested(void)
{
    /* 掉电是总状态机的最高优先级条件。
     * DBUS源下读拨码或外部强制掉电；
     * Keyboard源下仅保留外部强制掉电，避免遥控器继续插手键盘控制。
     */
    if (Chassis_IsForcePowerOff() != 0U) {
        return 1U;
    }

    if (g_chassis_control_source_state == CHASSIS_CONTROL_SOURCE_STATE_DBUS) {
        return (remoter.sw2 == 2) ? 1U : 0U;
    }

    return 0U;
}

static Chassis_Control_Source_State_t Chassis_GetControlSourceState(void)
{
    return (Engineer_Mode.Ctrl_Logic_Mode == CTRL_LOGIC_MODE_Keyboard)
               ? CHASSIS_CONTROL_SOURCE_STATE_Keyboard
               : CHASSIS_CONTROL_SOURCE_STATE_DBUS;
}

static Chassis_Mode_State_t Chassis_GetRequestedModeState(void)
{
    if (Chassis_IsPowerOffRequested() != 0U) {
        return CHASSIS_MODE_STATE_PowerOff;
    }

    if (g_chassis_control_source_state == CHASSIS_CONTROL_SOURCE_STATE_Keyboard) {
        return (Engineer_Mode.Chassis_Ctrl_Mode == CHASSIS_CTRL_MODE_Rising)
                   ? CHASSIS_MODE_STATE_Rising
                   : CHASSIS_MODE_STATE_Normal;
    }

    return (remoter.sw2 == 1) ? CHASSIS_MODE_STATE_Rising : CHASSIS_MODE_STATE_Normal;
}

static void Chassis_SyncModeStateFromSource(void)
{
    const Chassis_Mode_State_t requested_mode = Chassis_GetRequestedModeState();

    /* g_chassis_mode_state 作为“总状态机结果”对外发布，
     * 但总状态本身是由控制源和当前模式输入共同推导出来的。
     */
    g_chassis_mode_state = requested_mode;

    if (g_chassis_control_source_state == CHASSIS_CONTROL_SOURCE_STATE_DBUS) {
        Engineer_Mode.Chassis_Ctrl_Mode =
            (requested_mode == CHASSIS_MODE_STATE_Rising) ? CHASSIS_CTRL_MODE_Rising : CHASSIS_CTRL_MODE_Normal;
    }
}

static void Chassis_SanitizeRisingBehaviorState(void)
{
    if ((uint8_t)g_chassis_rising_behavior_state > (uint8_t)CHASSIS_RISING_BEHAVIOR_STATE_DoubleLift) {
        g_chassis_rising_behavior_state = CHASSIS_RISING_BEHAVIOR_STATE_SingleLift;
    }
}

static uint8_t Chassis_IsArmInRisingMode(void)
{
    return (Arm_Current_Control_Mode == Arm_Rising_Mode) ? 1U : 0U;
}

static uint8_t Chassis_IsDbusLeftNotFrontRightFront(void)
{
    if (g_chassis_control_source_state != CHASSIS_CONTROL_SOURCE_STATE_DBUS) {
        return 0U;
    }

    return ((remoter.sw1 != 1U) && (remoter.sw2 == 1U)) ? 1U : 0U;
}

static uint8_t Chassis_IsDbusLeftMiddleRightFront(void)
{
    if (g_chassis_control_source_state != CHASSIS_CONTROL_SOURCE_STATE_DBUS) {
        return 0U;
    }

    return ((remoter.sw1 == 3U) && (remoter.sw2 == 1U)) ? 1U : 0U;
}

static Chassis_Rising_Behavior_State_t Chassis_GetActiveRisingBehaviorState(void)
{
    /* DBUS 模式下:
     * 右拨杆前沿触发后只执行一级 rising；
     * Keyboard 模式仍保持独立测试逻辑，不在这里改动。
     */
    if (g_chassis_control_source_state == CHASSIS_CONTROL_SOURCE_STATE_DBUS) {
        return CHASSIS_RISING_BEHAVIOR_STATE_SingleLift;
    }

    return g_chassis_rising_behavior_state;
}

static uint8_t Chassis_TakeDbusRisingFrontEdge(void)
{
    uint8_t current_front = 0U;
    uint8_t edge_detected = 0U;

    if (g_chassis_control_source_state == CHASSIS_CONTROL_SOURCE_STATE_DBUS) {
        current_front = (remoter.sw2 == 1) ? 1U : 0U;
    }

    if ((current_front != 0U) && (s_chassis_dbus_rising_switch_prev_front == 0U)) {
        edge_detected = 1U;
    }

    s_chassis_dbus_rising_switch_prev_front = current_front;
    return edge_detected;
}

static uint8_t Chassis_TakeDbusLeftNotFrontRightFrontEdge(void)
{
    const uint8_t current_active = Chassis_IsDbusLeftNotFrontRightFront();
    uint8_t edge_detected = 0U;

    if ((current_active != 0U) &&
        (s_chassis_dbus_left_not_front_right_front_prev_active == 0U)) {
        edge_detected = 1U;
    }

    s_chassis_dbus_left_not_front_right_front_prev_active = current_active;
    return edge_detected;
}

static uint32_t Chassis_MsToTicks(uint32_t duration_ms)
{
    return (uint32_t)((duration_ms * osKernelGetTickFreq()) / 1000U);
}

static void Chassis_ResetRisingRuntimeCtxOnly(Chassis_Rising_Runtime_Ctx_t *ctx)
{
    if (ctx == NULL) {
        return;
    }

    ctx->state = CHASSIS_RISING_RUNTIME_STATE_IDLE;
    ctx->state_start_tick = 0U;
}

static void Chassis_ResetRisingRuntime(Chassis_Rising_Runtime_Ctx_t *ctx)
{
    Chassis_ResetRisingRuntimeCtxOnly(ctx);
    s_chassis_rising_start_request = 0U;
}

static void Chassis_UpdateRisingRuntime(Chassis_Rising_Runtime_Ctx_t *ctx)
{
    const uint32_t now = osKernelGetTickCount();
    const Chassis_Rising_Behavior_State_t active_behavior = Chassis_GetActiveRisingBehaviorState();
    const uint8_t dbus_rising_front_edge = Chassis_TakeDbusRisingFrontEdge();

    if (ctx == NULL) {
        return;
    }

    if (g_chassis_mode_state != CHASSIS_MODE_STATE_Rising) {
        Chassis_ResetRisingRuntime(ctx);
        return;
    }

    if (g_chassis_control_source_state == CHASSIS_CONTROL_SOURCE_STATE_DBUS) {
        /* DBUS 模式下：
         * 1. 机械臂当前模式是一级 rising 的允许条件；
         * 2. 右拨杆拨到前的边沿才是一级 rising 的真正触发条件；
         * 3. 不满足条件时保持原版普通 rising。
         */
        if ((dbus_rising_front_edge != 0U) &&
            (Chassis_IsArmInRisingMode() != 0U) &&
            ((ctx->state == CHASSIS_RISING_RUNTIME_STATE_IDLE) ||
             (ctx->state == CHASSIS_RISING_RUNTIME_STATE_FINISHED))) {
            ctx->state = CHASSIS_RISING_RUNTIME_STATE_LIFTING;
            ctx->state_start_tick = now;
        }
    } else {
        /* Keyboard 模式保持现有测试逻辑:
         * 只有机械臂当前处在 rising 模式时，R 启动请求才允许推进子状态机。
         */
        if (Chassis_IsArmInRisingMode() == 0U) {
            Chassis_ResetRisingRuntime(ctx);
            return;
        }

        if ((s_chassis_rising_start_request != 0U) &&
            ((ctx->state == CHASSIS_RISING_RUNTIME_STATE_IDLE) ||
             (ctx->state == CHASSIS_RISING_RUNTIME_STATE_FINISHED))) {
            ctx->state = CHASSIS_RISING_RUNTIME_STATE_LIFTING;
            ctx->state_start_tick = now;
            s_chassis_rising_start_request = 0U;
        }
    }

    switch (ctx->state) {
        case CHASSIS_RISING_RUNTIME_STATE_LIFTING:
            if ((uint32_t)(now - ctx->state_start_tick) >=
                Chassis_MsToTicks((active_behavior == CHASSIS_RISING_BEHAVIOR_STATE_SingleLift)
                                      ? CHASSIS_RISING_SINGLE_LIFT_DURATION_MS
                                      : CHASSIS_RISING_DOUBLE_LIFT_DURATION_MS)) {
                ctx->state = CHASSIS_RISING_RUNTIME_STATE_TRANSITION;
                ctx->state_start_tick = now;
            }
            break;

        case CHASSIS_RISING_RUNTIME_STATE_TRANSITION:
            if ((uint32_t)(now - ctx->state_start_tick) >=
                Chassis_MsToTicks((active_behavior == CHASSIS_RISING_BEHAVIOR_STATE_SingleLift)
                                      ? CHASSIS_RISING_SINGLE_TRANSITION_DURATION_MS
                                      : CHASSIS_RISING_DOUBLE_TRANSITION_DURATION_MS)) {
                ctx->state = CHASSIS_RISING_RUNTIME_STATE_DRIVING;
                ctx->state_start_tick = now;
            }
            break;

        case CHASSIS_RISING_RUNTIME_STATE_DRIVING:
            if ((uint32_t)(now - ctx->state_start_tick) >=
                Chassis_MsToTicks((active_behavior == CHASSIS_RISING_BEHAVIOR_STATE_SingleLift)
                                      ? CHASSIS_RISING_SINGLE_DRIVE_DURATION_MS
                                      : CHASSIS_RISING_DOUBLE_DRIVE_DURATION_MS)) {
                if (active_behavior == CHASSIS_RISING_BEHAVIOR_STATE_DoubleLift) {
                    ctx->state = CHASSIS_RISING_RUNTIME_STATE_DOUBLE_RISING_HOLD;
                    ctx->state_start_tick = now;
                } else {
                    ctx->state = CHASSIS_RISING_RUNTIME_STATE_FINISHED;
                }
            }
            break;

        case CHASSIS_RISING_RUNTIME_STATE_DOUBLE_RISING_HOLD:
            if ((uint32_t)(now - ctx->state_start_tick) >=
                Chassis_MsToTicks(CHASSIS_RISING_DOUBLE_RISING_HOLD_DURATION_MS)) {
                ctx->state = CHASSIS_RISING_RUNTIME_STATE_DOUBLE_NORMAL_HOLD;
                ctx->state_start_tick = now;
            }
            break;

        case CHASSIS_RISING_RUNTIME_STATE_DOUBLE_NORMAL_HOLD:
            if ((uint32_t)(now - ctx->state_start_tick) >=
                Chassis_MsToTicks(CHASSIS_RISING_DOUBLE_NORMAL_HOLD_DURATION_MS)) {
                ctx->state = CHASSIS_RISING_RUNTIME_STATE_FINISHED;
            }
            break;

        case CHASSIS_RISING_RUNTIME_STATE_FINISHED:
        case CHASSIS_RISING_RUNTIME_STATE_IDLE:
        default:
            break;
    }
}

/* 复制一份 DBUS Rising 内部状态机，专门挂到“左不在前、右在前”的拨杆组合上。
 * 这样后续如果要把这一路单独改动作，不会影响现有的普通 Rising 流程。
 */
static void Chassis_UpdateDbusLeftNotFrontRightFrontRuntime(Chassis_Rising_Runtime_Ctx_t *ctx)
{
    const uint32_t now = osKernelGetTickCount();
    const Chassis_Rising_Behavior_State_t active_behavior = Chassis_GetActiveRisingBehaviorState();
    const uint8_t dbus_combo_front_edge = Chassis_TakeDbusLeftNotFrontRightFrontEdge();

    if (ctx == NULL) {
        return;
    }

    if ((g_chassis_mode_state != CHASSIS_MODE_STATE_Rising) ||
        (Chassis_IsDbusLeftNotFrontRightFront() == 0U)) {
        Chassis_ResetRisingRuntime(ctx);
        return;
    }

        if ((dbus_combo_front_edge != 0U) &&
            (Chassis_IsArmInRisingMode() != 0U) &&
            ((ctx->state == CHASSIS_RISING_RUNTIME_STATE_IDLE) ||
         (ctx->state == CHASSIS_RISING_RUNTIME_STATE_FINISHED))) {
        ctx->state = CHASSIS_RISING_RUNTIME_STATE_LIFTING;
        ctx->state_start_tick = now;
    }

    switch (ctx->state) {
        case CHASSIS_RISING_RUNTIME_STATE_LIFTING:
            if ((uint32_t)(now - ctx->state_start_tick) >=
                Chassis_MsToTicks((active_behavior == CHASSIS_RISING_BEHAVIOR_STATE_SingleLift)
                                      ? CHASSIS_RISING_SINGLE_LIFT_DURATION_MS
                                      : CHASSIS_RISING_DOUBLE_LIFT_DURATION_MS)) {
                ctx->state = CHASSIS_RISING_RUNTIME_STATE_TRANSITION;
                ctx->state_start_tick = now;
            }
            break;

        case CHASSIS_RISING_RUNTIME_STATE_TRANSITION:
            if ((uint32_t)(now - ctx->state_start_tick) >=
                Chassis_MsToTicks((active_behavior == CHASSIS_RISING_BEHAVIOR_STATE_SingleLift)
                                      ? CHASSIS_RISING_SINGLE_TRANSITION_DURATION_MS
                                      : CHASSIS_RISING_DOUBLE_TRANSITION_DURATION_MS)) {
                ctx->state = CHASSIS_RISING_RUNTIME_STATE_DRIVING;
                ctx->state_start_tick = now;
            }
            break;

        case CHASSIS_RISING_RUNTIME_STATE_DRIVING:
            if ((uint32_t)(now - ctx->state_start_tick) >=
                Chassis_MsToTicks((active_behavior == CHASSIS_RISING_BEHAVIOR_STATE_SingleLift)
                                      ? CHASSIS_RISING_SINGLE_DRIVE_DURATION_MS
                                      : CHASSIS_RISING_DOUBLE_DRIVE_DURATION_MS)) {
                if (active_behavior == CHASSIS_RISING_BEHAVIOR_STATE_DoubleLift) {
                    ctx->state = CHASSIS_RISING_RUNTIME_STATE_DOUBLE_RISING_HOLD;
                    ctx->state_start_tick = now;
                } else {
                    ctx->state = CHASSIS_RISING_RUNTIME_STATE_FINISHED;
                }
            }
            break;

        case CHASSIS_RISING_RUNTIME_STATE_DOUBLE_RISING_HOLD:
            if ((uint32_t)(now - ctx->state_start_tick) >=
                Chassis_MsToTicks(CHASSIS_RISING_DOUBLE_RISING_HOLD_DURATION_MS)) {
                ctx->state = CHASSIS_RISING_RUNTIME_STATE_DOUBLE_NORMAL_HOLD;
                ctx->state_start_tick = now;
            }
            break;

        case CHASSIS_RISING_RUNTIME_STATE_DOUBLE_NORMAL_HOLD:
            if ((uint32_t)(now - ctx->state_start_tick) >=
                Chassis_MsToTicks(CHASSIS_RISING_DOUBLE_NORMAL_HOLD_DURATION_MS)) {
                ctx->state = CHASSIS_RISING_RUNTIME_STATE_FINISHED;
            }
            break;

        case CHASSIS_RISING_RUNTIME_STATE_FINISHED:
        case CHASSIS_RISING_RUNTIME_STATE_IDLE:
        default:
            break;
    }
}

static void Chassis_ExecuteNormalBySource(const keyboard_t *active_kb)
{
    if (g_chassis_control_source_state == CHASSIS_CONTROL_SOURCE_STATE_Keyboard) {
        /* 键盘源下完全屏蔽遥控器底盘控制，底盘与抬升Normal都不再读取 remoter。 */
        Rising_Normal_Mode(NULL);
        Chassis_Keyboard_Mode(active_kb, 0U);
        return;
    }

    Rising_Normal_Mode(&remoter);
    Chassis_Normal_Mode(&remoter);
}

static void Chassis_ExecuteRisingRegularBySource(const keyboard_t *active_kb)
{
    if (g_chassis_control_source_state == CHASSIS_CONTROL_SOURCE_STATE_Keyboard) {
        rc_info_t rising_rc = {0};

        /* 键盘 rising 下仍然不给遥控器控制权，只构造抬升所需的最小输入。 */
        rising_rc.ch2 = CHASSIS_RISING_KEYBOARD_RC_CH2;
        Rising_Upstairs_Mode(&rising_rc);
        Chassis_Keyboard_Mode(active_kb, 1U);
        return;
    }

    Rising_Upstairs_Mode(&remoter);
    Chassis_Upstairs_Mode(&remoter);
}

static void Chassis_RunSequenceLift(int16_t chassis_ch2, int16_t rising_ch2)
{
    rc_info_t chassis_rc = {0};
    rc_info_t rising_rc = {0};

    chassis_rc.ch2 = chassis_ch2;
    rising_rc.ch2 = rising_ch2;

    Rising_Upstairs_Mode(&rising_rc);
    Chassis_Upstairs_Mode(&chassis_rc);
}

static void Chassis_RunNormalHoldStop(void)
{
    rc_info_t chassis_rc = {0};

    Rising_Normal_Hold_Mode();
    Chassis_Normal_Mode(&chassis_rc);
}

static void Chassis_RunNormalHoldDrive(int16_t chassis_ch2)
{
    rc_info_t chassis_rc = {0};

    chassis_rc.ch2 = chassis_ch2;
    Rising_Normal_Hold_Mode();
    Chassis_Normal_Mode(&chassis_rc);
}

static void Chassis_RunNormalHoldBySource(const keyboard_t *active_kb)
{
    if (g_chassis_control_source_state == CHASSIS_CONTROL_SOURCE_STATE_Keyboard) {
        Rising_Normal_Hold_Mode();
        Chassis_Keyboard_Mode(active_kb, 0U);
        return;
    }

    Rising_Normal_Hold_Mode();
    Chassis_Normal_Mode(&remoter);
}

static void Chassis_ExecuteRisingStateMachine(const keyboard_t *active_kb,
                                              const Chassis_Rising_Runtime_Ctx_t *ctx)
{
    const Chassis_Rising_Behavior_State_t active_behavior = Chassis_GetActiveRisingBehaviorState();

    if ((ctx == NULL) ||
        (ctx->state == CHASSIS_RISING_RUNTIME_STATE_IDLE) ||
        (ctx->state == CHASSIS_RISING_RUNTIME_STATE_FINISHED)) {
        Chassis_ExecuteRisingRegularBySource(active_kb);
        return;
    }

    switch (ctx->state) {
        case CHASSIS_RISING_RUNTIME_STATE_LIFTING:
            if (active_behavior == CHASSIS_RISING_BEHAVIOR_STATE_SingleLift) {
                Chassis_RunSequenceLift(
                    (int16_t)((Remoter_CHMAX * CHASSIS_RISING_SINGLE_LIFT_CHASSIS_SPEED_RATIO_NUM) /
                              CHASSIS_RISING_SINGLE_LIFT_CHASSIS_SPEED_RATIO_DEN),
                    (int16_t)CHASSIS_RISING_SINGLE_LIFT_RISING_RC_CH2);
            } else {
                Chassis_RunSequenceLift(
                    (int16_t)((Remoter_CHMAX * CHASSIS_RISING_DOUBLE_LIFT_CHASSIS_SPEED_RATIO_NUM) /
                              CHASSIS_RISING_DOUBLE_LIFT_CHASSIS_SPEED_RATIO_DEN),
                    (int16_t)CHASSIS_RISING_DOUBLE_LIFT_RISING_RC_CH2);
            }
            break;

        case CHASSIS_RISING_RUNTIME_STATE_TRANSITION:
            Chassis_RunNormalHoldStop();
            break;

        case CHASSIS_RISING_RUNTIME_STATE_DRIVING:
            if (active_behavior == CHASSIS_RISING_BEHAVIOR_STATE_SingleLift) {
                Chassis_RunNormalHoldDrive(
                    (int16_t)((Remoter_CHMAX * CHASSIS_RISING_SINGLE_DRIVE_SPEED_RATIO_NUM) /
                              CHASSIS_RISING_SINGLE_DRIVE_SPEED_RATIO_DEN));
            } else {
                Chassis_RunNormalHoldDrive(
                    (int16_t)((Remoter_CHMAX * CHASSIS_RISING_DOUBLE_DRIVE_SPEED_RATIO_NUM) /
                              CHASSIS_RISING_DOUBLE_DRIVE_SPEED_RATIO_DEN));
            }
            break;

        case CHASSIS_RISING_RUNTIME_STATE_DOUBLE_RISING_HOLD:
        case CHASSIS_RISING_RUNTIME_STATE_DOUBLE_NORMAL_HOLD:
        case CHASSIS_RISING_RUNTIME_STATE_FINISHED:
            Chassis_RunNormalHoldBySource(active_kb);
            break;

        case CHASSIS_RISING_RUNTIME_STATE_IDLE:
        default:
            Chassis_ExecuteRisingRegularBySource(active_kb);
            break;
    }
}

static void Chassis_ExecuteDbusLeftNotFrontRightFrontStateMachine(const keyboard_t *active_kb,
                                                                  const Chassis_Rising_Runtime_Ctx_t *ctx)
{
    const Chassis_Rising_Behavior_State_t active_behavior = Chassis_GetActiveRisingBehaviorState();

    if ((ctx == NULL) ||
        (ctx->state == CHASSIS_RISING_RUNTIME_STATE_IDLE) ||
        (ctx->state == CHASSIS_RISING_RUNTIME_STATE_FINISHED)) {
        Chassis_ExecuteRisingRegularBySource(active_kb);
        return;
    }

    switch (ctx->state) {
        case CHASSIS_RISING_RUNTIME_STATE_LIFTING:
            if (active_behavior == CHASSIS_RISING_BEHAVIOR_STATE_SingleLift) {
                Chassis_RunSequenceLift(
                    (int16_t)((Remoter_CHMAX * CHASSIS_RISING_SINGLE_LIFT_CHASSIS_SPEED_RATIO_NUM) /
                              CHASSIS_RISING_SINGLE_LIFT_CHASSIS_SPEED_RATIO_DEN),
                    (int16_t)CHASSIS_RISING_SINGLE_LIFT_RISING_RC_CH2);
            } else {
                Chassis_RunSequenceLift(
                    (int16_t)((Remoter_CHMAX * CHASSIS_RISING_DOUBLE_LIFT_CHASSIS_SPEED_RATIO_NUM) /
                              CHASSIS_RISING_DOUBLE_LIFT_CHASSIS_SPEED_RATIO_DEN),
                    (int16_t)CHASSIS_RISING_DOUBLE_LIFT_RISING_RC_CH2);
            }
            break;

        case CHASSIS_RISING_RUNTIME_STATE_TRANSITION:
            Chassis_RunNormalHoldStop();
            break;

        case CHASSIS_RISING_RUNTIME_STATE_DRIVING:
            if (active_behavior == CHASSIS_RISING_BEHAVIOR_STATE_SingleLift) {
                Chassis_RunNormalHoldDrive(
                    (int16_t)((Remoter_CHMAX * CHASSIS_RISING_SINGLE_DRIVE_SPEED_RATIO_NUM) /
                              CHASSIS_RISING_SINGLE_DRIVE_SPEED_RATIO_DEN));
            } else {
                Chassis_RunNormalHoldDrive(
                    (int16_t)((Remoter_CHMAX * CHASSIS_RISING_DOUBLE_DRIVE_SPEED_RATIO_NUM) /
                              CHASSIS_RISING_DOUBLE_DRIVE_SPEED_RATIO_DEN));
            }
            break;

        case CHASSIS_RISING_RUNTIME_STATE_DOUBLE_RISING_HOLD:
        case CHASSIS_RISING_RUNTIME_STATE_DOUBLE_NORMAL_HOLD:
        case CHASSIS_RISING_RUNTIME_STATE_FINISHED:
            Chassis_RunNormalHoldBySource(active_kb);
            break;

        case CHASSIS_RISING_RUNTIME_STATE_IDLE:
        default:
            Chassis_ExecuteRisingRegularBySource(active_kb);
            break;
    }
}

void Chassis_Task(void *argument)
{
    Chassis_Rising_Runtime_Ctx_t rising_runtime;
    Chassis_Rising_Runtime_Ctx_t dbus_left_not_front_right_front_runtime;
    Chassis_Mode_State_t last_mode_state;
    Chassis_Rising_Runtime_State_t last_rising_runtime_state;
    uint8_t last_dbus_left_not_front_right_front_active;

    (void)argument;
    osDelay(200);

    Chassis_Drive_Init();
    Rising_Ctrl_Init();

    Chassis_ResetRisingRuntime(&rising_runtime);
    Chassis_ResetRisingRuntime(&dbus_left_not_front_right_front_runtime);
    last_mode_state = g_chassis_mode_state;
    last_rising_runtime_state = rising_runtime.state;
    last_dbus_left_not_front_right_front_active = 0U;

    for (;;) {
        const keyboard_t *active_kb = Referee_GetActiveKeyboard();
        uint8_t dbus_left_not_front_right_front_active;
        Chassis_Rising_Runtime_State_t effective_rising_runtime_state;

        /* 第一层：控制源状态机。
         * DBUS 和 Keyboard 只通过这里统一切换，后续执行逻辑都基于这个状态。
         */
        g_chassis_control_source_state = Chassis_GetControlSourceState();
        Chassis_SanitizeRisingBehaviorState();
        Chassis_SyncModeStateFromSource();
        dbus_left_not_front_right_front_active = Chassis_IsDbusLeftNotFrontRightFront();
        /* 右拨杆在前时：
         * 1. 左拨杆下 -> 四轮都动；
         * 2. 左拨杆中 -> 只动后轮，前轮发送值强制为 0。
         */
        Chassis_SetFrontWheelsOutputBypass(Chassis_IsDbusLeftMiddleRightFront());

        /* 第二层：底盘总状态机。
         * 输出统一发布到 g_chassis_mode_state，供外部读取。
         */
        if (dbus_left_not_front_right_front_active != 0U) {
            Chassis_ResetRisingRuntimeCtxOnly(&rising_runtime);
            Chassis_UpdateDbusLeftNotFrontRightFrontRuntime(&dbus_left_not_front_right_front_runtime);
            effective_rising_runtime_state = dbus_left_not_front_right_front_runtime.state;
        } else {
            Chassis_ResetRisingRuntimeCtxOnly(&dbus_left_not_front_right_front_runtime);
            Chassis_UpdateRisingRuntime(&rising_runtime);
            effective_rising_runtime_state = rising_runtime.state;
        }

        if ((g_chassis_mode_state != last_mode_state) ||
            (effective_rising_runtime_state != last_rising_runtime_state) ||
            (dbus_left_not_front_right_front_active != last_dbus_left_not_front_right_front_active)) {
            Rising_Reset_DmImuPid();
            last_mode_state = g_chassis_mode_state;
            last_rising_runtime_state = effective_rising_runtime_state;
            last_dbus_left_not_front_right_front_active = dbus_left_not_front_right_front_active;
        }

        switch (g_chassis_mode_state) {
            case CHASSIS_MODE_STATE_PowerOff:
                Chassis_ResetRisingRuntime(&rising_runtime);
                Chassis_ResetRisingRuntimeCtxOnly(&dbus_left_not_front_right_front_runtime);
                last_rising_runtime_state = rising_runtime.state;
                Chassis_Stop();
                Rising_Stop();
                break;

            case CHASSIS_MODE_STATE_Normal:
                Chassis_ResetRisingRuntime(&rising_runtime);
                Chassis_ResetRisingRuntimeCtxOnly(&dbus_left_not_front_right_front_runtime);
                last_rising_runtime_state = rising_runtime.state;
                Chassis_ExecuteNormalBySource(active_kb);
                break;

            case CHASSIS_MODE_STATE_Rising:
                /* 第三层：rising 子状态机。
                 * 未启动时走原版 rising；
                 * 按 R 后才进入当前选中的一级/二级流程。
                 */
                if (dbus_left_not_front_right_front_active != 0U) {
                    Chassis_ExecuteDbusLeftNotFrontRightFrontStateMachine(
                        active_kb, &dbus_left_not_front_right_front_runtime);
                } else {
                    Chassis_ExecuteRisingStateMachine(active_kb, &rising_runtime);
                }
                break;

            default:
                Chassis_Stop();
                Rising_Stop();
                break;
        }

        osDelay(2);
    }
}
