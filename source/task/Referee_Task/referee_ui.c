#include "referee_ui.h"

#include "Chassis_Task.h"
#include "Referee_Task.h"
#include "arm_state_machine.h"
#include "referee_api.h"
#include "referee_protocol.h"
#include "ui_store01.h"

#include <stdio.h>
#include <string.h>

#define REFEREE_UI_UPDATE_PERIOD_MS 40U
#define REFEREE_UI_CLEAR_SETTLE_MS 500U
#define REFEREE_UI_ACTIVE_COLOR UI_Color_Green
#define REFEREE_UI_INACTIVE_COLOR UI_Color_Main

static uint8_t s_referee_ui_ready = 0U;
static uint8_t s_referee_ui_cleared = 0U;
static uint8_t s_referee_ui_store01_inited = 0U;
static uint32_t s_referee_ui_last_update_tick = 0U;

volatile referee_ui_debug_t g_referee_ui_debug = {
    .last_send_status = HAL_OK,
};

static uint32_t Referee_UI_MsToTicks(uint32_t duration_ms)
{
    return (uint32_t)((duration_ms * osKernelGetTickFreq()) / 1000U);
}

static void Referee_UI_SendHook(const uint8_t *message, uint16_t length)
{
    HAL_StatusTypeDef status;

    /* 这里发送的是 0x0301 机器人交互/UI 帧，接收对象是官方选手端，不是自定义客户端。 */
    status = referee_send_raw_data(message, length, 20U);
    g_referee_ui_debug.last_send_status = status;
    g_referee_ui_debug.last_send_len = length;
    if (status == HAL_OK) {
        g_referee_ui_debug.send_count++;
        g_referee_ui_debug.send_bytes += length;
    } else {
        g_referee_ui_debug.send_fail_count++;
    }
}

static void Referee_UI_SetString(ui_interface_string_t *target, const char *text)
{
    if ((target == NULL) || (text == NULL)) {
        return;
    }

    memset(target->string, 0, sizeof(target->string));
    strncpy(target->string, text, sizeof(target->string) - 1U);
    target->string[sizeof(target->string) - 1U] = '\0';
    target->str_length = (uint32_t)strlen(target->string);
    if (target->operate_type != UI_Graph_ADD) {
        target->operate_type = UI_Graph_Change;
    }
}

static const char *Referee_UI_GetChassisModeText(Chassis_Mode_State_t mode_state)
{
    switch (mode_state) {
        case CHASSIS_MODE_STATE_PowerOff:
            return "PowerOff";

        case CHASSIS_MODE_STATE_Normal:
            return "Normal";

        case CHASSIS_MODE_STATE_Rising:
            return "Rising";

        default:
            return "Unknown";
    }
}

static const char *Referee_UI_GetArmModeText(arm_control_mode_t mode)
{
    switch (mode) {
        case Arm_IDLE_Mode:
            return "Idle";

        case Arm_Custom_Controller_Follow_Mode:
            return "Custom";

        case Arm_Frozen_Mode:
            return "Frozen";

        case Arm_Set_Radian:
            return "SetRad";

        case Arm_Transition_Mode:
            return "Trans";

        case Arm_Traj_Mode:
            return "Traj";

        case Arm_Rising_Mode:
            return "Rising";

        case Arm_Zero_Mode:
            return "Zero";

        default:
            return "Unknown";
    }
}

static const char *Referee_UI_GetRisingBehaviorText(Chassis_Rising_Behavior_State_t behavior_state)
{
    switch (behavior_state) {
        case CHASSIS_RISING_BEHAVIOR_STATE_SingleLift:
            return "Single";

        case CHASSIS_RISING_BEHAVIOR_STATE_DoubleLift:
            return "Double";

        default:
            return "Unknown";
    }
}

static const char *Referee_UI_GetControlSourceText(Chassis_Control_Source_State_t source_state)
{
    switch (source_state) {
        case CHASSIS_CONTROL_SOURCE_STATE_DBUS:
            return "DBUS";

        case CHASSIS_CONTROL_SOURCE_STATE_Keyboard:
            return "Keyboard";

        default:
            return "Unknown";
    }
}

static void Referee_UI_UpdateIndicators(Chassis_Mode_State_t chassis_mode,
                                        arm_control_mode_t arm_mode,
                                        Chassis_Rising_Behavior_State_t rising_behavior,
                                        Chassis_Control_Source_State_t control_source)
{
    /* 4 个圆点分别用于提示：
     * 1. 底盘是否在 Rising 总模式；
     * 2. 机械臂是否在 Rising 模式；
     * 3. 当前选择的是一级还是二级抬升；
     * 4. 当前控制原是 DBUS 还是键盘。
     */
    ui_store01_Ungroup_store00->color =
        (chassis_mode == CHASSIS_MODE_STATE_Rising) ? REFEREE_UI_ACTIVE_COLOR : REFEREE_UI_INACTIVE_COLOR;
    ui_store01_Ungroup_store03->color =
        (arm_mode == Arm_Rising_Mode) ? REFEREE_UI_ACTIVE_COLOR : REFEREE_UI_INACTIVE_COLOR;
    ui_store01_Ungroup_store01->color =
        (rising_behavior == CHASSIS_RISING_BEHAVIOR_STATE_DoubleLift) ? UI_Color_Orange : UI_Color_Cyan;
    ui_store01_Ungroup_store02->color =
        (control_source == CHASSIS_CONTROL_SOURCE_STATE_Keyboard) ? UI_Color_Yellow : UI_Color_White;
}

static void Referee_UI_UpdateStore01Content(uint16_t robot_id)
{
    const Chassis_Mode_State_t chassis_mode = Chassis_GetModeState();
    const Chassis_Control_Source_State_t control_source = Chassis_GetControlSourceStatePublic();
    const Chassis_Rising_Behavior_State_t rising_behavior = Chassis_GetRisingBehaviorState();
    char chassis_text[30];
    char arm_text[30];
    char rising_text[30];
    char control_text[30];

    memset(chassis_text, 0, sizeof(chassis_text));
    memset(arm_text, 0, sizeof(arm_text));
    memset(rising_text, 0, sizeof(rising_text));
    memset(control_text, 0, sizeof(control_text));

    snprintf(chassis_text,
             sizeof(chassis_text),
             "%s",
             Referee_UI_GetChassisModeText(chassis_mode));
    snprintf(arm_text,
             sizeof(arm_text),
             "%s",
             Referee_UI_GetArmModeText(Arm_Current_Control_Mode));
    snprintf(rising_text,
             sizeof(rising_text),
             "%s",
             Referee_UI_GetRisingBehaviorText(rising_behavior));
    snprintf(control_text,
             sizeof(control_text),
             "%s %u",
             Referee_UI_GetControlSourceText(control_source),
             (unsigned int)robot_id);

    Referee_UI_SetString(ui_store01_Ungroup_Chas_disp, chassis_text);
    Referee_UI_SetString(ui_store01_Ungroup_arm_disp, arm_text);
    Referee_UI_SetString(ui_store01_Ungroup_risg_disp, rising_text);
    Referee_UI_SetString(ui_store01_Ungroup_ctrl_disp, control_text);

    Referee_UI_UpdateIndicators(chassis_mode,
                                Arm_Current_Control_Mode,
                                rising_behavior,
                                control_source);
}

void Referee_UI_Init(void)
{
    ui_set_send_hook(Referee_UI_SendHook);

    s_referee_ui_ready = 1U;
    s_referee_ui_cleared = 0U;
    s_referee_ui_store01_inited = 0U;
    s_referee_ui_last_update_tick = 0U;
}

void Referee_UI_RequestRefresh(void)
{
    if (s_referee_ui_ready == 0U) {
        return;
    }

    s_referee_ui_cleared = 0U;
    s_referee_ui_store01_inited = 0U;
    s_referee_ui_last_update_tick = 0U;
}

void Referee_UI_Service(void)
{
    const referee_info_t *referee = get_referee_msg();
    const uint32_t now = osKernelGetTickCount();
    const uint32_t update_period_ticks = Referee_UI_MsToTicks(REFEREE_UI_UPDATE_PERIOD_MS);
    uint16_t robot_id = 0U;

    g_referee_ui_debug.service_count++;

    if ((s_referee_ui_ready == 0U) || (referee_is_inited() == 0U) || (referee == NULL)) {
        return;
    }

    robot_id = referee->GameRobotState.robot_id;
    g_referee_ui_debug.last_robot_id = robot_id;
    if (robot_id == 0U) {
        return;
    }

    /* 裁判系统的 UI 接收者是当前机器人对应的操作手客户端，client_id = robot_id + 0x0100。 */
    ui_self_id = robot_id;

    if (s_referee_ui_cleared == 0U) {
        ui_delete_layer(UI_Data_Del_ALL, 0U);
        s_referee_ui_cleared = 1U;
        s_referee_ui_last_update_tick = now;
        return;
    }

    if (s_referee_ui_store01_inited == 0U) {
        if ((s_referee_ui_last_update_tick == 0U) ||
            ((uint32_t)(now - s_referee_ui_last_update_tick) < Referee_UI_MsToTicks(REFEREE_UI_CLEAR_SETTLE_MS))) {
            return;
        }
        ui_init_store01();
        Referee_UI_UpdateStore01Content(robot_id);
        ui_update_store01();
        s_referee_ui_store01_inited = 1U;
        s_referee_ui_last_update_tick = now;
        return;
    }

    if ((s_referee_ui_last_update_tick == 0U) ||
        ((uint32_t)(now - s_referee_ui_last_update_tick) >= update_period_ticks)) {
        Referee_UI_UpdateStore01Content(robot_id);
        ui_update_store01();
        s_referee_ui_last_update_tick = now;
    }
}
