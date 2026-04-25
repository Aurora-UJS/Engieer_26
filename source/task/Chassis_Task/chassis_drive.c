#include "chassis_drive.h"
#include "PowerControl.h"
#include "PIDtool.h"
#include "chassis_debug.h"
#include "omni_mecanum_kinematics.h"
#include "referee_api.h"
#include "rising_ctrl.h"
#include "tool.h"
#include <math.h>
#include <string.h>

static DJI_motor_t s_chassis_motor_obj;
static DJI_motor_t *s_chassis_motor = &s_chassis_motor_obj;

static pid_type_def s_chassis_pid[4];
static LowPassFilter s_chassis_lpf[4];

static float32_t s_chassis_target_velocity[4];
static int16_t s_chassis_ctrl_output[4];
static uint16_t s_chassis_power_buffer_energy = 0U;
static float s_chassis_power_virtual_cap_percent = 0.0f;
static float s_chassis_power_effective_limit = Chassis_PowerLimit_UserMax_Default;
static float s_chassis_power_scale_filtered = 1.0f;
static uint8_t s_chassis_front_wheels_output_bypass = 0U;

static void Chassis_PowerAssignHook(float alloc_power);
static void Chassis_PowerControl_Init(void);
static void Chassis_PowerControl_UpdateInputs(void);
static void Chassis_ApplyPowerLimit(void);
static void Chassis_UpdateActualSpeedDebug(void);
static void Chassis_PublishDriveOutput(void);
static uint8_t Chassis_IsFrontWheelIndex(int index);
static uint8_t Chassis_ShouldBypassWheelOutput(int index);
static void Chassis_ApplyWheelOutputBypass(void);
static uint8_t Chassis_IsPowerCalcGroupEnabled(uint8_t group);

static uint8_t Chassis_IsFrontWheelIndex(int index)
{
    return ((index == Chassis_Motor_3508_ZQ) ||
            (index == Chassis_Motor_3508_YQ)) ? 1U : 0U;
}

static uint8_t Chassis_ShouldBypassWheelOutput(int index)
{
    return ((s_chassis_front_wheels_output_bypass != 0U) &&
            (Chassis_IsFrontWheelIndex(index) != 0U)) ? 1U : 0U;
}

static void Chassis_ApplyWheelOutputBypass(void)
{
    if (s_chassis_front_wheels_output_bypass == 0U) {
        return;
    }

    s_chassis_ctrl_output[Chassis_Motor_3508_ZQ] = 0;
    s_chassis_ctrl_output[Chassis_Motor_3508_YQ] = 0;
}

static uint8_t Chassis_IsPowerCalcGroupEnabled(uint8_t group)
{
    if (group >= Chassis_PowerCalc_Group_Count) {
        return 0U;
    }

    return (g_chassis_debug.chassis_power_calc_enable[group] != 0U) ? 1U : 0U;
}

static float Chassis_GetMotorPowerDemand(float ctrl_output,
                                         float motor_speed,
                                         const MotorPowerParams_t *params)
{
    float global_scale = g_chassis_debug.chassis_power_model_global_scale;

    if (global_scale <= 0.0f) {
        global_scale = Chassis_PowerModel_GlobalScale_Default;
        g_chassis_debug.chassis_power_model_global_scale = global_scale;
    }

    return MotorPower_CalculateSingle(fabsf(ctrl_output), fabsf(motor_speed), params) * global_scale;
}

static void Chassis_PowerAssignHook(float alloc_power)
{
    MotorPowerParams_t power_params;
    DJI_motor_t *rising_motor = Rising_Get3508Motor();
    int16_t *rising_ctrl_output = Rising_Get3508CtrlOutput();
    float total_estimated_power = 0.0f;
    float limited_total_estimated_power = 0.0f;
    float power_scale = 1.0f;
    float attack = g_chassis_debug.chassis_power_scale_attack;
    float release = g_chassis_debug.chassis_power_scale_release;
    const uint8_t chassis_power_calc_enable = Chassis_IsPowerCalcGroupEnabled(Chassis_PowerCalc_Group_Chassis);
    const uint8_t rising_power_calc_enable = Chassis_IsPowerCalcGroupEnabled(Chassis_PowerCalc_Group_Rising);

    if (s_chassis_motor == NULL) {
        return;
    }

    power_params.torque_coeff = g_chassis_debug.chassis_power_model_torque_coeff;
    power_params.k1 = g_chassis_debug.chassis_power_model_k1;
    power_params.k2 = g_chassis_debug.chassis_power_model_k2;
    power_params.k3 = g_chassis_debug.chassis_power_model_k3;

    if (power_params.torque_coeff <= 0.0f) {
        power_params.torque_coeff = Chassis_PowerModel_TorqueCoeff_Default;
        g_chassis_debug.chassis_power_model_torque_coeff = power_params.torque_coeff;
    }
    if (power_params.k1 < 0.0f) {
        power_params.k1 = Chassis_PowerModel_K1_Default;
        g_chassis_debug.chassis_power_model_k1 = power_params.k1;
    }
    if (power_params.k2 < 0.0f) {
        power_params.k2 = Chassis_PowerModel_K2_Default;
        g_chassis_debug.chassis_power_model_k2 = power_params.k2;
    }
    if (power_params.k3 < 0.0f) {
        power_params.k3 = Chassis_PowerModel_K3_Default;
        g_chassis_debug.chassis_power_model_k3 = power_params.k3;
    }

    if (alloc_power < 0.0f) {
        alloc_power = 0.0f;
    }
    g_chassis_debug.chassis_power_alloc_limit = alloc_power;

    if (attack < 0.0f) {
        attack = 0.0f;
        g_chassis_debug.chassis_power_scale_attack = attack;
    } else if (attack > 1.0f) {
        attack = 1.0f;
        g_chassis_debug.chassis_power_scale_attack = attack;
    }

    if (release < 0.0f) {
        release = 0.0f;
        g_chassis_debug.chassis_power_scale_release = release;
    } else if (release > 1.0f) {
        release = 1.0f;
        g_chassis_debug.chassis_power_scale_release = release;
    }

    for (int i = 0; i < 4; i++) {
        if ((chassis_power_calc_enable == 0U) || (Chassis_ShouldBypassWheelOutput(i) != 0U)) {
            g_chassis_debug.chassis_power_motor_estimate_3508[i] = 0.0f;
            continue;
        }

        const float motor_speed = s_chassis_motor->motor_msg[i].motor_speed;
        const float estimated_power =
            Chassis_GetMotorPowerDemand((float)s_chassis_ctrl_output[i], motor_speed, &power_params);

        g_chassis_debug.chassis_power_motor_estimate_3508[i] = estimated_power;
        total_estimated_power += estimated_power;
    }

    for (int i = 0; i < 2; i++) {
        float estimated_power = 0.0f;

        if ((rising_power_calc_enable != 0U) &&
            (rising_motor != NULL) &&
            (rising_ctrl_output != NULL)) {
            estimated_power = Chassis_GetMotorPowerDemand(
                (float)rising_ctrl_output[i], rising_motor->motor_msg[i].motor_speed, &power_params);
        }

        g_chassis_debug.rising_power_motor_estimate_3508[i] = estimated_power;
        total_estimated_power += estimated_power;
    }

    if ((total_estimated_power > alloc_power) && (total_estimated_power > 1.0e-6f)) {
        power_scale = alloc_power / total_estimated_power;
        if (power_scale < 0.0f) {
            power_scale = 0.0f;
        }
    }

    if (power_scale < s_chassis_power_scale_filtered) {
        s_chassis_power_scale_filtered += attack * (power_scale - s_chassis_power_scale_filtered);
    } else {
        s_chassis_power_scale_filtered += release * (power_scale - s_chassis_power_scale_filtered);
    }

    if (s_chassis_power_scale_filtered > 1.0f) {
        s_chassis_power_scale_filtered = 1.0f;
    } else if (s_chassis_power_scale_filtered < 0.0f) {
        s_chassis_power_scale_filtered = 0.0f;
    }

    for (int i = 0; i < 4; i++) {
        if (Chassis_ShouldBypassWheelOutput(i) != 0U) {
            s_chassis_ctrl_output[i] = 0;
            continue;
        }

        if (chassis_power_calc_enable != 0U) {
            float limited_output = (float)s_chassis_ctrl_output[i] * s_chassis_power_scale_filtered;
            LimitMax(limited_output, Chassis_3508_PID_Maxout);
            s_chassis_ctrl_output[i] = (int16_t)lroundf(limited_output);
        }
    }

    for (int i = 0; i < 2; i++) {
        if (rising_ctrl_output == NULL) {
            break;
        }

        if (rising_power_calc_enable != 0U) {
            float limited_output = (float)rising_ctrl_output[i] * s_chassis_power_scale_filtered;
            LimitMax(limited_output, Rising_3508_PID_Maxout);
            rising_ctrl_output[i] = (int16_t)lroundf(limited_output);
        }
    }

    for (int i = 0; i < 4; i++) {
        if ((chassis_power_calc_enable == 0U) || (Chassis_ShouldBypassWheelOutput(i) != 0U)) {
            g_chassis_debug.chassis_power_motor_limited_estimate_3508[i] = 0.0f;
            continue;
        }

        const float limited_power = Chassis_GetMotorPowerDemand(
            (float)s_chassis_ctrl_output[i], s_chassis_motor->motor_msg[i].motor_speed, &power_params);
        g_chassis_debug.chassis_power_motor_limited_estimate_3508[i] = limited_power;
        limited_total_estimated_power += limited_power;
    }

    for (int i = 0; i < 2; i++) {
        float limited_power = 0.0f;

        if ((rising_power_calc_enable != 0U) &&
            (rising_motor != NULL) &&
            (rising_ctrl_output != NULL)) {
            limited_power = Chassis_GetMotorPowerDemand(
                (float)rising_ctrl_output[i], rising_motor->motor_msg[i].motor_speed, &power_params);
        }

        g_chassis_debug.rising_power_motor_limited_estimate_3508[i] = limited_power;
        limited_total_estimated_power += limited_power;
    }

    g_chassis_debug.chassis_power_total_estimate = total_estimated_power;
    g_chassis_debug.chassis_power_total_limited_estimate = limited_total_estimated_power;
    g_chassis_debug.chassis_power_scale = s_chassis_power_scale_filtered;
}

static void Chassis_PowerControl_Init(void)
{
    pid_type_def chassis_power_pid;

    PID_Init(&chassis_power_pid, 0.0f, 0.0f, 0.0f, 200.0f, 200.0f);
    PowerControl_Init(&s_chassis_power_buffer_energy,
                      &s_chassis_power_virtual_cap_percent,
                      &s_chassis_power_effective_limit,
                      &chassis_power_pid,
                      Chassis_PowerAssignHook);
    Set_PowerControlMode(POWER_LOSS);
}

static void Chassis_PowerControl_UpdateInputs(void)
{
    referee_info_t *referee = get_referee_msg();
    float referee_limit = 0.0f;
    float user_limit = g_chassis_debug.chassis_power_limit_user_max;

    if (user_limit <= 0.0f) {
        user_limit = Chassis_PowerLimit_UserMax_Default;
        g_chassis_debug.chassis_power_limit_user_max = user_limit;
    }

    if (referee != NULL) {
        s_chassis_power_buffer_energy = referee->PowerHeatData.buffer_energy;
        referee_limit = (float)referee->GameRobotState.chassis_power_limit;
        g_chassis_debug.chassis_power_referee_actual = referee->PowerHeatData.chassis_power;
    } else {
        s_chassis_power_buffer_energy = 0U;
        g_chassis_debug.chassis_power_referee_actual = 0.0f;
    }

    if (referee_limit > 0.0f) {
        s_chassis_power_effective_limit = (user_limit < referee_limit) ? user_limit : referee_limit;
    } else {
        s_chassis_power_effective_limit = user_limit;
    }

    if (s_chassis_power_effective_limit < 1.0f) {
        s_chassis_power_effective_limit = 1.0f;
    }

    g_chassis_debug.chassis_power_referee_limit = referee_limit;
    g_chassis_debug.chassis_power_buffer_energy = (float32_t)s_chassis_power_buffer_energy;
    g_chassis_debug.chassis_power_limit_effective_max = s_chassis_power_effective_limit;
}

static void Chassis_ApplyPowerLimit(void)
{
    Chassis_PowerControl_UpdateInputs();

    if (g_chassis_debug.chassis_power_limit_enable != 0U) {
        PowerControl_Update();
        return;
    }

    Chassis_PowerAssignHook(1.0e9f);
    g_chassis_debug.chassis_power_alloc_limit = g_chassis_debug.chassis_power_limit_effective_max;
    g_chassis_debug.chassis_power_scale = 1.0f;
}

static void Chassis_UpdateActualSpeedDebug(void)
{
    if (s_chassis_motor == NULL) {
        return;
    }

    for (int i = 0; i < 4; i++) {
        g_chassis_debug.chassis_actual_speed_3508[i] =
            s_chassis_motor->motor_msg[i].motor_speed * (Motor_Wheel_Trans);
    }
}

static void Chassis_PublishDriveOutput(void)
{
    Chassis_ApplyWheelOutputBypass();

    for (int i = 0; i < 4; i++) {
        g_chassis_debug.chassis_output_raw_3508[i] = (float32_t)s_chassis_ctrl_output[i];
    }

    Chassis_ApplyPowerLimit();
    Chassis_ApplyWheelOutputBypass();

    for (int i = 0; i < 4; i++) {
        g_chassis_debug.chassis_output_3508[i] = (float32_t)s_chassis_ctrl_output[i];
    }

    Chassis_Motor_SendControl_DJI(s_chassis_motor, s_chassis_ctrl_output);
    Rising_Publish3508Output();
    Chassis_UpdateActualSpeedDebug();
}

/**
 * @brief 初始化底盘轮系控制模块
 */
void Chassis_Drive_Init(void)
{
    Chassis_Wheel_LPF_Init(s_chassis_lpf, 0.8f);
    Chassis_Wheel_Init_DJI(&s_chassis_motor);
    Chassis_3508_PID_Init(s_chassis_pid);
    Chassis_PowerControl_Init();
    Chassis_Stop();
}

/**
 * @brief 关闭底盘轮电机输出（轮子停转）
 */
void Chassis_Stop(void)
{
    for (int i = 0; i < 4; i++) {
        s_chassis_ctrl_output[i] = 0;
        g_chassis_debug.chassis_target_speed_3508[i] = 0.0f;
        g_chassis_debug.chassis_output_raw_3508[i] = 0.0f;
        g_chassis_debug.chassis_output_3508[i] = 0.0f;
        g_chassis_debug.chassis_power_motor_estimate_3508[i] = 0.0f;
        g_chassis_debug.chassis_power_motor_limited_estimate_3508[i] = 0.0f;
    }
    g_chassis_debug.chassis_power_total_estimate = 0.0f;
    g_chassis_debug.chassis_power_total_limited_estimate = 0.0f;
    g_chassis_debug.rising_power_motor_estimate_3508[0] = 0.0f;
    g_chassis_debug.rising_power_motor_estimate_3508[1] = 0.0f;
    g_chassis_debug.rising_power_motor_limited_estimate_3508[0] = 0.0f;
    g_chassis_debug.rising_power_motor_limited_estimate_3508[1] = 0.0f;
    g_chassis_debug.chassis_power_scale = 1.0f;
    s_chassis_power_scale_filtered = 1.0f;

    if (s_chassis_motor != NULL) {
        Chassis_Motor_SendControl_DJI(s_chassis_motor, s_chassis_ctrl_output);
        Chassis_UpdateActualSpeedDebug();
    }
}

void Chassis_SetFrontWheelsOutputBypass(uint8_t enable)
{
    s_chassis_front_wheels_output_bypass = (enable != 0U) ? 1U : 0U;

    if (s_chassis_front_wheels_output_bypass != 0U) {
        s_chassis_ctrl_output[Chassis_Motor_3508_ZQ] = 0;
        s_chassis_ctrl_output[Chassis_Motor_3508_YQ] = 0;
    }
}

/**
 * @brief 底盘普通模式控制
 *
 * @param remoter 遥控器数据指针
 */
void Chassis_Normal_Mode(const rc_info_t *remoter)
{
    if (remoter == NULL || s_chassis_motor == NULL) {
        return;
    }

    Chassis_Motor_TargetVelocity(s_chassis_target_velocity, *remoter);

    for (int i = 0; i < 4; i++) {
        g_chassis_debug.chassis_target_speed_3508[i] = s_chassis_target_velocity[i];
    }

    Chassis_3508_PID_Calculate(s_chassis_pid,
                              s_chassis_target_velocity,
                              s_chassis_motor,
                              s_chassis_ctrl_output,
                              s_chassis_lpf);
    Chassis_PublishDriveOutput();
}

/**
 * @brief 底盘上岛模式控制
 *
 * @param remoter 遥控器数据指针
 */
void Chassis_Upstairs_Mode(const rc_info_t *remoter)
{
    if (remoter == NULL || s_chassis_motor == NULL) {
        return;
    }

    rc_info_t tmp = *remoter;
    tmp.ch3 = 0;

    Chassis_Motor_TargetVelocity(s_chassis_target_velocity, tmp);

    for (int i = 0; i < 4; i++) {
        g_chassis_debug.chassis_target_speed_3508[i] = s_chassis_target_velocity[i];
    }

    Chassis_3508_PID_Calculate(s_chassis_pid,
                              s_chassis_target_velocity,
                              s_chassis_motor,
                              s_chassis_ctrl_output,
                              s_chassis_lpf);
    Chassis_PublishDriveOutput();
}

/**
 * @brief 键盘模式底盘控制
 *
 * 功能说明：
 * - WASD键控制底盘平移（前后左右）
 * - 鼠标X轴控制底盘旋转（yaw角速度）
 * - 支持禁用旋转功能（用于抬升模式）
 *
 * 控制映射：
 * - W键：前进（motion.y = +Max_Velocity）
 * - S键：后退（motion.y = -Max_Velocity）
 * - A键：左移（motion.x = -Max_Velocity）
 * - D键：右移（motion.x = +Max_Velocity）
 * - Shift + WASD：平移速度降为 30%
 * - 鼠标X轴：旋转（motion.wz，带死区100和限幅±660）
 *
 * @param kb 键盘/鼠标数据指针（可来自裁判系统或遥控器DBUS）
 * @param disable_yaw 为1时禁用yaw旋转（wz=0），用于抬升Rising模式
 */
void Chassis_Keyboard_Mode(const keyboard_t *kb, uint8_t disable_yaw)
{
    if (kb == NULL || s_chassis_motor == NULL) {
        return;
    }

    basic_vector_t motion;
    const float32_t translation_speed =
        ((kb->key_code.bit.SHIFT != 0U) ? Chassis_Keyboard_Shift_Speed_Ratio : 1.0f) *
        (float32_t)Max_Velocity;
    motion.x = 0.0f;
    motion.y = 0.0f;
    motion.wz = 0.0f;

    /* ===== WASD键盘平移控制 =====
     * 长按直接给最大速度（不做跳变检测）
     * W/S控制前后（y轴），A/D控制左右（x轴）
     */
    if (kb->key_code.bit.W != 0U) {
        motion.x = translation_speed;  // 前进
    } else if (kb->key_code.bit.S != 0U) {
        motion.x = -translation_speed;  // 后退
    }

    if (kb->key_code.bit.A != 0U) {
        motion.y = -translation_speed;  // 左移
    } else if (kb->key_code.bit.D != 0U) {
        motion.y = translation_speed;  // 右移
    }

    /* ===== 鼠标X轴旋转控制 =====
     * 映射为yaw角速度（wz），带死区和限幅
     * 仅在disable_yaw=0时生效
     */
    if (disable_yaw == 0U) {
        int16_t mx = kb->mouse_x;

        /* 死区处理：宏定义可调，避免微小抖动 */
        int16_t abs_mx = (mx >= 0) ? mx : (int16_t)(-mx);
        if (abs_mx <= Chassis_Keyboard_MouseYaw_Deadzone) {
            motion.wz = 0.0f;
        } else {
            /* 限幅到[-660, 660]，防止过大输入 */
            if (mx > Remoter_CHMAX) {
                mx = Remoter_CHMAX;
            } else if (mx < -Remoter_CHMAX) {
                mx = -Remoter_CHMAX;
            }
            /* 鼠标 X 轴通过可调极性、死区和灵敏度映射到底盘旋转速度。 */
            motion.wz = Chassis_Keyboard_MouseYaw_Polarity * map((float32_t)mx,
                                                                 -(float32_t)Remoter_CHMAX,
                                                                 (float32_t)Remoter_CHMAX,
                                                                 -(float32_t)(Max_Velocity * Chassis_Keyboard_MouseYaw_Sensitivity),
                                                                 (float32_t)(Max_Velocity * Chassis_Keyboard_MouseYaw_Sensitivity));
        }
    }

    omni_mecanum_kinematics(&motion, s_chassis_target_velocity);

    for (int i = 0; i < 4; i++) {
        g_chassis_debug.chassis_target_speed_3508[i] = s_chassis_target_velocity[i];
    }

    Chassis_3508_PID_Calculate(s_chassis_pid,
                              s_chassis_target_velocity,
                              s_chassis_motor,
                              s_chassis_ctrl_output,
                              s_chassis_lpf);
    Chassis_PublishDriveOutput();
}

void Chassis_Wheel_LPF_Init(LowPassFilter lpf[4], float alpha)
{
    for (int i = 0; i < 4; i++) {
        lizeFilter_init(&lpf[i], alpha);
    }
}

void Chassis_Wheel_Init_DJI(DJI_motor_t **Chassis_Motor)
{
    if (Chassis_Motor == NULL || *Chassis_Motor == NULL) {
        return;
    }

    memset(*Chassis_Motor, 0, sizeof(DJI_motor_t));
    (*Chassis_Motor)->can_cfg.port = CAN1_PORT;
    (*Chassis_Motor)->can_cfg.id = Chassis_Motor_ALL_id;

    (*Chassis_Motor)->motor_msg[Chassis_Motor_3508_ZQ].can_msg.id = Chassis_Motor_3508_ZQ_id;
    (*Chassis_Motor)->motor_msg[Chassis_Motor_3508_ZH].can_msg.id = Chassis_Motor_3508_ZH_id;
    (*Chassis_Motor)->motor_msg[Chassis_Motor_3508_YH].can_msg.id = Chassis_Motor_3508_YH_id;
    (*Chassis_Motor)->motor_msg[Chassis_Motor_3508_YQ].can_msg.id = Chassis_Motor_3508_YQ_id;

    Motor_DJI_Init(*Chassis_Motor);
}

void Chassis_Motor_SendControl_DJI(DJI_motor_t *DJMotor, int16_t output[])
{
    Motor_DJI_Refresh(DJMotor);
    set_motor_parameter(
        DJMotor,
        output[Chassis_Motor_3508_ZQ],
        output[Chassis_Motor_3508_ZH],
        output[Chassis_Motor_3508_YH],
        output[Chassis_Motor_3508_YQ]);
}

void Chassis_Motor_TargetVelocity(float32_t Target_Velocity[], rc_info_t remoter)
{
    basic_vector_t motion;

    motion.x = map(remoter.ch2,
                   -Remoter_CHMAX,
                   Remoter_CHMAX,
                   -Max_Velocity,
                   Max_Velocity);

    motion.y = map(remoter.ch1,
                   -Remoter_CHMAX,
                   Remoter_CHMAX,
                   -Max_Velocity,
                   Max_Velocity);

    motion.wz = Turning_Forward_Feedback * map(remoter.ch3,
                                              -Remoter_CHMAX,
                                              Remoter_CHMAX,
                                              -Max_Velocity,
                                              Max_Velocity);

    omni_mecanum_kinematics(&motion, Target_Velocity);
}

void Chassis_3508_PID_Init(pid_type_def pid[])
{
    for (int i = 0; i < 4; i++) {
        PID_Init(pid + i,
                 Chassis_3508_PID_kp,
                 Chassis_3508_PID_ki,
                 Chassis_3508_PID_kd,
                 Chassis_3508_PID_Maxout,
                 Chassis_3508_PID_Maxiout);
    }
}

void Chassis_3508_PID_Calculate(pid_type_def pid[], float32_t target_speed[],
                               DJI_motor_t *motor, int16_t output[], LowPassFilter lpf[])
{
    float32_t curren_wheel_speed[4];

    for (int i = 0; i < 4; i++) {
        curren_wheel_speed[i] = motor->motor_msg[i].motor_speed * (Motor_Wheel_Trans);
        output[i] = (int16_t)(PID_Calc_Pos(pid + i, curren_wheel_speed[i], *(target_speed + i)));
        output[i] = (int16_t)filterValue(&lpf[i], output[i]);
    }
}
