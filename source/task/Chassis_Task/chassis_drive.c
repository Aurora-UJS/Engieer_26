 #include "chassis_drive.h"
#include "PIDtool.h"
 #include "chassis_debug.h"
 #include "omni_mecanum_kinematics.h"
 #include "tool.h"
 #include <string.h>

static DJI_motor_t s_chassis_motor_obj;
static DJI_motor_t *s_chassis_motor = &s_chassis_motor_obj;

static pid_type_def s_chassis_pid[4];
static LowPassFilter s_chassis_lpf[4];

static float32_t s_chassis_target_velocity[4];
static int16_t s_chassis_ctrl_output[4];

/**
 * @brief 初始化底盘轮系控制模块
 */
void Chassis_Drive_Init(void)
{
    Chassis_Wheel_LPF_Init(s_chassis_lpf, 0.8f);
    Chassis_Wheel_Init_DJI(&s_chassis_motor);
    Chassis_3508_PID_Init(s_chassis_pid);
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
        g_chassis_debug.chassis_output_3508[i] = 0.0f;
    }

    if (s_chassis_motor != NULL) {
        Chassis_Motor_SendControl_DJI(s_chassis_motor, s_chassis_ctrl_output);

        for (int i = 0; i < 4; i++) {
            g_chassis_debug.chassis_actual_speed_3508[i] = s_chassis_motor->motor_msg[i].motor_speed * (Motor_Wheel_Trans);
        }
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

    for (int i = 0; i < 4; i++) {
        g_chassis_debug.chassis_output_3508[i] = (float32_t)s_chassis_ctrl_output[i];
    }
    Chassis_Motor_SendControl_DJI(s_chassis_motor, s_chassis_ctrl_output);

    for (int i = 0; i < 4; i++) {
        g_chassis_debug.chassis_actual_speed_3508[i] = s_chassis_motor->motor_msg[i].motor_speed * (Motor_Wheel_Trans);
    }
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

    for (int i = 0; i < 4; i++) {
        g_chassis_debug.chassis_output_3508[i] = (float32_t)s_chassis_ctrl_output[i];
    }
    Chassis_Motor_SendControl_DJI(s_chassis_motor, s_chassis_ctrl_output);

    for (int i = 0; i < 4; i++) {
        g_chassis_debug.chassis_actual_speed_3508[i] = s_chassis_motor->motor_msg[i].motor_speed * (Motor_Wheel_Trans);
    }
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
 * - 鼠标X轴：旋转（motion.wz，带死区100和限幅±660）
 *
 * @param kb 键盘/鼠标数据指针（可来自裁判系统或遥控器DBUS）
 * @param disable_yaw 为1时禁用yaw旋转（wz=0），用于抬升Rising模式
 */
/* 调试变量：在debugger中观察这些值来定位键盘模式问题 */
volatile uint16_t dbg_kb_keycode = 0;      // 读到的键盘码（非0=有按键）
volatile float    dbg_kb_motion_x = 0.0f;  // 运动学输入x
volatile float    dbg_kb_motion_y = 0.0f;  // 运动学输入y
volatile uint8_t  dbg_kb_entered = 0;      // 函数是否被进入（每次+1）

void Chassis_Keyboard_Mode(const keyboard_t *kb, uint8_t disable_yaw)
{
    if (kb == NULL || s_chassis_motor == NULL) {
        return;
    }

    dbg_kb_entered++;
    dbg_kb_keycode = kb->key_code.key_code;

    basic_vector_t motion;
    motion.x = 0.0f;
    motion.y = 0.0f;
    motion.wz = 0.0f;

    /* ===== WASD键盘平移控制 =====
     * 长按直接给最大速度（不做跳变检测）
     * W/S控制前后（y轴），A/D控制左右（x轴）
     */
    if (kb->key_code.bit.W != 0U) {
        motion.x = (float32_t)Max_Velocity;  // 前进
    } else if (kb->key_code.bit.S != 0U) {
        motion.x = -(float32_t)Max_Velocity;  // 后退
    }

    if (kb->key_code.bit.A != 0U) {
        motion.y = -(float32_t)Max_Velocity;  // 左移
    } else if (kb->key_code.bit.D != 0U) {
        motion.y = (float32_t)Max_Velocity;  // 右移
    }

    dbg_kb_motion_x = motion.x;
    dbg_kb_motion_y = motion.y;

    /* ===== 鼠标X轴旋转控制 =====
     * 映射为yaw角速度（wz），带死区和限幅
     * 仅在disable_yaw=0时生效
     */
    if (disable_yaw == 0U) {
        int16_t mx = kb->mouse_x;

        /* 死区处理：±100范围内不响应，避免微小抖动 */
        int16_t abs_mx = (mx >= 0) ? mx : (int16_t)(-mx);
        if (abs_mx <= 100) {
            motion.wz = 0.0f;
        } else {
            /* 限幅到[-660, 660]，防止过大输入 */
            if (mx > Remoter_CHMAX) {
                mx = Remoter_CHMAX;
            } else if (mx < -Remoter_CHMAX) {
                mx = -Remoter_CHMAX;
            }
            /* 映射到角速度：mouse_x → wz */
            motion.wz = (float32_t)Turning_Forward_Feedback * map((float32_t)mx,
                                                                 -(float32_t)Remoter_CHMAX,
                                                                 (float32_t)Remoter_CHMAX,
                                                                 -(float32_t)Max_Velocity,
                                                                 (float32_t)Max_Velocity);
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

    for (int i = 0; i < 4; i++) {
        g_chassis_debug.chassis_output_3508[i] = (float32_t)s_chassis_ctrl_output[i];
    }
    Chassis_Motor_SendControl_DJI(s_chassis_motor, s_chassis_ctrl_output);

    for (int i = 0; i < 4; i++) {
        g_chassis_debug.chassis_actual_speed_3508[i] = s_chassis_motor->motor_msg[i].motor_speed * (Motor_Wheel_Trans);
    }
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
