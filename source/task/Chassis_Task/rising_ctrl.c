#include "rising_ctrl.h"
#include "chassis_debug.h"
#include "tool.h"
#include "../IMU_Task/IMU_Task.h"
#include <string.h>

static void test_Rising_DM_PID_Init(pid_type_def pid[]);
#if RISING_DM_IMU_TEST
static void test_pid_IMU(IMU_data_t IMU_data, float32_t target_angle[], float32_t output[]);
#endif

static DJI_motor_t s_rising_dji_obj;
static DJI_motor_t *s_rising_dji = &s_rising_dji_obj;

static pid_type_def s_rising_pid[2];
static pid_type_def s_rising_dm_pid[2];
static float32_t s_rising_target_velocity[2];
static int16_t s_rising_ctrl_output[2];

static DM_motor_t s_rising_dm_l_obj;
static DM_motor_t s_rising_dm_r_obj;
static DM_motor_t *s_rising_dm_l = &s_rising_dm_l_obj;
static DM_motor_t *s_rising_dm_r = &s_rising_dm_r_obj;

static float32_t s_dm_target_angle_l = 0.0f;
static float32_t s_dm_target_angle_r = 0.0f;


/**
 * @brief 初始化抬升控制模块
 */
void Rising_Ctrl_Init(void)
{
    Rising_Init_DJI(&s_rising_dji);
    Motor_Init_DM(&s_rising_dm_l, &s_rising_dm_r);
    Rising_3508_PID_Init(s_rising_pid);
    test_Rising_DM_PID_Init(s_rising_dm_pid);
    Rising_Stop();

    g_chassis_debug.rising_target_speed_3508[Rising_Motor_3508_Left] = 0.0f;
    g_chassis_debug.rising_target_speed_3508[Rising_Motor_3508_Right] = 0.0f;
    g_chassis_debug.rising_actual_speed_3508[Rising_Motor_3508_Left] = 0.0f;
    g_chassis_debug.rising_actual_speed_3508[Rising_Motor_3508_Right] = 0.0f;
    g_chassis_debug.rising_target_angle_dm_l = 0.0f;
    g_chassis_debug.rising_target_angle_dm_r = 0.0f;
    g_chassis_debug.rising_actual_angle_dm_l = 0.0f;
    g_chassis_debug.rising_actual_angle_dm_r = 0.0f;
}

/**
 * @brief 关闭抬升电机输出
 */
void Rising_Stop(void)
{
    s_rising_ctrl_output[Rising_Motor_3508_Left] = 0;
    s_rising_ctrl_output[Rising_Motor_3508_Right] = 0;

    g_chassis_debug.rising_target_speed_3508[Rising_Motor_3508_Left] = 0.0f;
    g_chassis_debug.rising_target_speed_3508[Rising_Motor_3508_Right] = 0.0f;

    if (s_rising_dji != NULL) {
        Rising_Motor_SendControl_DJI(s_rising_dji, s_rising_ctrl_output);

        g_chassis_debug.rising_actual_speed_3508[Rising_Motor_3508_Left] = s_rising_dji->motor_msg[Rising_Motor_3508_Left].motor_speed * (Motor_Wheel_Trans);
        g_chassis_debug.rising_actual_speed_3508[Rising_Motor_3508_Right] = s_rising_dji->motor_msg[Rising_Motor_3508_Right].motor_speed * (Motor_Wheel_Trans);
    }

    if (s_rising_dm_l != NULL && s_rising_dm_r != NULL) {
        g_chassis_debug.rising_target_angle_dm_l = Rising_DM_ZeroPoint;
        g_chassis_debug.rising_target_angle_dm_r = -Rising_DM_ZeroPoint;

        Rising_Motor_SendControl_DM(s_rising_dm_l, s_rising_dm_r, Rising_DM_ZeroPoint, -Rising_DM_ZeroPoint);

        g_chassis_debug.rising_actual_angle_dm_l = s_rising_dm_l->motor_msg.motor_angle;
        g_chassis_debug.rising_actual_angle_dm_r = s_rising_dm_r->motor_msg.motor_angle;
    }
}

/**
 * @brief 普通模式下的抬升控制逻辑
 *
 * @param remoter 遥控器数据指针
 */
void Rising_Normal_Mode(const rc_info_t *remoter)
{
    (void)remoter;
    Rising_Stop();
}

/**
 * @brief 上楼模式下的抬升控制逻辑
 *
 * @param remoter 遥控器数据指针
 */
void Rising_Upstairs_Mode(const rc_info_t *remoter)
{
    if (remoter == NULL || s_rising_dji == NULL || s_rising_dm_l == NULL || s_rising_dm_r == NULL) {
        return;
    }

    Rising_Motor_TargetVelocity(s_rising_target_velocity, *remoter);

    g_chassis_debug.rising_target_speed_3508[Rising_Motor_3508_Left] = s_rising_target_velocity[Rising_Motor_3508_Left];
    g_chassis_debug.rising_target_speed_3508[Rising_Motor_3508_Right] = s_rising_target_velocity[Rising_Motor_3508_Right];

    Rising_3508_PID_Calculate(s_rising_pid,
                             s_rising_target_velocity,
                             s_rising_dji,
                             s_rising_ctrl_output);
    Rising_Motor_SendControl_DJI(s_rising_dji, s_rising_ctrl_output);

    g_chassis_debug.rising_actual_speed_3508[Rising_Motor_3508_Left] = s_rising_dji->motor_msg[Rising_Motor_3508_Left].motor_speed * (Motor_Wheel_Trans);
    g_chassis_debug.rising_actual_speed_3508[Rising_Motor_3508_Right] = s_rising_dji->motor_msg[Rising_Motor_3508_Right].motor_speed * (Motor_Wheel_Trans);

#if RISING_DM_IMU_TEST
    /* IMU Pitch -> DM angle closed-loop test: target pitch is 0, output is DM target angle command (clamped). */
    float32_t target_angle[2];
    float32_t output_angle[2];
    test_pid_IMU(IMU_data, target_angle, output_angle);
    s_dm_target_angle_l = output_angle[0];
    s_dm_target_angle_r = output_angle[1];
#else
    Rising_Motor_TargetAngle(&s_dm_target_angle_l, &s_dm_target_angle_r, *remoter);
#endif

    g_chassis_debug.rising_target_angle_dm_l = s_dm_target_angle_l;
    g_chassis_debug.rising_target_angle_dm_r = s_dm_target_angle_r;
    Rising_Motor_SendControl_DM(s_rising_dm_l, s_rising_dm_r, s_dm_target_angle_l, s_dm_target_angle_r);

    g_chassis_debug.rising_actual_angle_dm_l = s_rising_dm_l->motor_msg.motor_angle;
    g_chassis_debug.rising_actual_angle_dm_r = s_rising_dm_r->motor_msg.motor_angle;
}

void Rising_Init_DJI(DJI_motor_t **Rising_Motor)
{
    if (Rising_Motor == NULL || *Rising_Motor == NULL) {
        return;
    }

    memset(*Rising_Motor, 0, sizeof(DJI_motor_t));
    (*Rising_Motor)->can_cfg.port = CAN1_PORT;
    (*Rising_Motor)->can_cfg.id = Rising_Motor_ALL_id;

    (*Rising_Motor)->motor_msg[Rising_Motor_3508_Left].can_msg.id = Rising_Motor_3508_Left_id;
    (*Rising_Motor)->motor_msg[Rising_Motor_3508_Right].can_msg.id = Rising_Motor_3508_Right_id;

    Motor_DJI_Init(*Rising_Motor);
}

void Motor_Init_DM(DM_motor_t **Rising_Motor_L, DM_motor_t **Rising_Motor_R)
{
    if (Rising_Motor_L == NULL || *Rising_Motor_L == NULL) {
        return;
    }

    memset(*Rising_Motor_L, 0, sizeof(DM_motor_t));
    (*Rising_Motor_L)->can_cfg.id = DM_l0010l_CAN_ID_Left;
    (*Rising_Motor_L)->motor_msg.can_msg.id = DM_l0010l_Master_ID_Left;
    (*Rising_Motor_L)->can_cfg.port = CAN3_PORT;
    (*Rising_Motor_L)->tmp.PMAX = 12.5f;
    (*Rising_Motor_L)->tmp.VMAX = 3.0f;
    (*Rising_Motor_L)->tmp.TMAX = 100.0f;

    Motor_DM_Init(*Rising_Motor_L);
    Motor_DM_Enable(*Rising_Motor_L);

    if (Rising_Motor_R == NULL || *Rising_Motor_R == NULL) {
        return;
    }

    memset(*Rising_Motor_R, 0, sizeof(DM_motor_t));
    (*Rising_Motor_R)->can_cfg.id = DM_l0010l_CAN_ID_Right;
    (*Rising_Motor_R)->motor_msg.can_msg.id = DM_l0010l_Master_ID_Right;
    (*Rising_Motor_R)->can_cfg.port = CAN3_PORT;
    (*Rising_Motor_R)->tmp.PMAX = 12.5f;
    (*Rising_Motor_R)->tmp.VMAX = 3.0f;
    (*Rising_Motor_R)->tmp.TMAX = 100.0f;

    Motor_DM_Init(*Rising_Motor_R);
    Motor_DM_Enable(*Rising_Motor_R);

    osDelay(200);
}

void Rising_Motor_SendControl_DJI(DJI_motor_t *DJMotor, int16_t output[])
{
    Motor_DJI_Refresh(DJMotor);
    set_motor_parameter(
        DJMotor,
        output[Rising_Motor_3508_Left],
        output[Rising_Motor_3508_Right],
        0,
        0);
}

void Rising_Motor_SendControl_DM(DM_motor_t *DMMotor_L, DM_motor_t *DMMotor_R, float32_t output_L, float32_t output_R)
{
    Motor_DM_Refresh(DMMotor_L);
    Motor_DM_Refresh(DMMotor_R);

    PosSpeed_CtrlMotorDM(DMMotor_L, output_L, Rising_DM_Velocity);
    osDelay(1);
    PosSpeed_CtrlMotorDM(DMMotor_R, output_R, Rising_DM_Velocity);
}

DM_motor_t *Rising_Get_DmMotor_L(void)
{
    return s_rising_dm_l;
}

DM_motor_t *Rising_Get_DmMotor_R(void)
{
    return s_rising_dm_r;
}

void Rising_Motor_TargetVelocity(float32_t Target_Velocity[], rc_info_t remoter)
{
    float32_t Velocity = map(remoter.ch2,
                             -Remoter_CHMAX,
                             Remoter_CHMAX,
                             -Max_Rising_Motor_Velocity,
                             Max_Rising_Motor_Velocity);

    Target_Velocity[Rising_Motor_3508_Left] = Velocity;
    Target_Velocity[Rising_Motor_3508_Right] = -Velocity;
}

void Rising_Motor_TargetAngle(float32_t *Target_Angle_L, float32_t *Target_Angle_R, rc_info_t remoter)
{
    float32_t ch4 = (float32_t)remoter.ch4;

    if (ch4 < 0.0f) {
        ch4 = 0.0f;
    }

    if (ch4 > 660.0f) {
        ch4 = 660.0f;
    }

    float32_t Angle = map(ch4,
                          Rising_DM_ZeroPoint,
                          Remoter_CHMAX,
                          Rising_DM_ZeroPoint,
                          Max_Rising_DM_angle);

    *Target_Angle_L = Angle;
    *Target_Angle_R = -Angle;
}

void Rising_3508_PID_Init(pid_type_def pid[])
{
    for (int i = 0; i < 2; i++) {
        PID_Init(pid + i,
                 Rising_3508_PID_kp,
                 Rising_3508_PID_ki,
                 Rising_3508_PID_kd,
                 Rising_3508_PID_Maxout,
                 Rising_3508_PID_Maxiout);
    }
}

void Rising_3508_PID_Calculate(pid_type_def pid[], float32_t target_speed[], DJI_motor_t *motor, int16_t output[])
{
    float32_t curren_wheel_speed[2];

    for (int i = 0; i < 2; i++) {
        curren_wheel_speed[i] = motor->motor_msg[i].motor_speed * (Motor_Wheel_Trans);
        output[i] = (int16_t)(PID_Calc_Add(pid + i, curren_wheel_speed[i], *(target_speed + i)));
    }
}

void test_Rising_DM_PID_Init(pid_type_def pid[])
{
    for (int i = 0; i < 2; i++) {
        PID_Init(pid + i,
                 Rising_DM_PID_kp,
                 Rising_DM_PID_ki,
                 Rising_DM_PID_kd,
                 Rising_DM_PID_Maxout,
                 Rising_DM_PID_Maxiout);
    }
}

#if RISING_DM_IMU_TEST
static void test_pid_IMU(IMU_data_t IMU_data,float32_t target_angle[],float32_t output[])
{
    if (target_angle == NULL || output == NULL) {
        return;
    }

    float32_t pitch = IMU_data.Pitch * RISING_IMU_PITCH_SIGN;
    float32_t delta_angle = PID_Calc_Pos(&s_rising_dm_pid[0], pitch, 0.0f);
    /* Map PID output to angle command and clamp between mechanical limits (0-point and max-point). */
    float32_t angle_cmd = limit(Rising_DM_ZeroPoint + delta_angle, Rising_DM_ZeroPoint, Max_Rising_DM_angle);

    target_angle[0] = Rising_DM_ZeroPoint;
    target_angle[1] = -Rising_DM_ZeroPoint;

    output[0] = angle_cmd;
    output[1] = -angle_cmd;
}
#endif
