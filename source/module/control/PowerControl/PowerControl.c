#include "PowerControl.h"
#include "arm_math.h"
#include <stdbool.h>

/* 静态变量 */
static bool PowerControl_Init_flag = false;
static float judgeSys_true_out_power = 0;
static power_control_state_t power_control_mode = POWER_LOSS;
static total_power_control_t total_power;
static float *cap_power = NULL;

/* 用户可注册的钩子函数 */
static void (*user_power_assign_hook)(float total_power, float pid_output) = NULL;

/**************************************
 * 注册用户功率分配钩子函数
 * 参数：
 *   - hook: 用户自定义的功率分配函数
 *           原型：void func(float total_power, float motor_powers[4])
 **************************************/
void Register_PowerAssignHook(void (*hook)(float, float))
{
    user_power_assign_hook = hook;
}

/**************************************
 * 初始化函数
 * 参数：
 *   - buffer_ptr: 底盘功率缓冲值指针
 *   - cap_ptr: 电容电量指针（百分比）
 *   - max_power: 裁判系统最大功率限制
 *   - pid_params: 功率缓冲PID参数
 *    - hook: 用户自定义的功率分配函数
 **************************************/
void PowerControl_Init(uint16_t *buffer_ptr,
                       float *cap_ptr,
                       float *max_power,
                       pid_type_def *pid_params, void (*hook)(float, float))
{
    /* 参数有效性检查 */
    if (!buffer_ptr || !cap_ptr || !pid_params || !hook)
    {
        // 可添加错误处理（如断言或日志）
        return;
    }

    /* 初始化依赖指针 */
    total_power.chassis_power_buffer = buffer_ptr;
    cap_power = cap_ptr;
    total_power.chassis_power_MAX = max_power;

    /* 配置PID控制器 */
    memcpy(&total_power.chassis_power_pid, pid_params, sizeof(pid_type_def)); // 确保目标结构体有独立内存

    Register_PowerAssignHook(hook);

    PowerControl_Init_flag = true;
}

/**************************************
 * 主控制函数（需周期性调用）
 **************************************/
void PowerControl_Update(void)
{
    /* 安全检查 */
    if (!PowerControl_Init_flag || !user_power_assign_hook)
        return;

    /* 阶段1：计算真实输出功率 */
    PID_Calc_Pos(&total_power.chassis_power_pid,
                 *total_power.chassis_power_buffer,
                 30.0f); // 30为缓冲目标值
    judgeSys_true_out_power = *total_power.chassis_power_MAX -
                              total_power.chassis_power_pid.out;

    /* 阶段2：确定可分配功率 */
    float alloc_power = 0;
    if (*cap_power < 5.0f)
    {
        alloc_power = judgeSys_true_out_power;
        power_control_mode = POWER_LOSS;
    }
    else
    {
        switch (power_control_mode)
        {
        case POWER_NORMAL:
            alloc_power = judgeSys_true_out_power + 5.0f;
            break;
        case POWER_BURST:
            alloc_power = judgeSys_true_out_power + 200.0f;
            break;
        default:
            alloc_power = judgeSys_true_out_power;
        }
    }

    /* 阶段3：调用用户功率分配策略 */
    if (user_power_assign_hook)
    {
        user_power_assign_hook(alloc_power, total_power.chassis_power_pid.out);
    }
}

/**************************************
 * 设置功率控制模式
 **************************************/
void Set_PowerControlMode(power_control_state_t mode)
{
    if (mode >= POWER_LOSS && mode <= POWER_BURST)
    {
        power_control_mode = mode;
    }
}

/**
 * @brief 计算单个电机的理论功率消耗
 * @param pid_output     电机速度环PID输出值
 * @param speed_rad      电机当前转速 (rad/s)
 * @param params         功率计算参数结构体指针
 * @return float         计算得到的功率值 (W)
 */
float MotorPower_CalculateSingle(
    float pid_output,
    float speed_rad,
    const MotorPowerParams_t *params)
{
    // 参数安全检查
    if (!params || params->torque_coeff <= 0)
    {
        return 0.0f;
    }

    // 计算各分量
    const float torque = pid_output * params->torque_coeff;
    const float active_power = torque * speed_rad;                // 有效功率
    const float resistive_loss = params->k1 * powf(speed_rad, 2); // 机械损耗
    const float copper_loss = params->k2 * powf(torque, 2);       // 铜损

    // 总功率 = 有效功率 + 损耗
    return active_power + resistive_loss + copper_loss + params->k3;
}

/**
 * @brief 计算二次方程判别式，用于判断电机功率控制方程的根情况
 * @details 根据输入的电机速度、功率参数及缩放功率值，计算二次方程判别式
 *          判别式结果用于判断方程是否有实数解，进而决定后续控制逻辑
 * @param motor_speed 电机当前转速（单位：rad/s）
 * @param params 包含功率计算系数的结构体（k1:机械损耗系数，k2:铜损系数，k3:固定偏移量）
 * @param scaled_power 经过缩放处理的目标功率值
 * @return float 判别式计算结果（b² - 4*k1*c），正数表示存在实数解，负数表示无解
 */
float calculate_Torque_dis(float motor_speed, MotorPowerParams_t params, float scaled_power)
{
    // 定义变量b，值为电机速度
    float b = motor_speed;
    // 定义变量c，值为参数k2乘以电机速度的平方减去缩放功率加上参数k3
    float c = params.k2 * motor_speed * motor_speed - scaled_power + params.k3;
    // 返回b的平方减去4乘以参数k1乘以c
    return (b * b) - (4 * params.k1 * c);
}

float calculate_speed_dis(float torque, MotorPowerParams_t params, float max_power)
{

    // 计算二次方程的系数
    float b_coeff = torque;                                              // 系数 b 对应 τ / 9.55
    float c_coeff = params.k1 * torque * torque + params.k3 - max_power; // 系数 c 对应 k1τ² + k3 - P_max

    // 计算判别式
    return b_coeff * b_coeff - 4 * params.k2 * c_coeff;
}