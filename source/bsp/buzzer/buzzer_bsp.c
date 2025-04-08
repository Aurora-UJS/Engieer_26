#include "buzzer_bsp.h"
#include "tim.h"

/**
 * @brief 蜂鸣器初始化
 * 
 */
void buzzer_init(void)
{
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
}

/**
 * @brief 蜂鸣器开关
 * 
 * @param frequency 频率
 * @param duty_cycle 占空比
 */
void buzzer_on(uint32_t frequency, float duty_cycle)
{
    if (frequency < 16)
    {
        frequency = 16;
    }
    else if (frequency > 1000000)
    {
        frequency = 1000000;
    }

    // 计算PWM频率，所对应的自动重装载值   ---> ARR = 主频 / (预分频+1) / 预期PWM频率(Hz) - 1
    uint16_t pwm_freq_arr = 1000000 / frequency - 1;
    // 计算PWM占空比，所对应比较寄存器的值 ---> CCR = 预期占空比 * (自动重装载值+1)
    // 占空比则由捕获/比较寄存器（TIMx_CRx）寄存器决定。占空比:duty = Pluse / (ARR+1)
    float pwm_duty_pulse = duty_cycle * (pwm_freq_arr + 1);

    // 配置PWM频率 ARR
    __HAL_TIM_SetAutoreload(&htim12, pwm_freq_arr);
    // 配置PWM占空比
    __HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_2, (uint16_t)pwm_duty_pulse);
}

/**
 * @brief 蜂鸣器关闭
 * 
 */
void buzzer_off(void)
{
    buzzer_on(16, 0);
}
