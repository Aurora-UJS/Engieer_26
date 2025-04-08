#include "LPF.h"

/**
 * @brief 一阶低通滤波器初始化函数。
 * 
 * 此函数用于初始化一阶低通滤波器的参数，设置滤波器的平滑因子（alpha）以及初始输出值。
 * 平滑因子 alpha 的取值范围通常在 (0, 1] 之间：
 * - 当 alpha 接近 1 时，滤波器对输入信号的变化更敏感，响应更快，但滤波效果较弱。
 * - 当 alpha 接近 0 时，滤波器对输入信号的变化反应较慢，但滤波效果更强。
 * 
 * @param filter 指向 LowPassFilter 结构体的指针，表示滤波器实例。
 * @param alpha 平滑因子，控制滤波器的响应速度和平滑程度。
 */
void lizeFilter_init(LowPassFilter *filter, float alpha)
{
    // 设置滤波器的平滑因子 alpha
    filter->alpha = alpha;

    // 初始化滤波器的上一次输出值为 0.0
    filter->previous_output = 0.0;
}

/**
 * @brief 一阶低通滤波器计算函数。
 * 
 * 此函数根据当前输入值和滤波器的平滑因子 alpha 计算滤波后的输出值。
 * 滤波公式为：
 *     output = (1.0 - alpha) * previous_output + alpha * input
 * 其中：
 * - previous_output 表示上一次滤波的输出值。
 * - alpha 是平滑因子，控制滤波器的响应速度和平滑程度。
 * - input 是当前输入值。
 * 
 * @param filter 指向 LowPassFilter 结构体的指针，表示滤波器实例。
 * @param input 当前输入值。
 * @return float 返回滤波后的输出值。
 */
float filterValue_Calc(LowPassFilter *filter, float input)
{
    // 根据滤波公式计算当前输出值
    float output = (1.0 - filter->alpha) * filter->previous_output + filter->alpha * input;

    // 更新滤波器的上一次输出值为当前计算的输出值
    filter->previous_output = output;

    // 返回滤波后的输出值
    return output;
}
