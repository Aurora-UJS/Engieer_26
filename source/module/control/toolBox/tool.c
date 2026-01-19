#include "tool.h"
#include "stdint.h"
#include "string.h"
#include "main.h"
#include "cmsis_os2.h"
#include "arm_math_types.h"
#include "arm_math.h"


/**
 * @brief 辅助PID零点跨越计算函数
 * 
 * @param target 目标值
 * @param current 当前值
 * @param circule_num 周期宽度（例如一圈周期宽度为2PI）
 * @return float 
 */
float circule_compensate(float target, float current, float circule_num)
{
    float temp = target - current;
    if (temp > circule_num / 2.0f)
    {
        temp = -circule_num;
    }
    else if (temp < -circule_num / 2.0f)
    {
        temp = circule_num;
    }
    else
    {
        temp = 0;
    }
    return temp;
}

/**
 * @brief 线性映射函数
 * @param value 要映射的值
 * @param in_min 输入范围的最小值
 * @param in_max 输入范围的最大值
 * @param out_min 输出范围的最小值
 * @param out_max 输出范围的最大值
 * @return 映射后的值
 */
float map(float value, float in_min, float in_max, float out_min, float out_max)
{
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
 * @brief 限制函数
 * 
 * @param value 要限制的值
 * @param min 最小值
 * @param max 最大值
 * @return float 
 */
float limit(float value, float min, float max)
{
    // 检查 min 是否大于 max
    if (min > max)
    {
        float temp = min;
        min = max;
        max = temp;
    }

    // 检查是否为 NaN
    if (isnan(value) || isnan(min) || isnan(max))
    {
        return NAN;
    }

    // 使用 isless 和 isgreater 避免浮点数比较的精度问题
    if (isless(value, min))
    {
        return min;
    }
    else if (isgreater(value, max))
    {
        return max;
    }
    else
    {
        return value;
    }
}

/**
 * @brief  通用版浮点数取模函数（兼容所有CMSIS-DSP版本）
 * @param  x: 被取模的浮点数（支持负数、小数）
 * @param  y: 取模的整数（非0）
 * @retval 浮点数取模结果，范围 [0, y)
 */
float arm_float_mod(float x, int32_t y) 
{
    // 异常处理：除数不能为0
    if (y == 0) {
        return NAN;
    }
    
    // 步骤1：计算x/y的商（单精度浮点数）
    float quotient = x / (float)y;
    
    // 步骤2：用标准C的floorf()向下取整（替代arm_floor_f32，兼容所有版本）
    float floor_quotient = floorf(quotient);  // floorf是单精度版floor，更适配float
    
    // 步骤3：计算浮点数取模结果
    float result = x - (float)y * floor_quotient;
    
    return result;
}

/**
 * @brief  浮点数取模并归化到 ±(period/2) 范围
 * @param  x: 被取模的浮点数（支持负数）
 * @param  period: 周期宽度（例如25）
 * @retval 归化后的结果，范围 [-period/2, +period/2)
 * @example angle_normalize(30, 25) = 5
 *          angle_normalize(-5, 25) = -5
 *          angle_normalize(15, 25) = -10
 */
float angle_normalize(float x, float period)
{
    if (period == 0.0f) {
        return NAN;
    }
    
    // 步骤1：取模到 [0, period) 范围
    float result = fmodf(x, period);
    if (result < 0.0f) {
        result += period;  // 保证结果为正
    }
    
    // 步骤2：归化到 [-period/2, +period/2)
    float half_period = period / 2.0f;
    if (result >= half_period) {
        result -= period;
    }
    
    return result;
}
