#include "tool.h"
#include "stdint.h"
#include "string.h"
#include "main.h"
#include "cmsis_os2.h"


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