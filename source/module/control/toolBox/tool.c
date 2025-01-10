#include "tool.h"
#include "stdint.h"
#include "string.h"
#include "main.h"
#include "cmsis_os2.h"
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

float limit(float value, float min, float max)
{
    // 检查 min 是否大于 max
    if (min > max)
    {
        return NAN; // 返回 NaN 表示无效输入
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