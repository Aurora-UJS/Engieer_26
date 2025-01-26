#include "LPF.h"

// 初始化滤波器
void initializeFilter(LowPassFilter *filter, float alpha)
{
    filter->alpha = alpha;
    filter->previous_output = 0.0;
}

// 一阶低通滤波函数
float filterValue(LowPassFilter *filter, float input)
{
    // 计算输出
    float output = (1.0 - filter->alpha) * filter->previous_output + filter->alpha * input;

    // 更新上一次的输出
    filter->previous_output = output;

    return output;
}
