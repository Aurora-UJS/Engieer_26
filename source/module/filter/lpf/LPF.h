#ifndef __LPF_H__
#define __LPF_H__

typedef struct
{
    float alpha;           // 时间常数
    float previous_output; // 上一时刻的输出
} LowPassFilter;

float filterValue(LowPassFilter *filter, float input);     // 输入新的数据，返回滤波后的数据
float filterValue_Calc(LowPassFilter *filter, float input); // 输入新的数据，返回滤波后的数据
void initializeFilter(LowPassFilter *filter, float alpha); // 初始化滤波器
void lizeFilter_init(LowPassFilter *filter, float alpha);  // 初始化滤波器
#endif
