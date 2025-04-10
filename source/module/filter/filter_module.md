# 滤波模块说明文档

## 模块概述
本模块提供多种数字滤波算法实现，主要用于：
- 传感器数据平滑处理
- 信号噪声抑制
- 数据预处理

## 子模块说明

### 1. 低通滤波器(LPF)
- **功能**：一阶低通滤波，用于信号平滑处理
- **核心文件**：
  - LPF.h/c - 低通滤波实现
- **算法原理**：
  ```
  output = (1.0 - alpha) * previous_output + alpha * input
  ```
  其中alpha为平滑因子(0-1)，值越小滤波效果越强

- **主要接口**：
  ```c
  // 初始化滤波器
  void lizeFilter_init(LowPassFilter *filter, float alpha);
  
  // 滤波计算
  float filterValue_Calc(LowPassFilter *filter, float input);
  ```

- **数据结构**：
  ```c
  typedef struct {
    float alpha;           // 时间常数(0-1)
    float previous_output; // 上一时刻输出
  } LowPassFilter;
  ```

- **参数说明**：
  | 参数 | 范围 | 说明 |
  |------|------|------|
  | alpha | 0.0-1.0 | 值越小滤波效果越强，响应越慢 |


## 使用示例

### 低通滤波使用
```c
LowPassFilter lpf;
lizeFilter_init(&lpf, 0.1f); // 初始化，alpha=0.1

float raw_value = getSensorValue();
float filtered = filterValue(&lpf, raw_value); // 滤波处理
```

### 参数选择建议
1. 对于快速变化信号：alpha=0.3-0.5
2. 对于缓慢变化信号：alpha=0.1-0.2
3. 对于强噪声信号：alpha=0.05-0.1

## 注意事项
1. 首次使用需调用lizeFilter_init初始化
2. alpha参数需根据信号特性调整
3. 对于阶跃信号会有延迟，不适合快速响应场景
4. 多通道信号需为每个通道创建独立滤波器实例
