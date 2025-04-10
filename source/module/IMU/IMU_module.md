# IMU模块文档

## 概述
本模块处理惯性测量单元数据，主要功能包括：
- 使用Mahony互补滤波算法进行姿态解算
- 实时提供横滚角、俯仰角和偏航角
- 支持偏航角偏移校正

## 核心函数

### Mahony_Init
初始化Mahony滤波器

**函数原型**
```c
void Mahony_Init(float sample_rate_hz);
```

**参数说明**
- `sample_rate_hz`: 采样频率(Hz)

### BMI088_init
初始化BMI088传感器

**函数原型**
```c
void BMI088_init(void);
```

### IMUsys
姿态解算主函数

**函数原型**
```c
void IMUsys(void);
```

## 姿态获取函数

### Get_Roll
获取横滚角

**函数原型**
```c
float getRoll(void);
```

### Get_Pitch
获取俯仰角

**函数原型**
```c
float getPitch(void);
```

### Get_Yaw
获取偏航角

**函数原型**
```c
float getYaw(void);
```

## 配置参数

### 偏航角校正
```c
#define IMU_YAW_OFFSET_CORRECTION (360.0f / 292.0f)
```
- 应用于Mahony滤波器中的偏航角数据
- 默认值补偿传感器特定偏差
- 可根据校准结果调整

## 使用示例
```c
#include "IMUtool.h"

void IMU_Task(void) {
    // 初始化
    Mahony_Init(1000); // 1kHz采样率
    BMI088_init();
    
    while(1) {
        IMUsys(); // 姿态解算
        
        // 获取姿态数据
        float roll = getRoll();
        float pitch = getPitch();
        float yaw = getYaw();
        
        osDelay(1); // 1ms周期
    }
}
```

## 注意事项
1. 需要定期调用IMUsys()进行连续估计
2. 传感器校准对结果准确性至关重要
3. 磁力计数据可提高偏航角精度(可选)
4. 默认配置适用于BMI088传感器
