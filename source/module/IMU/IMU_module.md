# IMU模块说明文档

## 模块概述
IMU模块负责处理惯性测量单元数据，提供姿态解算功能。主要使用Mahony互补滤波算法。

## 主要API说明

### `Mahony_Init`
初始化Mahony滤波器
```c
void Mahony_Init(float sample_rate_hz);
```

### `BMI088_init`
初始化BMI088 IMU传感器
```c
void BMI088_init(void);
```

### `IMUsys`
姿态解算主函数
```c
void IMUsys(void);
```

### 偏航角修正参数
```c
#define IMU_YAW_OFFSET_CORRECTION 360.0f / 292.0f  // 偏航角修正系数
```
- 该参数用于Mahony滤波器中偏航角数据的修正
- 默认值为360.0f / 292.0f，可根据实际情况调整
- 在Mahony_update函数中应用于yaw轴陀螺仪数据

## 使用示例
```c
#include "IMUtool.h"
#include "BMI088driver.h"

void IMU_Task(void)
{
    // 初始化
    Mahony_Init(1000);  // 1kHz采样率
    BMI088_init();
    
    // 主循环
    while(1) {
        IMUsys();  // 姿态解算
        
        // 获取姿态数据
        float roll = Get_Roll();
        float pitch = Get_Pitch();
        float yaw = Get_Yaw();
        
        osDelay(1);  // 1ms周期
    }
}
