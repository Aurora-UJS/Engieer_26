# 功率控制模块使用说明

## 1. 概述

功率控制模块用于控制设备的功率输出，包括设备的率调节操作。该模块提供了一系列的API接口，用户可以通过调用这些接口来实现对设备的控制。但当前主要是进行电机的功率控制。

## 2. 接口说明

### 2.1. 初始化函数

```c
void PowerControl_Init(float *buffer_ptr,
                       float *cap_ptr,
                       float max_power,
                       pid_type_def *pid_params, void (*hook)(float));
void PowerControl_Update(void); // 更新功率控制状态
```

函数功能：初始化功率控制模块，设置功率控制参数。

参数说明：

- `buffer_ptr`：功率控制缓冲区指针，用于存储功率控制数据。
- `cap_ptr`：功率控制电容指针，用于存储功率控制电容数据。
- `max_power`：功率控制最大功率，用于限制功率控制的最大输出功率。
- `pid_params`：PID参数结构体指针，用于设置功率控制的PID参数。
- `hook`：回调函数指针，用于在功率控制过程中调用主要设备参数。

### 2.2. 更新函数

```c
void PowerControl_Update(void); // 更新功率控制状态
```

函数功能：更新功率控制状态。

### 2.3. 其他函数

#### 2.3.1. 电机功率计算函数

```c
float MotorPower_CalculateSingle(
    float pid_output, 
    float speed_rpm, 
    const MotorPowerParams_t *params);
```

函数功能：计算单个电机的理论功率消耗。

参数说明：

- `pid_output`：PID输出值，用于计算功率消耗。
- `speed_rpm`：电机转速，用于计算功率消耗。
- `params`：电机功率参数结构体指针，用于设置电机功率参数。

返回值：

- 返回计算得到的电机功率消耗值。
- 如果输入参数有NULL，则返回0。

## 3. 使用示例

```c
#include "PowerControl.h"

void hook(float power)
{
    // 在这里调用主要设备参数
}

int main(void)
{
    float buffer;
    float cap;
    float max_power = 100.0;
    pid_type_def pid_params;
    pid_init(&pid_params, 1, 0, 0, 16000,16000);  

    PowerControl_Init(&buffer, &cap, &max_power, &pid_params, hook);

    while (1)
    {
        PowerControl_Update();
    }

    return 0;
}
```

## 4. 注意事项

- 在使用功率控制模块之前，需要先初始化功率控制模块。

## 5. 附录

一些功率控制模块的实现示例代码。

```c
void hook(float power)
{
    
}
```
