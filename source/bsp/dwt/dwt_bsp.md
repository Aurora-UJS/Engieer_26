# DWT BSP 模块

## 概述

DWT (Data Watchpoint and Trace) BSP模块利用ARM Cortex-M内核的DWT外设，为STM32微控制器提供高精度的计时和延时功能。该模块能够实现精确的时间测量和微秒级的系统时间跟踪。

## 功能特点

- 高精度系统时间跟踪
- 微秒级分辨率的时间测量
- 精确的延时功能
- 64位计时器计数器，防止溢出问题
- 多种格式的时间获取（秒、毫秒、微秒）

## 硬件要求

- 带有DWT外设的ARM Cortex-M内核STM32微控制器
- 正确配置的系统时钟

## API参考

### 初始化

```c
void DWT_Init(uint32_t CPU_Freq_mHz);
```

**描述：** 初始化DWT外设用于计时操作。

**参数：**
- `CPU_Freq_mHz`：CPU频率，单位为MHz

**示例：**
```c
// 使用480MHz的CPU频率初始化DWT
DWT_Init(480);
```

### 时间测量函数

#### 获取时间差（32位浮点数）

```c
float DWT_GetDeltaT(uint32_t *cnt_last);
```

**描述：** 计算当前时间与之前时间戳之间的时间差。

**参数：**
- `cnt_last`：指向存储之前时间戳的变量的指针（将被更新为当前时间）

**返回值：** 时间差，单位为秒（float类型）

**示例：**
```c
uint32_t last_time = 0;
float dt = DWT_GetDeltaT(&last_time);
// dt包含了以秒为单位的经过时间
```

#### 获取时间差（64位双精度浮点数）

```c
double DWT_GetDeltaT64(uint32_t *cnt_last);
```

**描述：** 计算当前时间与之前时间戳之间的时间差，具有更高的精度。

**参数：**
- `cnt_last`：指向存储之前时间戳的变量的指针（将被更新为当前时间）

**返回值：** 时间差，单位为秒（double类型）

### 系统时间函数

#### 更新系统时间

```c
void DWT_SysTimeUpdate(void);
```

**描述：** 使用当前时间值更新内部系统时间结构。

#### 获取系统时间（秒）

```c
float DWT_GetTimeline_s(void);
```

**描述：** 获取当前系统时间，单位为秒。

**返回值：** 当前系统时间，单位为秒（float类型）

**示例：**
```c
float current_time = DWT_GetTimeline_s();
// current_time包含了以秒为单位的系统时间
```

#### 获取系统时间（毫秒）

```c
float DWT_GetTimeline_ms(void);
```

**描述：** 获取当前系统时间，单位为毫秒。

**返回值：** 当前系统时间，单位为毫秒（float类型）

**示例：**
```c
float current_time_ms = DWT_GetTimeline_ms();
// current_time_ms包含了以毫秒为单位的系统时间
```

#### 获取系统时间（微秒）

```c
uint64_t DWT_GetTimeline_us(void);
```

**描述：** 获取当前系统时间，单位为微秒。

**返回值：** 当前系统时间，单位为微秒（uint64_t类型）

**示例：**
```c
uint64_t current_time_us = DWT_GetTimeline_us();
// current_time_us包含了以微秒为单位的系统时间
```

### 延时函数

```c
void DWT_Delay(float Delay);
```

**描述：** 提供精确的延时，单位为秒。

**参数：**
- `Delay`：延时时间，单位为秒

**示例：**
```c
// 延时0.5秒
DWT_Delay(0.5);
```

## 内部数据结构

### DWT_Time_t

该结构体保存系统时间组件：

```c
typedef struct
{
    uint32_t s;  // 秒
    uint32_t ms; // 毫秒
    uint32_t us; // 微秒
} DWT_Time_t;
```

## 实现细节

DWT模块使用Cortex-M内核的DWT CYCCNT寄存器，该寄存器计数CPU周期。该模块：

1. 初始化DWT外设并启用周期计数器
2. 跟踪计数器溢出以维持64位周期计数
3. 根据CPU频率将周期计数转换为时间单位
4. 提供各种函数以不同格式访问时间

该实现通过检测当前计数是否小于前一个计数来处理计数器溢出，这表明32位计数器已经环绕。

## 使用注意事项

1. 在使用任何计时功能之前，必须使用正确的CPU频率初始化DWT模块
2. 为了获得最大精度，请使用适合您应用程序的最高精度函数
3. 64位计数器实现防止了计时器溢出问题，使其适用于长时间运行的应用程序
4. 时间测量基于CPU周期，因此CPU频率的任何变化都会影响计时精度

## 使用示例

```c
// 使用480MHz的CPU频率初始化DWT
DWT_Init(480);

// 用于计时的变量
uint32_t last_time = 0;
float elapsed_time;

// 主循环
while (1)
{
    // 获取自上次测量以来的时间差
    elapsed_time = DWT_GetDeltaT(&last_time);
    
    // 使用经过的时间做一些事情
    if (elapsed_time > 1.0f)
    {
        // 每秒执行一次的操作
        // ...
    }
    
    // 获取当前系统时间
    float current_time_s = DWT_GetTimeline_s();
    float current_time_ms = DWT_GetTimeline_ms();
    uint64_t current_time_us = DWT_GetTimeline_us();
    
    // 使用精确延时
    DWT_Delay(0.01); // 10毫秒延时
}
