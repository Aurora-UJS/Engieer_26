# 小电脑交互系统使用文档

## 概述

本系统实现开发板与小电脑间的数据交互，主要包含视觉数据处理、弹道解算、雷达数据接收和机器人状态反馈等功能。代码基于HAL库开发，支持多模式配置。

---

## 使用示例

```c
初始化流程

computer_uart_init(&huart3);    // 初始化串口
主循环处理

while(1) {
  computer_receive_solve();       // 接收数据处理
  vision_solution(&yaw, &pitch);  // 弹道解算
  send_robot_state();             // 状态反馈
  vTaskDelay(10);
}
```

---

## 模块说明

### 1. 数据接收模块

文件：`computer_receive.c`

```c
void computer_receive_init(UART_HandleTypeDef *huart);
void computer_receive_solve();
```

功能：
    初始化串口接收缓冲区（COMPUTER_BUFFER_SIZE=100）
    接收数据处理流程：
    检查新数据到达
    调用视觉解算vision_receive_solve()
    雷达数据接收receiveRadarData()（需开启radar_MOD）

### 2. 弹道解算模块

文件：`vision_solution.c`

```c
// 弹道解算模式选择
#if BALLISTIC_SOLVER == 1  // 二维弹道模型
#elif BALLISTIC_SOLVER == 2 // 三维弹道模型
#endif
```

核心功能：
    装甲板运动预测
    空气阻力补偿
    迭代角度优化
    弹道延迟补偿

### 3. 状态发送模块

文件：`send_robot_state.c`

```c
void robot_state_msg_init(UART_HandleTypeDef *huart);
void send_robot_state();
```

功能：
    初始化状态发送串口
    发送机器人状态数据包

### 4. 雷达数据接收模块

文件：`receiveRadarData.c`

```c
void receiveRadarData();
```

功能：
数据包格式：

```c
typedef struct {
  uint8_t header;        // 0x5A
  uint8_t detect_color;  // 颜色识别
  float yaw;             // 当前偏航角
  float aim_x;           // 目标X坐标
  // ... 其他字段
} SendPacketVision_t;
```

---

## 配置指南

config.h 关键配置项

```c
#define radar_MOD 1       // 雷达模块开关
#define BALLISTIC_SOLVER 1// 弹道解算器版本
#define BULLET_TYPE 0     // 子弹类型(0-17mm/1-42mm)
#define BULLET_SPEED 17   // 初速度(m/s)
#define TOLERANCE 0.005f  // 解算精度阈值(5mm)
等...
```

## 数据结构说明

```c
视觉数据包结构
c
typedef struct {
  uint8_t header;     // 0xA5
  bool tracking;      // 目标锁定状态
  float x, y, z;      // 目标坐标
  float vx, vy, vz;   // 目标速度
  // ... 其他字段
} ReceivedPacketVision_t;
```

## 注意事项

CRC校验：所有数据包自动添加CRC16校验
实时性要求：主循环周期需≤10ms
坐标系约定：
X轴：前进方向?
Y轴：左侧方向?
Z轴：垂直向上
调试支持：可通过aim_x_n等归一化坐标监控解算结果
版本兼容性

|   组件   | 版本  |          特性          |
| :------: | :---: | :--------------------: |
| 弹道解算 | v1.0  |    二维模型/欧拉法     |
| 弹道解算 | v2.0  | 三维模型/改进迭代算法  |
| 雷达模块 | v1.0  | 支持毫米波雷达数据接入 |
