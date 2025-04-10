# 小电脑交互系统使用文档

## 概述

本系统实现开发板与小电脑间的数据交互，主要包含视觉数据处理、弹道解算、雷达数据接收和机器人状态反馈等功能。代码基于HAL库开发，支持多模式配置。

---

## 使用示例

```c
//初始化流程
SendPacketAllRobotHP_t hp;
SendPacketGameStatus_t sendGameStatus;   // 比赛状态数据包
SendPacketRobotStatus_t sendRobotStatus; // 机器人状态数据包
computer_uart_init(&huart3);    // 初始化串口
set_robot_angle(&now_yaw, &now_pitch); // 绑定当前角度
set_robot_id(id);   // 设置机器人ID 依据裁判系统主要区分蓝黄

//主循环处理
while(1) {
  computer_receive_solve();       // 接收数据处理
  vision_solution(&target_yaw, &target_pitch);  // 弹道解算发目标角度
  send_robot_state();             // 状态反馈

  hp.red_1_robot_hp = get_red_1_hp(); // 获取机器人血量(这个你们具体看裁判系统那边怎么写诸如子类的填满)
  ...// 其他机器人血量
  sendAllRobotHPData(&hp);           // 发送血量数据包

  sendGameStatus.game_progress = get_game_progress(); // 获取比赛状态(这个也是)
  ...// 其他比赛状态
  sendGameStatusData(sendGameStatus); // 发送比赛状态数据包

  sendRobotStatus.robot_id = get_robot_id(); // 获取机器人ID
  ...// 其他机器人状态
  sendRobotStatusData(sendRobotStatus); // 发送机器人状态数据包

  if(ReceivedPacketVision.id != what_I_want) {// 判断是否捕获指定机器人的坐标
    change_robot_target();//尝试换一个目标（不一定成功，因为可能只看到一个机器人）
  }
  vTaskDelay(10);
}

/* 视觉数据获取宏组 --------------------------------------------------------------*/

/**
 * @brief 获取视觉数据包的ID值
 * 
 * 该宏用于快速访问ReceivedPacketVision结构体中的id字段，通常用于标识当前接收到的
 * 视觉数据包编号或类型。宏展开后直接返回结构体的整型id成员值。
 * 
 * @return int 视觉数据包标识符，具体取值范围由协议定义
 */
#define get_vision_id() (ReceivedPacketVision.id)

/**
 * @brief 检测视觉跟踪状态
 * 
 * 该宏用于快速获取目标跟踪状态，通过访问ReceivedPacketVision结构体的tracking字段
 * 判断当前视觉系统是否处于有效跟踪状态。宏展开后返回结构体的布尔型tracking成员值。
 * 
 * @return bool 跟踪状态标识，true表示正在跟踪，false表示丢失目标
 */
#define is_tracking() (ReceivedPacketVision.tracking)

/* 雷达/云台目标运动状态获取宏组 ----------------------------------------------------*/

/**
 * @brief 获取目标在X轴方向的线速度
 * @return 目标X轴线速度值，单位：米/秒(m/s)
 */
#define get_target_x() (radarTwist.linear_x)

/**
 * @brief 获取目标在Y轴方向的线速度
 * @return 目标Y轴线速度值，单位：米/秒(m/s) 
 */
#define get_target_x() (radarTwist.linear_x)

/**
 * @brief 获取目标在Z轴方向的线速度
 * @return 目标Z轴线速度值，单位：弧度/秒(rad/s)
 */
#define get_target_z() (radarTwist.angular_z)

/* 雷达/云台自身角速度获取宏组 ------------------------------------------------------*/

/**
 * @brief 获取雷达/云台绕X轴的旋转角速度
 * @return X轴角速度值，单位：弧度/秒(rad/s)
 */
#define get_angular_x() (radarTwist.angular_x)

/**
 * @brief 获取雷达/云台绕Y轴的旋转角速度
 * @return Y轴角速度值，单位：弧度/秒(rad/s)
 */
#define get_angular_y() (radarTwist.angular_y)

/**
 * @brief 获取雷达/云台绕Z轴的旋转角速度
 * @return Z轴角速度值，单位：弧度/秒(rad/s)
 */
#define get_angular_z() (radarTwist.angular_z)

/* 机器人控制状态获取宏组 ----------------------------------------------------------*/

/**
 * @brief 获取云台扫描停止状态
 * @return 停止扫描标志位，布尔类型：true-停止扫描，false-允许扫描
 */
#define get_gimbal_stop() (robotControl.stop_gimbal_scan)

/**
 * @brief 获取底盘旋转速度
 * @return 底盘旋转速度值，单位：弧度/秒(rad/s)
 * @note 正值表示顺时针旋转，负值表示逆时针旋转
 */
#define get_chassis_spin() (robotControl.chassis_spin_vel)
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
-    初始化串口接收缓冲区
-    接收数据处理流程：
-    检查新数据到达
-    调用视觉解算vision_receive_solve()
-    雷达数据接收receiveRadarData()（需开启radar_MOD）

### 2. 弹道解算模块

文件：`vision_solution.c`

```c
// 弹道解算模式选择
#if BALLISTIC_SOLVER == 1  // 二维弹道模型
#elif BALLISTIC_SOLVER == 2 //二维弹道模型
#endif
```

核心功能：
-    装甲板运动预测
-    空气阻力补偿
-    迭代角度优化
-    弹道延迟补偿

### 3. 状态发送模块

文件：`send_robot_state.c`

```c
void robot_state_msg_init(UART_HandleTypeDef *huart);
void send_robot_state();
```

功能：
-    初始化状态发送串口
-    发送机器人状态数据包

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

实时性要求：主循环周期需约等于10ms
坐标系约定：
- X轴：初式正方向（可能是）
- Y轴：初式左侧方向（可能是）
- Z轴：垂直向上

调试支持：可通过aim_x_n等归一化坐标监控解算结果

## 版本：

|   组件   | 版本  |          特性          |
|------------|------|-----------------------|
| 弹道解算   | v1.0 | 二维模型/欧拉法（有问题）       |
| 弹道解算   | v2.0 | 二维模型/解析法（没那么有问题） |
| 雷达模块   | v1.0 | 激光雷达数据接入    |
