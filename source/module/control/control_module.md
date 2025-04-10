# 控制模块文档

## 概述
本文档描述了控制模块，负责机器人运动控制和功率管理。主要功能包括：
- 功率分配与管理
- PID控制算法实现
- 运动学解算（麦克纳姆轮、舵轮）
- 通用工具函数

## 函数

### PowerControl_Init
初始化功率控制系统参看具体子文档[功率控制系统](/source/module/control/PowerControl/PowerControl.md)


### PID_Init
初始化PID控制器

**原型**
```c
void PID_Init(pid_type_def *pid, float kp, float ki, float kd,
             float max_out, float max_iout);
```

**参数**
- `pid`: PID参数结构体指针
- `kp`: 比例系数
- `ki`: 积分系数  
- `kd`: 微分系数
- `max_out`: 输出限幅
- `max_iout`: 积分限幅

**返回值**
- 无

### omni_mecanum_kinematics
麦克纳姆轮运动学解算

**原型**
```c
void omni_mecanum_kinematics(basic_vector_t *motion_vector, float *motor_speed_out);
```

**参数**
- `motion_vector`: 运动向量指针
- `motor_speed_out`: 电机速度输出数组指针

**返回值**
- 无

### circule_compensate
圆周补偿计算

**原型**
```c
float circule_compensate(float target, float current, float circule_num);
```

**参数**
- `target`: 目标角度
- `current`: 当前角度
- `circule_num`: 圆周周期数(如2π为1)

**返回值**
- 补偿后的角度值

### map
数值映射

**原型**
```c
float map(float value, float in_min, float in_max, float out_min, float out_max);
```

**参数**
- `value`: 输入值
- `in_min`: 输入范围最小值
- `in_max`: 输入范围最大值
- `out_min`: 输出范围最小值
- `out_max`: 输出范围最大值

**返回值**
- 映射后的值

### limit
数值限幅

**原型**
```c
float limit(float value, float min, float max);
```

**参数**
- `value`: 输入值
- `min`: 最小值
- `max`: 最大值

**返回值**
- 限幅后的值

### RefFrame_Transform_Calc
参考系变换计算

**原型**
```c
basic_vector_t RefFrame_Transform_Calc(const basic_vector_t basic, float angle);
```

**参数**
- `basic`: 基础运动向量
- `angle`: 变换角度(弧度)

**返回值**
- 变换后的运动向量

### steering_wheel_kinematics
舵轮运动学解算

**原型**
```c
basic_angleV_t steering_wheel_kinematics(basic_vector_t *motion_vector, basic_angleV_t *motor_angleV_out);
```

**参数**
- `motion_vector`: 运动向量指针
- `motor_angleV_out`: 电机角度速度输出指针

**返回值**
- 舵轮角度速度结构体

## 数据结构

### PID参数结构
```c
typedef struct {
    float Kp;//PID 三参数
    float Ki;
    float Kd;

    float max_out;  //输出限幅
    float max_iout; //最大积分输出
    
    float set;// 设定值和反馈值
    float fdb;

    float out; //总输出

    float Pout;//PID 分项输出
    float Iout;
    float Dout;

    float Dbuf[3];  //微分项 0最新 1上一次 2上上次
    float error[3]; //误差项 0最新 1上一次 2上上次
} pid_type_def;
```

### 运动状态结构体
```c
/**
 * @brief 基本运动矢量结构体
 * @param x x方向的速度 (m/s)
 * @param y y方向的速度 (m/s) 
 * @param wz 旋转速度 (rad/s)
 */
typedef struct {
    float x;
    float y;
    float wz;
} basic_vector_t;

/**
 * @brief 角度速度结构体
 * @param angle 角度 (rad)
 * @param v 速度 (m/s)
 */
typedef struct {
    float angle;
    float v;
} basic_angleV_t;
```

## 使用示例
```c
// PID控制示例
pid_type_def pid;
PID_Init(&pid, 1.0, 0.1, 0.05, 1000, 500);
float output = PID_Calc_Pos(&pid, current, target);

// 麦克纳姆轮控制示例  
basic_vector_t motion = {.x=1.0, .y=0.5, .wz=0.2};
float motor_speeds[4];
omni_mecanum_kinematics(&motion, motor_speeds);

// 舵轮控制示例
basic_vector_t steering_motion = {.x=0.8, .y=0.3, .wz=0.1};
basic_angleV_t motor_angles[4]; 
steering_wheel_kinematics(&steering_motion, motor_angles);

// 参考系变换示例
basic_vector_t original = {.x=1.0, .y=0.0, .wz=0.0};
float transform_angle = PI/4; // 45度
basic_vector_t transformed = RefFrame_Transform_Calc(original, transform_angle);
```

## 注意事项
1. 使用前必须正确初始化各子系统
2. PID参数需要根据实际系统调试
3. 运动学解算需匹配机械结构
4. 工具函数已包含安全检查
5. 确保指针参数有效
