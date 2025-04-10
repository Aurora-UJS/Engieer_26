# 控制模块(Control Module)说明文档

## 1. 模块概述

控制模块负责机器人运动控制和功率管理，包含以下子模块：
- **PowerControl**: 功率控制与管理
- **PID**: PID控制算法实现  
- **KinematicSolution**: 运动学解算(角度映射、麦克纳姆轮、舵轮)
- **toolBox**: 通用工具函数集合

## 2. 子模块说明

### 2.1 PowerControl功率控制

#### 功能
- 电机功率计算与管理
- 功率限制与分配

#### 核心API
```c
void PowerControl_Init(float *buffer_ptr, float *cap_ptr, float max_power, 
                      pid_type_def *pid_params, void (*hook)(float));
void PowerControl_Update(void);
float MotorPower_CalculateSingle(float pid_output, float speed_rpm, 
                               const MotorPowerParams_t *params);
```

### 2.2 PID控制

#### 功能
- 增量式PID计算
- 位置式PID计算  

#### 数据结构
```c
typedef struct {
    float Kp, Ki, Kd;       // PID参数
    float max_out, max_iout; // 输出限幅
    float set, fdb;          // 目标值/反馈值
    float out;               // 输出值
    float Pout, Iout, Dout;  // 各分量输出
    float Dbuf[3];           // 微分项缓冲区
    float error[3];          // 误差项缓冲区
} pid_type_def;
```

#### 核心API
```c 
void PID_Init(pid_type_def *pid, float kp, float ki, float kd, 
             float max_out, float max_iout);
float PID_Calc_Add(pid_type_def *pid, float now, float target); // 增量式
float PID_Calc_Pos(pid_type_def *pid, float now, float target); // 位置式
```

### 2.3 KinematicSolution运动学解算

#### 2.3.1 角度映射
```c
basic_vector_t RefFrame_Transform_Calc(const basic_vector_t basic, float angle);
```

#### 2.3.2 麦克纳姆轮运动学
```c
void omni_mecanum_kinematics(basic_vector_t *motion_vector, float *motor_speed_out);
```

#### 2.3.3 舵轮运动学  
```c
basic_angleV_t steering_wheel_kinematics(basic_vector_t *motion_vector, 
                                       basic_angleV_t *motor_angleV_out);
```

### 2.4 toolBox工具函数

#### 核心功能
- 数值限制与映射
- 循环补偿计算

#### 核心API
```c
float circule_compensate(float target, float current, float circule_num);
float map(float value, float in_min, float in_max, float out_min, float out_max); 
float limit(float value, float min, float max);
```

## 3. 使用示例

### 3.1 PID控制示例
```c
pid_type_def pid;
PID_Init(&pid, 1.0, 0.1, 0.05, 1000, 500);

while(1) {
    float output = PID_Calc_Pos(&pid, current_value, target_value);
    // 使用output控制执行器
}
```

### 3.2 麦克纳姆轮控制示例
```c
basic_vector_t motion = {.x=1.0, .y=0.5, .wz=0.2};
float motor_speeds[4];
omni_mecanum_kinematics(&motion, motor_speeds);
// 使用motor_speeds控制四个电机
```

## 4. 注意事项

1. 使用PowerControl前需正确初始化
2. PID参数需根据实际系统调试
3. 运动学解算需匹配实际机械结构
4. 工具函数包含安全检查，可直接使用
