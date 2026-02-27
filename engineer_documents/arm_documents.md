# 机械臂代码说明

## 代码规范

### 变量命名

#### 全局变量

下划线分开

首字母大写

`float Mannal_Joint_Radian[6] = {0};`

#### 局部变量

下划线分开

小写

重要变量首字母大写

### 函数命名

#### 初始化函数

小写

下划线分开

` void joint_init(Joint_t *Joint) `

#### 其它函数

首字母大写

下划线分开

## 机械臂状态机

```c
typedef enum {
  Arm_IDLE_Mode = 0,
  Arm_Custom_Controller_Follow_Mode,
  Arm_Frozen_Mode,
  Arm_Set_Radian,
  Arm_Transition_Mode,
  Arm_Traj_Mode,
  Arm_Rising_Mode
} arm_control_mode_t;
```

`Arm_IDLE_Mode`为默认空闲模式可以切换至任意模式也可以被任意模式切换
`Arm_Custom_Controller_Follow_Mode` 为自定义控制器模式由于其特殊性，所以必须由 `Arm_IDLE_Mode`切换
`Arm_Traj_Mode` 轨迹执行模式结束后将切换至 `Arm_IDLE_Mode`，可以被 `Arm_Custom_Controller_Follow_Mode` 切换。

