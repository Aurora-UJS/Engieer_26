# 键盘模式调试指南

## 当前配置
- **键盘数据源**: `USE_REMOTER_KEYBOARD = 1` (使用遥控器DBUS键盘数据)
- **默认模式**: DBUS模式（遥控器摇杆控制）

## 问题排查步骤

### 1. 确认模式切换
按 **B 键**，检查是否能切换到 Keyboard 模式。
- 如果能切换，说明键盘数据有被接收到
- 如果不能切换，说明 `remoter.keyboard` 没有数据

### 2. 检查遥控器连接
**重要**：DR16 遥控器通过 DBUS 协议传输键盘数据时，需要：
- 遥控器已连接并正常工作
- 遥控器接收机正常接收信号
- UART5 正常接收 18 字节数据包

### 3. 检查键盘数据
在 `Chassis_Keyboard_Mode` 函数开始处添加调试代码：

```c
void Chassis_Keyboard_Mode(const keyboard_t *kb, uint8_t disable_yaw)
{
    if (kb == NULL || s_chassis_motor == NULL) {
        return;
    }

    // 调试：打印键盘数据
    // printf("KB: W=%d S=%d A=%d D=%d, mouse_x=%d\r\n", 
    //        kb->key_code.bit.W, kb->key_code.bit.S, 
    //        kb->key_code.bit.A, kb->key_code.bit.D, kb->mouse_x);
    
    // ... 后续代码
}
```

### 4. 可能的问题

#### 问题 A：遥控器没有键盘功能
**现象**：DR16 遥控器本身不支持键盘输入
**解决**：
- 如果使用的是标准 DR16 遥控器（只有摇杆和拨码），它**不包含键盘数据**
- 需要使用图传链路或裁判系统来传输键盘数据
- 将宏定义改为使用裁判系统键盘：
  ```c
  #define USE_REMOTER_KEYBOARD 0  // 改为使用裁判系统键盘
  ```

#### 问题 B：键盘数据全为 0
**现象**：`remoter.keyboard.key_code.key_code` 始终为 0
**原因**：DBUS 协议的键盘字段没有实际数据
**解决**：使用裁判系统键盘数据源

#### 问题 C：底盘电机未响应
**检查**：
1. `sw2` 拨码开关位置（不能在断电位置 2）
2. 电机 CAN 通信是否正常
3. PID 参数是否合理

## 推荐配置

### 如果使用图传链路/裁判系统键盘
```c
// Referee_Task.h
#define USE_REMOTER_KEYBOARD 0  // 使用裁判系统键盘
```

### 如果使用遥控器DBUS键盘（需要硬件支持）
```c
// Referee_Task.h
#define USE_REMOTER_KEYBOARD 1  // 使用遥控器DBUS键盘
```

## 快速测试

1. 将 `USE_REMOTER_KEYBOARD` 改为 `0`
2. 重新编译
3. 使用图传链路或裁判系统连接键盘
4. 按 B 键切换到 Keyboard 模式
5. 按 WASD 测试底盘移动

## 数据流程图

```
DBUS模式（遥控器摇杆）:
remoter.ch1/ch2/ch3 → Chassis_Normal_Mode/Upstairs_Mode → 底盘电机

Keyboard模式（键盘鼠标）:
[裁判系统] kb_info ←┐
                     ├→ Referee_GetActiveKeyboard() → Chassis_Keyboard_Mode → 底盘电机
[遥控器DBUS] remoter.keyboard ←┘
                     (由 USE_REMOTER_KEYBOARD 宏选择)
```
