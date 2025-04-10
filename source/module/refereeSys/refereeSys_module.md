# 裁判系统模块说明文档

## 模块概述
裁判系统模块负责处理与RoboMaster比赛裁判系统的通信，包括：
- 比赛状态数据解析
- 机器人状态数据上报
- 自定义控制器数据交互
- 图形化UI数据发送

## 主要功能
1. **数据解析**：
   - 裁判系统数据帧解析
   - 自定义控制器数据解析
   - CRC校验检查
   - 多帧数据包处理

2. **数据上报**：
   - 机器人状态数据打包
   - 自定义UI数据打包
   - 数据帧序列号管理
   - UART数据发送

3. **数据获取**：
   - 比赛状态实时获取
   - 机器人血量/状态获取
   - 自定义控制器数据获取

## 关键数据结构

### 裁判系统数据结构
```c
typedef struct {
    xFrameHeader FrameHeader; // 帧头信息
    uint16_t CmdID;          // 命令ID
    ext_game_state_t GameState;      // 比赛状态
    ext_game_result_t GameResult;    // 比赛结果
    ext_game_robot_HP_t GameRobotHP; // 机器人血量
    ext_event_data_t EventData;      // 事件数据
    ext_supply_projectile_action_t SupplyProjectileAction; // 补给动作
    ext_game_robot_state_t GameRobotState; // 机器人状态
    ext_power_heat_data_t PowerHeatData;   // 功率热量数据
    ext_game_robot_pos_t GameRobotPos;     // 机器人位置
    ext_buff_musk_t BuffMusk;              // 增益状态
    aerial_robot_energy_t AerialRobotEnergy; // 空中机器人能量
    ext_robot_hurt_t RobotHurt;            // 伤害数据
    ext_shoot_data_t ShootData;            // 发射数据
    Communicate_ReceiveData_t ReceiveData; // 自定义接收数据
    uint8_t init_flag;                     // 初始化标志
} referee_info_t;
```

### 自定义控制器数据结构
```c
typedef struct {
    xFrameHeader FrameHeader; // 帧头信息
    uint16_t CmdID;          // 命令ID
    custom_controller_data_t CustomController; // 自定义控制器数据
    keyboard_t keyboard;     // 键鼠数据
} custom_controller_info_t;
```

## 主要API说明

### `referee_init`
初始化裁判系统UART接收
```c
void referee_init(UART_HandleTypeDef *huart);
```

### `JudgeReadData`
裁判系统数据解析
```c
void JudgeReadData(uint8_t *buff);
```

### `get_referee_msg`
获取裁判系统消息
```c
referee_info_t *get_referee_msg(void);
```

### `CtrllerReadData`
自定义控制器数据解析
```c
void CtrllerReadData(uint8_t *buff);
```

### `get_custom_controller_msg`
获取自定义控制器消息
```c
custom_controller_info_t *get_custom_controller_msg(void);
```

### `referee_data_pack_handle`
数据打包发送
```c
void referee_data_pack_handle(
    uint8_t sof,      // 帧头标识符
    uint16_t cmd_id,  // 命令ID
    uint8_t *p_data,  // 数据指针
    uint16_t len      // 数据长度
);
```

## 使用示例
```c
// 初始化
referee_init(&huart1);

// 主循环中获取裁判数据
while(1) {
    referee_info_t *referee = get_referee_msg();
    
    // 使用裁判数据
    if(referee->CmdID == ID_game_state) {
        // 处理比赛状态...
    }
    
    // 发送自定义数据
    uint8_t custom_data[128];
    // 填充自定义数据...
    referee_data_pack_handle(0xA5, 0x0301, custom_data, sizeof(custom_data));
    
    HAL_Delay(10);
}
