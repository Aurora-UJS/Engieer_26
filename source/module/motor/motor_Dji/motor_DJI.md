# 大疆电机模块使用说明

## 1. 示例代码

```C
{
    // 定义一个指向 DJI_motor_t 类型的指针 DJMotor
    DJI_motor_t *DJMotor;

    // 动态分配内存，以创建一个 DJI_motor_t 类型的实例，并将其地址赋给 DJMotor
    DJMotor = pvPortMalloc(sizeof(DJI_motor_t));

    // 配置 DJMotor 的 CAN 端口，设置为 CAN1_PORT
    DJMotor->can_cfg.port = CAN1_PORT;

    // 配置 DJMotor 的 CAN 通信 ID，设置为 0x200
    DJMotor->can_cfg.id = 0x200;
    
    // 配置电机消息的 CAN ID，依次设置为 0x201, 0x202, 0x203, 0x204
    DJMotor->motor_msg.can_msg[0].id = 0x201;
    DJMotor->motor_msg.can_msg[1].id = 0x202;
    DJMotor->motor_msg.can_msg[2].id = 0x203;
    DJMotor->motor_msg.can_msg[3].id = 0x204;

    // 调用初始化函数，对 DJI_motor 进行初始化设置
    Motor_DJI_Init(DJMotor);

    // 开始无限循环，持续控制电机
    while (1)
    {
        // 刷新 DJMotor 状态，以获取最新的电机数据
        Motor_DJI_Refresh(DJMotor);

        // 设置电机的电压，参数解释为 CAN 端口，CAN ID 和多个电压值
        set_motor_voltage(&DJMotor, CAN1_PORT, 0x200, 1000, 1000, 1000, 1000);
    }
}

```
