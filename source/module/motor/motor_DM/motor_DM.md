# 达妙电机说明文档

## 1. 代码示例

```c
void task()
{
    // 定义一个 DM_motor_t 类型的 motor 结构体
    DM_motor_t *DM_motor = pvPortMalloc(sizeof(DM_motor_t));

    // 设置 CAN 配置的 ID
    DM_motor->can_cfg.id = 0x211;

    // 设置电机消息的 CAN ID
    DM_motor->motor_msg.can_msg.id = 0x21;

    // 设置 CAN 端口
    DM_motor->can_cfg.port = CAN1_PORT;

    // 初始化电机
    Motor_DM_Init(&DM_motor);

    // 启用电机
    Motor_DM_Enable(&DM_motor);

    // 无限循环
    while (true)
    {
        // 刷新电机状态
        Motor_DM_Refresh(&DM_motor);

        // 获取电机角度
        angle = DM_motor.motor_msg.motor_angle;

        // 控制电机的位置和速度
        PosSpeed_CtrlMotor(&DM_motor, 233, 114);
    }
}

```
