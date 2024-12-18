# 大疆电机模块使用说明

## 1. 示例代码

```C
void DJI_Ctrl(void)
{
    DJI_motor_t *DJMotor;
    DJMotor = pvPortMalloc(sizeof(DJI_motor_t));
    DJMotor->can_cfg.port = CAN1_PORT;
    DJMotor->can_cfg.id = 0x200;
    
    DJMotor->motor_msg.can_msg[0].id = 0x201;
    DJMotor->motor_msg.can_msg[1].id = 0x202;
    DJMotor->motor_msg.can_msg[2].id = 0x203;
    DJMotor->motor_msg.can_msg[3].id = 0x204;

    Motor_DJI_Init(DJMotor);

    while(1)
    {

        Motor_DJI_Refresh(DJMotor);
        set_motor_voltage(&DJMotor,CAN1_PORT,0x200,1000,1000,1000,1000);
    }
}
```
