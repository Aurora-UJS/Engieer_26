#include "main.h"
#include "cmsis_os2.h"
#include "DBusSys.h"
#include "motor_DM.h"

#define Multiple               2.356    //目标值乘的倍数
DM_motor_t *DM_motor_8009_Down ,*DM_motor_8009_Up;
void Remoter_Task(void const *argument)
{
    UNUSED(argument);
    osDelay(1000);
        // 定义一个 DM_motor_t 类型的 motor 结构体
    DM_motor_8009_Down = pvPortMalloc(sizeof(DM_motor_t));

    // 设置 CAN 配置的 ID
    DM_motor_8009_Down->can_cfg.id = 0x101;

    // 设置电机消息的 CAN ID
    DM_motor_8009_Down->motor_msg.can_msg.id = 0x11;

    // 设置 CAN 端口
    DM_motor_8009_Down->can_cfg.port = CAN2_PORT;

    // 初始化电机
    Motor_DM_Init(DM_motor_8009_Down);

    // 启用电机
    Motor_DM_Enable(DM_motor_8009_Down);
            // 定义一个 DM_motor_t 类型的 motor 结构体
    DM_motor_8009_Up = pvPortMalloc(sizeof(DM_motor_t));

    // 设置 CAN 配置的 ID
    DM_motor_8009_Up->can_cfg.id = 0x02;

    // 设置电机消息的 CAN ID
    DM_motor_8009_Up->motor_msg.can_msg.id = 0x12;

    // 设置 CAN 端口
    DM_motor_8009_Up->can_cfg.port = CAN1_PORT;

    // 初始化电机
    Motor_DM_Init(DM_motor_8009_Up);

    // 启用电机
    Motor_DM_Enable(DM_motor_8009_Up);

    DBus_Init();
     PosSpeed_CtrlMotorDM(DM_motor_8009_Down, 0, 1.5);
    while (1)
    {
        DBus_Refresh();
        Motor_DM_Refresh(DM_motor_8009_Down);     // 刷新电机状态
        Motor_DM_Refresh(DM_motor_8009_Up);     // 刷新电机状态
        if(remoter.sw1==2&&remoter.sw2==1)      
        {

        PosSpeed_CtrlMotorDM(DM_motor_8009_Down, (remoter.ch2/660.0)*Multiple, 2);

        }  


        osDelay(10);
    }
}
