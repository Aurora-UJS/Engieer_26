#include "main.h"
#include "cmsis_os2.h"
#include "DBusSys.h"
#include "motor_DM.h"
#include "motor_DJI.h"
#include "PIDtool.h"


DM_motor_t *DM_motor_8009_Down, *DM_motor_8009_Up, *DM_motor_6220;
DJI_motor_t *DJ_motor_2006_Left, DJ_motor_2006_Right;
float Pos_Down = 0, Pos_Up = 0, Pos_Spin = 0, end_angle = 0;




float limit(float value, float min, float max)
{
    // 检查 min 是否大于 max
    if (min > max)
    {
        return NAN; // 返回 NaN 表示无效输入
    }

    // 检查是否为 NaN
    if (isnan(value) || isnan(min) || isnan(max))
    {
        return NAN;
    }

    // 使用 isless 和 isgreater 避免浮点数比较的精度问题
    if (isless(value, min))
    {
        return min;
    }
    else if (isgreater(value, max))
    {
        return max;
    }
    else
    {
        return value;
    }
}

/**
 * @brief 线性映射函数
 * @param value 要映射的值
 * @param in_min 输入范围的最小值
 * @param in_max 输入范围的最大值
 * @param out_min 输出范围的最小值
 * @param out_max 输出范围的最大值
 * @return 映射后的值
 */
float map(float value, float in_min, float in_max, float out_min, float out_max)
{
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void DM_motor_init()
{
    // 定义一个 DM_motor_t 类型的 motor 结构体
    DM_motor_8009_Down = pvPortMalloc(sizeof(DM_motor_t));

    // 设置 CAN 配置的 ID
    DM_motor_8009_Down->can_cfg.id = 0x101;

    // 设置电机消息的 CAN ID
    DM_motor_8009_Down->motor_msg.can_msg.id = 0x11;

    // 设置 CAN 端口
    DM_motor_8009_Down->can_cfg.port = CAN2_PORT;

    // 定义一个 DM_motor_t 类型的 motor 结构体
    DM_motor_8009_Up = pvPortMalloc(sizeof(DM_motor_t));

    // 设置 CAN 配置的 ID
    DM_motor_8009_Up->can_cfg.id = 0x102;

    // 设置电机消息的 CAN ID
    DM_motor_8009_Up->motor_msg.can_msg.id = 0x12;

    // 设置 CAN 端口
    DM_motor_8009_Up->can_cfg.port = CAN2_PORT;

    DM_motor_6220 = pvPortMalloc(sizeof(DM_motor_t));

    // 设置 CAN 配置的 ID
    DM_motor_6220->can_cfg.id = 0x103;

    // 设置电机消息的 CAN ID
    DM_motor_6220->motor_msg.can_msg.id = 0x13;

    // 设置 CAN 端口
    DM_motor_6220->can_cfg.port = CAN2_PORT;

    // 初始化电机
    DJ_motor_2006_Left = pvPortMalloc(sizeof(DJI_motor_t));
    DJ_motor_2006_Left->can_cfg.id = 0x200;
    DJ_motor_2006_Left->motor_msg[0].can_msg.id = 0x201;
    DJ_motor_2006_Left->can_cfg.port = CAN1_PORT;

    for (uint8_t i = 0; i < 4; i++)
    {
        Motor_DM_Init(DM_motor_8009_Down);
        Motor_DM_Init(DM_motor_8009_Up);
        Motor_DM_Init(DM_motor_6220);

        Motor_DM_Enable(DM_motor_8009_Up);
        Motor_DM_Enable(DM_motor_8009_Down);
        Motor_DM_Enable(DM_motor_6220);
    }

    // // 启用电机
    // Motor_DM_Enable(DM_motor_6220);
    osDelay(800);
}
void motor_Reset()
{
    PosSpeed_CtrlMotorDM(DM_motor_8009_Down, 0, 1);
    PosSpeed_CtrlMotorDM(DM_motor_8009_Up, 0, 1);
    PosSpeed_CtrlMotorDM(DM_motor_6220, 0, 1);
}
void motor_8009_down_movement()
{
    if (remoter.ch2 > 250 /* condition */)
    {
        Pos_Down -= map(remoter.ch2, 200, 660, 0.005, 0.01);
        Pos_Down = limit(Pos_Down, -4, 0);
        PosSpeed_CtrlMotorDM(DM_motor_8009_Down, Pos_Down, 0.5); /* code */
    }
    else if (remoter.ch2 < -250)
    {
        Pos_Down += map(remoter.ch2, -200, -660, 0.01, 0.01);
        Pos_Down = limit(Pos_Down, -4, 0);
        PosSpeed_CtrlMotorDM(DM_motor_8009_Down, Pos_Down, 0.5); /* code */
    }
    else
    {
        PosSpeed_CtrlMotorDM(DM_motor_8009_Down, Pos_Down, 0.5); /* code */
    }
}
void motor_8009_up_movement()
{

    if (remoter.ch4 > 250 /* condition */)
    {
        Pos_Up += map(remoter.ch4, 200, 660, 0.005, 0.01);
        Pos_Up = limit(Pos_Up, 0, 2.7);
        PosSpeed_CtrlMotorDM(DM_motor_8009_Up, Pos_Up, 0.5); /* code */
    }
    else if (remoter.ch4 < -250)
    {
        Pos_Up -= map(remoter.ch4, -200, -660, 0.005, 0.01);
        ;
        Pos_Up = limit(Pos_Up, 0, 2.7);
        PosSpeed_CtrlMotorDM(DM_motor_8009_Up, Pos_Up, 0.5); /* code */
    }
    else
    {
        PosSpeed_CtrlMotorDM(DM_motor_8009_Up, Pos_Up, 0.5); /* code */
    }
}
void motor_spin_movement()
{

    if (remoter.ch1 > 250 /* condition */)
    {
        Pos_Spin += map(remoter.ch4, 200, 660, 0.005, 0.03);
        PosSpeed_CtrlMotorDM(DM_motor_6220, Pos_Spin, 0.5); /* code */
    }
    else if (remoter.ch1 < -250)
    {
        Pos_Spin -= map(remoter.ch4, -200, -660, 0.005, 0.03);
        PosSpeed_CtrlMotorDM(DM_motor_6220, Pos_Spin, 0.5); /* code */
    }
    else
    {
        PosSpeed_CtrlMotorDM(DM_motor_6220, Pos_Spin, 0.5); /* code */
    }
}



void motor_Zero()
{
    Pos_Down = 0;
    Pos_Up = 0;
    Pos_Spin = 0;

    PosSpeed_CtrlMotorDM(DM_motor_8009_Up, Pos_Up, 1);     /* code */
    PosSpeed_CtrlMotorDM(DM_motor_8009_Down, Pos_Down, 1); /* code */
    PosSpeed_CtrlMotorDM(DM_motor_6220, Pos_Spin, 1);      /* code */
}