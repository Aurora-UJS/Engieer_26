#include "can_api.h"
#include "can_bsp.h"
#include "super_str.h"
#include "string.h"

// 初始化 Super_TR_Manager
bool Super_TR_Init(Super_TR_t *super, uint32_t can_id, uint32_t can_send_id, can_port_t can_port)
{
    if (super == NULL)
    {
        return false; // 检查指针是否为空
    }
    // 初始化发送和接收的消息
    memset(&super, 0, sizeof(super));

    // 设置 接收CAN 消息数据
    super->super_can_receive.id = can_id;     // 设置 CAN ID
    super->super_can_receive.port = can_port; // 设置 CAN 端口
    can_msg_add_item(&super->super_can_receive);

    super->super_can_send.port = can_port;         // 设置 CAN 端口
    super->super_can_send.id = can_send_id;        // 设置 CAN ID
    super->super_can_send.len = FDCAN_DLC_BYTES_8; // 数据长度为 8 字节
    return true;                                   // 初始化成功
}

// 发送 CAN 消息
bool Super_TR_Send(Super_TR_t *super, uint16_t data)
{
    if (super == NULL)
    {
        return false; // 检查指针是否为空
    }
    memset(super->super_can_send.data, 0, sizeof(super->super_can_send.data));
    // 设置 CAN 消息数据
    super->super_can_send.len = FDCAN_DLC_BYTES_8;                 // 数据长度为 2 字节
    super->super_can_send.data[0] = (uint8_t)((data >> 8) & 0xFF); // 高字节
    super->super_can_send.data[1] = (uint8_t)(data & 0xFF);        // 低字节

    // 发送 CAN 消息
    can_msg_send_classical(&super->super_can_send);

    return true; // 发送成功
}

// 接收并解析 CAN 消息（带超时机制）
bool Super_TR_Refresh(Super_TR_t *super)
{
    static uint8_t count = 0;
    if (super == NULL)
    {
        return false; // 检查指针是否为空
    }

    if (super->super_can_receive.cnt == count)
    {
        return true;//避免反复解算
    }
    count = super->super_can_receive.cnt;

    uint16_t Voilt_In, Voilt_C, Current_In, Target_power;

    // 解析接收到的数据
    Voilt_In = (uint16_t)(((uint16_t)(super->super_can_receive.data[1]) << 8) | (uint16_t)(super->super_can_receive.data[0]));
    Voilt_C = (uint16_t)(((uint16_t)(super->super_can_receive.data[3]) << 8) | (uint16_t)(super->super_can_receive.data[2]));
    Current_In = (uint16_t)(((uint16_t)(super->super_can_receive.data[5]) << 8) | (uint16_t)(super->super_can_receive.data[4]));
    Target_power = (uint16_t)(((uint16_t)(super->super_can_receive.data[7]) << 8) | (uint16_t)(super->super_can_receive.data[6]));

    // 将解析后的数据赋值给 super_solve
    super->super_can_solve.Voilt_In = Voilt_In / 100.0f;
    super->super_can_solve.Voilt_C = Voilt_C / 100.0f;
    super->super_can_solve.Current_In = Current_In / 100.0f;
    super->super_can_solve.Target_power = Target_power / 100.0f;

    return true; // 接收并解析成功
}

float get_Voilt_In(Super_TR_t *super)
{
    return super->super_can_solve.Voilt_In;
}

float get_Voilt_C(Super_TR_t *super)
{
    return super->super_can_solve.Voilt_C;
}

float get_Current_In(Super_TR_t *super)
{
    return super->super_can_solve.Current_In;
}
float get_Target_power(Super_TR_t *super)
{
    return super->super_can_solve.Target_power;
}

