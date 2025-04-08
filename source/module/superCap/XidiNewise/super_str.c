/**
 * @file super_str.c
 * @brief 超级电容模块相关功能实现。
 * 
 * 本文件实现了超级电容模块的初始化、数据发送、数据刷新以及数据获取功能。
 * 主要包括以下功能：
 * - 初始化超级电容模块的CAN通信参数。
 * - 发送控制数据到超级电容模块。
 * - 刷新并解析从超级电容模块接收到的数据。
 * - 提供接口函数以获取超级电容模块的关键参数（如输入电压、电容电压、输入电流和目标功率）。
 */

#include "can_api.h"
#include "can_bsp.h"
#include "super_str.h"
#include "string.h"

/**
 * @brief 超级电容模块初始化。
 * 
 * 初始化超级电容模块的CAN接收和发送消息结构体，并将其添加到CAN消息队列中。
 * 
 * @param super 指向Super_TR_t结构体的指针，表示超级电容模块实例。
 * @param can_id 接收CAN消息的ID。
 * @param can_send_id 发送CAN消息的ID。
 * @param can_port CAN端口类型。
 * @return true 初始化成功。
 * @return false 初始化失败（例如参数super为空）。
 */
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

/**
 * @brief 超级电容模块发送数据。
 * 
 * 将16位数据拆分为高字节和低字节，并通过CAN总线发送。
 * 
 * @param super 指向Super_TR_t结构体的指针，表示超级电容模块实例。
 * @param data 需要发送的16位数据。
 * @return true 发送成功。
 * @return false 发送失败（例如参数super为空）。
 */
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

/**
 * @brief 超级电容模块刷新数据。
 * 
 * 解析从CAN总线接收到的数据，并将其存储到super_can_solve结构体中。
 * 
 * @param super 指向Super_TR_t结构体的指针，表示超级电容模块实例。
 * @return true 刷新成功。
 * @return false 刷新失败（例如参数super为空）。
 */
bool Super_TR_Refresh(Super_TR_t *super)
{
    static uint8_t count = 0;
    if (super == NULL)
    {
        return false; // 检查指针是否为空
    }

    if (super->super_can_receive.cnt == count)
    {
        return true; // 避免反复解算
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

/**
 * @brief 获取超级电容模块的输入电压。
 * 
 * @param super 指向Super_TR_t结构体的指针，表示超级电容模块实例。
 * @return float 输入电压值（单位：V）。
 */
float get_Voilt_In(Super_TR_t *super)
{
    return super->super_can_solve.Voilt_In;
}

/**
 * @brief 获取超级电容模块的电容电压。
 * 
 * @param super 指向Super_TR_t结构体的指针，表示超级电容模块实例。
 * @return float 电容电压值（单位：V）。
 */
float get_Voilt_C(Super_TR_t *super)
{
    return super->super_can_solve.Voilt_C;
}

/**
 * @brief 获取超级电容模块的输入电流。
 * 
 * @param super 指向Super_TR_t结构体的指针，表示超级电容模块实例。
 * @return float 输入电流值（单位：A）。
 */
float get_Current_In(Super_TR_t *super)
{
    return super->super_can_solve.Current_In;
}

/**
 * @brief 获取超级电容模块的目标功率。
 * 
 * @param super 指向Super_TR_t结构体的指针，表示超级电容模块实例。
 * @return float 目标功率值（单位：W）。
 */
float get_Target_power(Super_TR_t *super)
{
    return super->super_can_solve.Target_power;
}