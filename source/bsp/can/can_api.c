#include "can_struct.h"
#include "can_api.h"
#include "can_bsp.h"
#include "fdcan.h"
#include "string.h"
#include "stdio.h"
#include "FreeRTOS.h"

// 定义三个CAN接收消息列表，分别对应CAN1, CAN2, CAN3端口
List_t can1_rx_msg_list;
List_t can2_rx_msg_list;
List_t can3_rx_msg_list;

// 初始化CAN系统，包括初始化三个CAN接收消息列表以及调用底层硬件初始化函数
void can_sys_init(void)
{
    vListInitialise(&can1_rx_msg_list);  // 初始化CAN1接收消息列表
    vListInitialise(&can2_rx_msg_list);  // 初始化CAN2接收消息列表
    vListInitialise(&can3_rx_msg_list);  // 初始化CAN3接收消息列表
    can_bsp_init();                      // 调用底层硬件初始化函数
}

// 根据CAN端口返回对应的接收消息列表指针
List_t *can_msg_find_list(can_port_t port)
{
    switch (port)
    {
    case CAN1_PORT:
        return &can1_rx_msg_list;  // 返回CAN1接收消息列表指针
    case CAN2_PORT:
        return &can2_rx_msg_list;  // 返回CAN2接收消息列表指针
    case CAN3_PORT:
        return &can3_rx_msg_list;  // 返回CAN3接收消息列表指针
    default:
        return &can1_rx_msg_list;  // 默认返回CAN1接收消息列表指针
    }
}

// 在指定的CAN消息列表中查找具有特定ID的消息项
can_msg_t *can_msg_find_item(List_t *can_msg_list, uint32_t id)
{
    ListItem_t *item = listGET_HEAD_ENTRY(can_msg_list);  // 获取列表头项
    while (item != listGET_END_MARKER(can_msg_list))       // 遍历列表直到结束标记
    {
        can_msg_t *msg = listGET_LIST_ITEM_OWNER(item);   // 获取当前项的所有者消息
        if (msg->id == id)
        {
            return msg;  // 找到匹配ID的消息项并返回
        }
        item = listGET_NEXT(item);  // 移动到下一项
    }
    return NULL;  // 未找到匹配消息项，返回NULL
}

// 向CAN消息列表中添加一个新的消息项
can_status_t can_msg_add_item(can_msg_t *msg)
{
    if (msg == NULL)
        return CAN_MSG_ERROR;  // 如果消息指针为空，返回错误状态

    // 检查消息是否已经存在于对应的CAN消息列表中
    if (can_msg_find_item(can_msg_find_list(msg->port), msg->id) != NULL)
        return CAN_MSG_REPEAT;  // 如果消息已存在，返回重复添加的消息状态

    // 分配内存为新的ListItem_t结构
    ListItem_t *can_item = pvPortMalloc(sizeof(ListItem_t));

    if (can_item == NULL)
        return CAN_Reg_ERROR;  // 如果内存分配失败，返回内存分配错误状态

    vListInitialiseItem(can_item);            // 初始化新ListItem_t项
    listSET_LIST_ITEM_VALUE(can_item, msg->id); // 设置ListItem_t项的值为消息ID
    listSET_LIST_ITEM_OWNER(can_item, msg);     // 设置ListItem_t项的所有者为消息结构指针

    switch (msg->port)
    {
    case CAN1_PORT:
        vListInsert(&can1_rx_msg_list, can_item);  // 将新项插入到CAN1接收消息列表中
        break;
    case CAN2_PORT:
        vListInsert(&can2_rx_msg_list, can_item);  // 将新项插入到CAN2接收消息列表中
        break;
    case CAN3_PORT:
        vListInsert(&can3_rx_msg_list, can_item);  // 将新项插入到CAN3接收消息列表中
        break;
    default:
        vPortFree(can_item);  // 如果端口未知，释放已分配的内存
        return CAN_PORT_ERROR;  // 返回端口错误状态
    }
    return CAN_OK;  // 成功添加消息项，返回OK状态
}

// 根据CAN端口和消息ID从接收消息列表中删除消息项
can_status_t can_msg_del_item(can_port_t port, uint32_t id)
{
    ListItem_t *item = listGET_HEAD_ENTRY(can_msg_find_list(port));  // 获取对应端口的列表头项
    const ListItem_t *end = listGET_END_MARKER(can_msg_find_list(port));  // 获取对应端口的列表结束标记
    while (item != end)
    {
        can_msg_t *msg = listGET_LIST_ITEM_OWNER(item);  // 获取当前项的所有者消息
        if (msg->id == id)
        {
            uxListRemove(item);  // 从列表中移除该项
            vPortFree(item);     // 释放ListItem_t项所占用的内存
            return CAN_OK;       // 成功删除消息项，返回OK状态
        }
        item = listGET_NEXT(item);  // 移动到下一项
    }
    return CAN_MSG_NOT_FOUND;  // 未找到匹配消息项，返回消息未找到状态
}

// 发送经典CAN格式的消息
can_status_t can_msg_send_classical(can_msg_t *msg)
{
    if (msg == NULL)
        return CAN_MSG_ERROR;  // 如果消息指针为空，返回错误状态

    switch (msg->port)
    {
    case CAN1_PORT:
        fdcanx_send_data(&hfdcan1, msg->id, msg->data, msg->len);  // 使用fdcanx_send_data发送CAN1端口的消息
        break;
    case CAN2_PORT:
        fdcanx_send_data(&hfdcan2, msg->id, msg->data, msg->len);  // 使用fdcanx_send_data发送CAN2端口的消息
        break;
    case CAN3_PORT:
        fdcanx_send_data(&hfdcan3, msg->id, msg->data, msg->len);  // 使用fdcanx_send_data发送CAN3端口的消息
        break;
    default:
        return CAN_PORT_ERROR;  // 如果端口未知，返回端口错误状态
    }    
    return CAN_OK;  // 成功发送消息，返回OK状态
}

// 发送扩展CAN格式的消息
can_status_t can_msg_send_extended(can_msg_t *msg)
{
    if (msg == NULL)
        return CAN_MSG_ERROR;  // 如果消息指针为空，返回错误状态

    switch (msg->port)
    {
    case CAN1_PORT:
        fdcanx_send_data_ext(&hfdcan1, msg->id, msg->data, msg->len);  // 使用fdcanx_send_data_ext发送CAN1端口的消息
        break;
    case CAN2_PORT:
        fdcanx_send_data_ext(&hfdcan2, msg->id, msg->data, msg->len);  // 使用fdcanx_send_data_ext发送CAN2端口的消息
        break;
    case CAN3_PORT:
        fdcanx_send_data_ext(&hfdcan3, msg->id, msg->data, msg->len);  // 使用fdcanx_send_data_ext发送CAN3端口的消息
        break;
    default:
        return CAN_PORT_ERROR;  // 如果端口未知，返回端口错误状态
    }    
    return CAN_OK;  // 成功发送消息，返回OK状态
}

// 发送CAN FD格式的消息
can_status_t can_msg_send_fd(can_msg_t *msg)
{
    if (msg == NULL)
        return CAN_MSG_ERROR;  // 如果消息指针为空，返回错误状态

    switch (msg->port)
    {
    case CAN1_PORT:
        fdcanx_send_data_fd(&hfdcan1, msg->id, msg->data, msg->len);  // 使用fdcanx_send_data_fd发送CAN1端口的消息
        break;
    case CAN2_PORT:
        fdcanx_send_data_fd(&hfdcan2, msg->id, msg->data, msg->len);  // 使用fdcanx_send_data_fd发送CAN2端口的消息
        break;
    case CAN3_PORT:    
        fdcanx_send_data_fd(&hfdcan3, msg->id, msg->data, msg->len);  // 使用fdcanx_send_data_fd发送CAN3端口的消息
        break;
    default:
        return CAN_PORT_ERROR;  // 如果端口未知，返回端口错误状态
    }    
    return CAN_OK;  // 成功发送消息，返回OK状态
}