#ifndef __CAN_API_H__
#define __CAN_API_H__

#include "main.h"
#include "FreeRTOS.h"
#include "list.h"

#include "can_struct.h"



#define CAN_MSG_Get_PORT(msg) ((msg)->port)
#define CAN_MSG_Get_ID(msg) ((msg)->id)
#define CAN_MSG_Get_DATA(msg) ((msg)->*data)
#define CAN_MSG_Get_LEN(msg) ((msg)->len)
#define CAN_MSG_Get_RTR(msg) ((msg)->rtr)
// 初始化CAN系统的函数
void can_sys_init(void);

// 根据CAN端口查找相应的消息列表(可以不用)
List_t *can_msg_find_list(uint8_t port);
// 在给定的消息列表中查找特定ID的消息项
can_msg_t *can_msg_find_item(List_t *can_msg_list, uint32_t id);
// 向消息列表中添加消息项
uint8_t can_msg_add_item(can_msg_t *msg);
// 从消息列表中删除指定ID的消息项
uint8_t can_msg_del_item(uint8_t port, uint32_t id);

// 发送经典CAN消息
uint8_t can_msg_send_classical(can_msg_t *msg);

// 发送扩展CAN消息
uint8_t can_msg_send_extended(can_msg_t *msg);

// 发送fdCAN消息
uint8_t can_msg_send_fd(can_msg_t *msg);

#endif /* __CAN_API_H__ */