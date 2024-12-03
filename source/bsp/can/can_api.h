#ifndef __CAN_API_H__
#define __CAN_API_H__
 
#include "main.h"
 
 
// 定义CAN端口的枚举类型
typedef enum
{
    CAN1_PORT = 0,
    CAN2_PORT = 1,
    CAN3_PORT = 2
} can_port_t;
  
// 定义CAN消息的结构体
typedef struct
{
    can_port_t port; // CAN1_ID, CAN2_ID, CAN3_ID
    uint32_t id;
    uint8_t data[8];
    uint8_t len;
    uint8_t rtr;
} can_msg_t;
  
// 定义CAN状态的枚举类型
typedef enum
{
    CAN_OK = 0,
    CAN_Reg_ERROR = 1,
    CAN_PORT_ERROR = 2,
    CAN_MSG_ERROR = 3,
    CAM_MSG_REPEAT = 4,
    CAN_MSG_NOT_FOUND = 5,
 
} can_status_t;
  
extern List_t can1_rx_msg_list;
extern List_t can2_rx_msg_list;
extern List_t can3_rx_msg_list;

#define CAN_MSG_Get_PORT(msg) ((msg)->port)
#define CAN_MSG_Get_ID(msg) ((msg)->id)
#define CAN_MSG_Get_DATA(msg) ((msg)->*data)
#define CAN_MSG_Get_LEN(msg) ((msg)->len)
#define CAN_MSG_Get_RTR(msg) ((msg)->rtr)
// 初始化CAN系统的函数
void can_sys_init(void);
 
// 根据CAN端口查找相应的消息列表(可以不用)
List_t *can_msg_find_list(can_port_t port);
// 在给定的消息列表中查找特定ID的消息项
can_msg_t *can_msg_find_item(List_t *can_msg_list, uint32_t id);
// 向消息列表中添加消息项
can_status_t can_msg_add_item(can_msg_t *msg);
// 从消息列表中删除指定ID的消息项
can_status_t can_msg_del_item(can_port_t port, uint32_t id);

// 发送经典CAN消息
can_status_t can_msg_send_classical(can_msg_t *msg);

// 发送扩展CAN消息
can_status_t can_msg_send_extended(can_msg_t *msg);

// 发送fdCAN消息
can_status_t can_msg_send_fd(can_msg_t *msg);
 
#endif /* __CAN_API_H__ */