#ifndef __CAN_STRUCT_H__
#define __CAN_STRUCT_H__

#include "main.h"
#include "FreeRTOS.h"
#include "list.h"


// 定义CAN端口的枚举类型
typedef enum
{
    CAN1_PORT = 0,
    CAN2_PORT,
    CAN3_PORT,
} can_port_t;

// 定义CAN消息的结构体
typedef struct
{
    uint8_t port; // CAN1_ID, CAN2_ID, CAN3_ID
    uint32_t id;
    uint8_t data[8];
    uint8_t len;
    uint8_t rtr;
}can_msg_t;

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

#endif /* __CAN_STRUCT_H__ */