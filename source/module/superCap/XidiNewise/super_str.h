#ifndef _SUPER_STR_H_
#define _SUPER_STR_H_

#include "stdint.h"
#include "can_struct.h"
#include "stdbool.h"

typedef enum
{
    Super_C_Tsg_ID = 0x210,
    Super_C_Msg_ID = 0x200,
} can_msg_id_e;

typedef struct
{
    float Voilt_In;
    float Voilt_C;
    float Current_In;
    float Target_power;
} Super_C_Msg;
typedef struct
{
    can_msg_t super_can_send;    // 发送的 CAN 消息
    can_msg_t super_can_receive; // 接收到的 CAN 消息
    Super_C_Msg super_can_solve; // 解析后的 CAN 消息

} Super_TR_t;

bool Super_TR_Init(Super_TR_t *super, uint32_t can_id, uint32_t can_send_id, can_port_t can_port);

bool Super_TR_Send(Super_TR_t *super, uint16_t data);

bool Super_TR_Refresh(Super_TR_t *super);

float get_Voilt_In(Super_TR_t *super);
float get_Voilt_C(Super_TR_t *super);
float get_Current_In(Super_TR_t *super);
float get_Target_power(Super_TR_t *super);

#endif