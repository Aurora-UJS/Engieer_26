#include "can_struct.h"
#include "can_api.h"
#include "can_bsp.h"
#include "fdcan.h"
#include "string.h"
#include "stdio.h"
#include "FreeRTOS.h"

List_t can1_rx_msg_list;
List_t can2_rx_msg_list;
List_t can3_rx_msg_list;

void can_sys_init(void)
{
    vListInitialise(&can1_rx_msg_list);
    vListInitialise(&can2_rx_msg_list);
    vListInitialise(&can3_rx_msg_list);
    can_bsp_init();
}

List_t *can_msg_find_list(uint8_t port)
{
    switch (port)
    {
    case CAN1_PORT:
        return &can1_rx_msg_list;
    case CAN2_PORT:
        return &can2_rx_msg_list;
    case CAN3_PORT:
        return &can3_rx_msg_list;
    default:
        return &can1_rx_msg_list;
    }
}

can_msg_t *can_msg_find_item(List_t *can_msg_list, uint32_t id)
{
    ListItem_t *item = listGET_HEAD_ENTRY(can_msg_list);
    while (item != listGET_END_MARKER(can_msg_list))
    {
        can_msg_t *msg = listGET_LIST_ITEM_OWNER(item);
        if (msg->id == id)
        {
            return msg;
        }
        item = listGET_NEXT(item);
    }
    return NULL;
}

uint8_t can_msg_add_item(can_msg_t *msg)
{
    if (msg == NULL)
        return CAN_MSG_ERROR;

    if (can_msg_find_item(can_msg_find_list(msg->port), msg->id) == NULL)
        return CAM_MSG_REPEAT;

    ListItem_t *can_item = pvPortMalloc(sizeof(ListItem_t));

    if (can_item == NULL)
        return CAN_Reg_ERROR;

    vListInitialiseItem(can_item);
    listSET_LIST_ITEM_VALUE(can_item, msg->id);
    listSET_LIST_ITEM_OWNER(can_item, msg);

    switch (msg->port)
    {
    case CAN1_PORT:
        vListInsert(&can1_rx_msg_list, can_item);
        break;
    case CAN2_PORT:
        vListInsert(&can2_rx_msg_list, can_item);
        break;
    case CAN3_PORT:
        vListInsert(&can3_rx_msg_list, can_item);
        break;
    default:
        vPortFree(can_item);
        return CAN_PORT_ERROR;
    }
    return CAN_OK;
}

uint8_t can_msg_del_item(uint8_t port, uint32_t id)
{
    ListItem_t *item = listGET_HEAD_ENTRY(can_msg_find_list(port));
    const ListItem_t *end = listGET_END_MARKER(can_msg_find_list(port));
    while (item != end)
    {
        can_msg_t *msg = listGET_LIST_ITEM_OWNER(item);
        if (msg->id == id)
        {
            uxListRemove(item);
            vPortFree(item);
            return CAN_OK;
        }
        item = listGET_NEXT(item);
    }
    return CAN_MSG_NOT_FOUND;
}

uint8_t can_msg_send_classical(can_msg_t *msg)
{
    if (msg == NULL)
        return CAN_MSG_ERROR;

    switch (msg->port)
    {
    case CAN1_PORT:
        fdcanx_send_data(&hfdcan1, msg->id, msg->data, msg->len);
        break;
    case CAN2_PORT:
        fdcanx_send_data(&hfdcan2, msg->id, msg->data, msg->len);
        break;
    case CAN3_PORT:
        fdcanx_send_data(&hfdcan3, msg->id, msg->data, msg->len);
        break;
    default:
        return CAN_PORT_ERROR;
    }    
    return CAN_OK;
}

uint8_t can_msg_send_extended(can_msg_t *msg)
{
    if (msg == NULL)
        return CAN_MSG_ERROR;
    switch (msg->port)
    {
    case CAN1_PORT:
        fdcanx_send_data_ext(&hfdcan1, msg->id, msg->data, msg->len);
        break;
    case CAN2_PORT:
        fdcanx_send_data_ext(&hfdcan2, msg->id, msg->data, msg->len);
        break;
    case CAN3_PORT:
        fdcanx_send_data_ext(&hfdcan3, msg->id, msg->data, msg->len);
        break;
    default:
        return CAN_PORT_ERROR;
    }    
    return CAN_OK;
}

uint8_t can_msg_send_fd(can_msg_t *msg)
{
    if (msg == NULL)
        return CAN_MSG_ERROR;
    switch (msg->port)
    {
    case CAN1_PORT:
        fdcanx_send_data_fd(&hfdcan1, msg->id, msg->data, msg->len);
        break;
    case CAN2_PORT:
        fdcanx_send_data_fd(&hfdcan2, msg->id, msg->data, msg->len);
        break;
    case CAN3_PORT:    
        fdcanx_send_data_fd(&hfdcan3, msg->id, msg->data, msg->len);
        break;
    default:
        return CAN_PORT_ERROR;
    }    
    return CAN_OK;
}