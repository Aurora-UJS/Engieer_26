#ifndef __UART_API_H__
#define __UART_API_H__

#include "main.h"

typedef struct
{
    UART_HandleTypeDef *huart;
    uint8_t *pBuffer;
    uint16_t Len;
}uart_msg_t;

typedef struct
{
    uart_msg_t *rx_msg;
    uint32_t count; //接收计数
    uint16_t time; //超时时间
}uart_rx_t;

typedef enum
{
    UART_OK = 0,
    UART_ERROR,
    UART_TIMEOUT
}uart_status_t;

extern uart_rx_t* uart5_msg;
extern uart_rx_t* uart7_msg;
extern uart_rx_t* uart1_msg;
extern uart_rx_t* uart2_msg;
extern uart_rx_t* uart3_msg;

uart_status_t uart_reg_cheak(uart_rx_t *uart_rx_msg);
uart_status_t uart_msg_cheak(uart_msg_t *rx_msg);
uart_status_t uart_rx_init(uart_rx_t* uart_rx_msg);

#endif /* __UART_API_H__ */