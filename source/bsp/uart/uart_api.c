#include "uart_api.h"
#include "usart.h"
#include "string.h"

#include <stdio.h>
#include <stdlib.h>

uart_rx_t *uart5_msg;
uart_rx_t *uart7_msg;
uart_rx_t *uart1_msg;
uart_rx_t *uart2_msg;
uart_rx_t *uart3_msg;
uart_rx_t *uart10_msg;

void (*uart5_rx_hook)(uint8_t *pData, uint32_t size) = NULL;
void (*uart7_rx_hook)(uint8_t *pData, uint32_t size) = NULL;
void (*uart1_rx_hook)(uint8_t *pData, uint32_t size) = NULL;
void (*uart2_rx_hook)(uint8_t *pData, uint32_t size) = NULL;
void (*uart3_rx_hook)(uint8_t *pData, uint32_t size) = NULL;
void (*uart10_rx_hook)(uint8_t *pData, uint32_t size) = NULL;

uart_status_t uart_rx_cheak(uart_rx_t *uart_rx_msg)
{
    // 安全效验
    if (uart_rx_msg == NULL)
    {
        return UART_ERROR;
    }
    else if (uart_rx_msg->rx_msg == NULL)
    {
        return UART_ERROR;
    }
    else if (uart_msg_cheak(uart_rx_msg->rx_msg) == UART_ERROR)
    {
        return UART_ERROR;
    }
    return UART_OK;
}
uart_status_t uart_msg_cheak(uart_msg_t *rx_msg)
{
    if (rx_msg->huart == NULL)
    {
        return UART_ERROR;
    }
    else if (rx_msg->pBuffer == NULL)
    {
        return UART_ERROR;
    }
    else if (rx_msg->Len == 0)
    {
        return UART_ERROR;
    }
    return UART_OK;
}

uart_status_t uart_reg_cheak(uart_rx_t *uart_rx_msg)
{

    // 安全效验
    if (uart_rx_msg->rx_msg->huart->Instance == USART1)
    {
        if (uart1_msg != NULL)
        {
            return UART_ERROR;
        }
    }
    else if (uart_rx_msg->rx_msg->huart->Instance == USART2)
    {
        if (uart2_msg != NULL)
        {
            return UART_ERROR;
        }
    }
    else if (uart_rx_msg->rx_msg->huart->Instance == USART3)
    {
        if (uart3_msg != NULL)
        {
            return UART_ERROR;
        }
    }
    else if (uart_rx_msg->rx_msg->huart->Instance == UART5)
    {
        if (uart5_msg != NULL)
        {
            return UART_ERROR;
        }
    }
    else if (uart_rx_msg->rx_msg->huart->Instance == UART7)
    {
        if (uart7_msg != NULL)
        {
            return UART_ERROR;
        }
    }
    return UART_OK;
}

uart_status_t uart_rx_init(uart_rx_t *uart_rx_msg)
{
    // 安全效验
    if (uart_rx_cheak(uart_rx_msg) == UART_ERROR)
    {
        return UART_ERROR;
    }

    if (uart_reg_cheak(uart_rx_msg) == UART_ERROR)
    {
        return UART_ERROR;
    }

    // 初始化串口接收
    switch ((unsigned long)uart_rx_msg->rx_msg->huart->Instance)
    {
    case (unsigned long)USART1_BASE:
        uart1_msg = uart_rx_msg;
        break;
    case (unsigned long)USART2_BASE:
        uart2_msg = uart_rx_msg;
        break;
    case (unsigned long)USART3_BASE:
        uart3_msg = uart_rx_msg;
        break;
    case (unsigned long)UART5_BASE:
        uart5_msg = uart_rx_msg;
        break;
    case (unsigned long)UART7_BASE:
        uart7_msg = uart_rx_msg;
        break;
    default:
        return UART_ERROR;
    }

    HAL_UARTEx_ReceiveToIdle_IT(uart_rx_msg->rx_msg->huart, uart_rx_msg->rx_msg->pBuffer, uart_rx_msg->rx_msg->Len);
    return UART_OK;
}

uart_status_t uart_tx_send_IT(uart_msg_t *uart_msg)
{
    // 安全效验
    if (uart_msg_cheak(uart_msg) == UART_ERROR)
    {
        return UART_ERROR;
    }
    // 发送数据
    HAL_UART_Transmit_IT(uart_msg->huart, uart_msg->pBuffer, uart_msg->Len);

    return UART_OK;
}

uart_status_t uart_rx_hook_reg(uart_rx_t *uart_rx_msg, void (*hook)(uint8_t *pData, uint32_t size))
{
    // 安全效验
    if (uart_rx_cheak(uart_rx_msg) == UART_ERROR)
    {
        return UART_ERROR;
    }
    // 注册串口接收钩子函数
    switch ((unsigned long)uart_rx_msg->rx_msg->huart->Instance)
    {
    case (unsigned long)USART1_BASE:
        uart1_rx_hook = hook;
        break;
    case (unsigned long)USART2_BASE:
        uart2_rx_hook = hook;
        break;
    case (unsigned long)USART3_BASE:
        uart3_rx_hook = hook;
        break;
    case (unsigned long)UART5_BASE:
        uart5_rx_hook = hook;
        break;
    case (unsigned long)UART7_BASE:
        uart7_rx_hook = hook;
        break;
    case (unsigned long)USART10_BASE:
        uart10_rx_hook = hook;
        break;
    default:
        return UART_ERROR;
    }

    return UART_OK;
}