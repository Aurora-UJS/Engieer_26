#include "uart_api.h"
#include "usart.h"
#include "string.h"

uart_rx_t *uart5_msg;
uart_rx_t *uart7_msg;
uart_rx_t *uart1_msg;
uart_rx_t *uart2_msg;
uart_rx_t *uart3_msg;

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

uart_status_t uart_rx_init(uart_rx_t *uart_rx_msg)
{
    // 安全效验
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

    HAL_UARTEx_ReceiveToIdle_IT(uart_rx_msg->rx_msg->huart, uart_rx_msg->rx_msg->pBuffer, uart_rx_msg->rx_msg->Len * 2);
    return UART_OK;
}

uart_status_t uart_tx_send_IT(uart_msg_t *uart_msg)
{
    // 安全效验
    if (uart_msg == NULL)
    if (uart_msg_cheak(uart_msg) == UART_ERROR)
    {
        return UART_ERROR;
    }
    // 发送数据
    HAL_UART_Transmit_IT(uart_msg->huart, uart_msg->pBuffer, uart_msg->Len);

    return UART_OK;
}