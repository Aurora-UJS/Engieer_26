#include "uart_api.h"
#include "usart.h"
#include "string.h"



uart_rx_t* uart5_msg;
uart_rx_t* uart7_msg;
uart_rx_t* uart1_msg;
uart_rx_t* uart2_msg;
uart_rx_t* uart3_msg;

uart_status_t uart_rx_init(uart_rx_t* uart_rx_msg)
{
    //安全效验
    if (uart_rx_msg == NULL)
    {
        return UART_ERROR;
    }
    else if (uart_rx_msg->rx_msg == NULL)
    {
        return UART_ERROR;
    }
    else if(uart_rx_msg->rx_msg->huart == NULL)
    {
        return UART_ERROR;
    }
    else if (uart_rx_msg->rx_msg->pBuffer == NULL)
    {
        return UART_ERROR;
    }
    else if (uart_rx_msg->rx_msg->Len == 0)
    {
        return UART_ERROR;
    }
    
    if (uart_rx_msg->rx_msg->huart->Instance == USART1)
    {
        uart1_msg = uart_rx_msg;
        
    }
    else if (uart_rx_msg->rx_msg->huart->Instance == USART2)
    {
        uart2_msg = uart_rx_msg;
    }
    else if (uart_rx_msg->rx_msg->huart->Instance == USART3)
    {
        uart3_msg = uart_rx_msg;
    }
    else if (uart_rx_msg->rx_msg->huart->Instance == UART5)
    {
        uart5_msg = uart_rx_msg;
    }
    else if (uart_rx_msg->rx_msg->huart->Instance == UART7)
    {
        uart7_msg = uart_rx_msg;
    }
    else
    {
        return UART_ERROR;
    }
    HAL_UARTEx_ReceiveToIdle_IT(uart_rx_msg->rx_msg->huart, uart_rx_msg->rx_msg->pBuffer, uart_rx_msg->rx_msg->Len * 2);
    return UART_OK;
}