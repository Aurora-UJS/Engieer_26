#include "uart_bsp.h"
#include "uart_api.h"
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "usart.h"

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	switch ((unsigned long)huart->Instance)
	{
	case (unsigned long)UART5_BASE:
		uart_rx_cheak(uart5_msg);
		uart5_msg->rx_msg->Len = Size;
		HAL_UARTEx_ReceiveToIdle_IT(uart5_msg->rx_msg->huart, uart5_msg->rx_msg->pBuffer, uart5_msg->rx_msg->Len * 2);
		uart5_msg->count++;
		uart5_msg->time = 0;
		if (uart5_rx_hook != NULL)
		{
			uart5_rx_hook(uart5_msg->rx_msg->pBuffer, Size); // 添加钩子函数调用
		}

		break;
	case (unsigned long)UART7_BASE:
		uart_rx_cheak(uart7_msg);
		uart7_msg->rx_msg->Len = Size;
		HAL_UARTEx_ReceiveToIdle_IT(uart7_msg->rx_msg->huart, uart7_msg->rx_msg->pBuffer, uart7_msg->rx_msg->Len * 2);
		uart7_msg->count++;
		uart7_msg->time = 0;
		if (uart7_rx_hook != NULL)
		{
			uart7_rx_hook(uart7_msg->rx_msg->pBuffer, Size);
		}
		break;
	case (unsigned long)USART1_BASE:
		uart_rx_cheak(uart1_msg);
		uart1_msg->rx_msg->Len = Size;
		HAL_UARTEx_ReceiveToIdle_IT(uart1_msg->rx_msg->huart, uart1_msg->rx_msg->pBuffer, uart1_msg->rx_msg->Len * 2);
		uart1_msg->count++;
		uart1_msg->time = 0;
		if (uart1_rx_hook != NULL)
		{
			uart1_rx_hook(uart5_msg->rx_msg->pBuffer, Size); // 添加钩子函数调用
		}
		break;
	case (unsigned long)USART2_BASE:
		uart_rx_cheak(uart2_msg);
		uart2_msg->rx_msg->Len = Size;
		HAL_UARTEx_ReceiveToIdle_IT(uart2_msg->rx_msg->huart, uart2_msg->rx_msg->pBuffer, uart2_msg->rx_msg->Len * 2);
		uart2_msg->count++;
		uart2_msg->time = 0;
		if (uart2_rx_hook != NULL)
		{
			uart2_rx_hook(uart5_msg->rx_msg->pBuffer, Size); // 添加钩子函数调用
		}
		break;
	case (unsigned long)USART3_BASE:
		uart_rx_cheak(uart3_msg);
		uart3_msg->rx_msg->Len = Size;
		HAL_UARTEx_ReceiveToIdle_IT(uart3_msg->rx_msg->huart, uart3_msg->rx_msg->pBuffer, uart3_msg->rx_msg->Len * 2);
		uart3_msg->count++;
		uart3_msg->time = 0;
		if (uart3_rx_hook != NULL)
		{
			uart3_rx_hook(uart5_msg->rx_msg->pBuffer, Size);
		}
		break;
	case (unsigned long)USART10_BASE:
		uart_rx_cheak(uart10_msg);
		uart10_msg->rx_msg->Len = Size;
		HAL_UARTEx_ReceiveToIdle_IT(uart10_msg->rx_msg->huart, uart10_msg->rx_msg->pBuffer, uart10_msg->rx_msg->Len * 2);
		uart10_msg->count++;
		uart10_msg->time = 0;
		if (uart10_rx_hook != NULL)
		{
			uart10_rx_hook(uart5_msg->rx_msg->pBuffer, Size);
		}
		break;
	default:
		break;
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{

	switch ((unsigned long)huart->Instance)
	{
	case (unsigned long)UART5_BASE:
		uart_rx_cheak(uart5_msg);
		HAL_UARTEx_ReceiveToIdle_IT(uart5_msg->rx_msg->huart, uart5_msg->rx_msg->pBuffer, uart5_msg->rx_msg->Len * 2);
		break;
	case (unsigned long)UART7_BASE:
		uart_rx_cheak(uart7_msg);
		HAL_UARTEx_ReceiveToIdle_IT(uart7_msg->rx_msg->huart, uart7_msg->rx_msg->pBuffer, uart7_msg->rx_msg->Len * 2);
		break;
	case (unsigned long)USART1_BASE:
		uart_rx_cheak(uart1_msg);
		HAL_UARTEx_ReceiveToIdle_IT(uart1_msg->rx_msg->huart, uart1_msg->rx_msg->pBuffer, uart1_msg->rx_msg->Len * 2);
		break;
	case (unsigned long)USART2_BASE:
		uart_rx_cheak(uart2_msg);
		HAL_UARTEx_ReceiveToIdle_IT(uart2_msg->rx_msg->huart, uart2_msg->rx_msg->pBuffer, uart2_msg->rx_msg->Len * 2);
		break;
	case (unsigned long)USART3_BASE:
		uart_rx_cheak(uart3_msg);
		HAL_UARTEx_ReceiveToIdle_IT(uart3_msg->rx_msg->huart, uart3_msg->rx_msg->pBuffer, uart3_msg->rx_msg->Len * 2);
		break;
	case (unsigned long)USART10_BASE:
		uart_rx_cheak(uart10_msg);
		HAL_UARTEx_ReceiveToIdle_IT(uart10_msg->rx_msg->huart, uart10_msg->rx_msg->pBuffer, uart10_msg->rx_msg->Len * 2);
		break;
	default:
		break;
	}
}
