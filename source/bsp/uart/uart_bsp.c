#include "uart_bsp.h"
#include "uart_api.h"
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "usart.h"
#include <string.h>

/* UART7 当前用于图传/控制器链路，数据会在回调里立刻进入上层解析。
 * 若先重开 DMA 再把原 DMA 缓冲区指针交给上层，下一包可能覆盖当前包内容，
 * 导致出现“偶尔解析成功、绝大多数失败”的抖动现象。
 * 这里先复制一份到 shadow buffer，再重开接收，最后把稳定副本交给钩子。
 */
static uint8_t uart7_rx_shadow[512];
static uint8_t uart1_rx_shadow[512];

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
	{
		uint16_t copy_size = Size;

		uart_rx_cheak(uart7_msg);
		if (copy_size > (uint16_t)sizeof(uart7_rx_shadow)) {
			copy_size = (uint16_t)sizeof(uart7_rx_shadow);
		}
		if ((uart7_msg != NULL) && (uart7_msg->rx_msg != NULL) && (uart7_msg->rx_msg->pBuffer != NULL) && (copy_size > 0U)) {
			memcpy(uart7_rx_shadow, uart7_msg->rx_msg->pBuffer, copy_size);
		}
		// 不修改 Len，保持缓冲区容量不变；用 Size 表示本次实际收到的长度
		HAL_UARTEx_ReceiveToIdle_IT(uart7_msg->rx_msg->huart, uart7_msg->rx_msg->pBuffer, uart7_msg->rx_msg->Len);
		uart7_msg->count++;
		uart7_msg->time = 0;
		if (uart7_rx_hook != NULL)
		{
			uart7_rx_hook(uart7_rx_shadow, copy_size);
		}
		break;
	}
			case (unsigned long)USART1_BASE:
			{
				uint16_t copy_size = Size;

				uart_rx_cheak(uart1_msg);
				if (copy_size > (uint16_t)sizeof(uart1_rx_shadow)) {
					copy_size = (uint16_t)sizeof(uart1_rx_shadow);
				}
				if ((uart1_msg != NULL) && (uart1_msg->rx_msg != NULL) && (uart1_msg->rx_msg->pBuffer != NULL) && (copy_size > 0U)) {
					memcpy(uart1_rx_shadow, uart1_msg->rx_msg->pBuffer, copy_size);
				}
				HAL_UARTEx_ReceiveToIdle_IT(uart1_msg->rx_msg->huart, uart1_msg->rx_msg->pBuffer, uart1_msg->rx_msg->Len);
				uart1_msg->count++;
				uart1_msg->time = 0;
				if (uart1_rx_hook != NULL)
				{
					uart1_rx_hook(uart1_rx_shadow, copy_size);
				}
				break;
			}
		case (unsigned long)USART2_BASE:
			uart_rx_cheak(uart2_msg);
			uart2_msg->rx_msg->Len = Size;
			HAL_UARTEx_ReceiveToIdle_IT(uart2_msg->rx_msg->huart, uart2_msg->rx_msg->pBuffer, uart2_msg->rx_msg->Len * 2);
			uart2_msg->count++;
			uart2_msg->time = 0;
			if (uart2_rx_hook != NULL)
			{
				uart2_rx_hook(uart2_msg->rx_msg->pBuffer, Size); // 将本串口实际接收到的数据交给对应钩子
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
				uart3_rx_hook(uart3_msg->rx_msg->pBuffer, Size);
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
			uart10_rx_hook(uart10_msg->rx_msg->pBuffer, Size);
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
		HAL_UARTEx_ReceiveToIdle_IT(uart7_msg->rx_msg->huart, uart7_msg->rx_msg->pBuffer, uart7_msg->rx_msg->Len);
		break;
		case (unsigned long)USART1_BASE:
			uart_rx_cheak(uart1_msg);
			HAL_UARTEx_ReceiveToIdle_IT(uart1_msg->rx_msg->huart, uart1_msg->rx_msg->pBuffer, uart1_msg->rx_msg->Len);
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
