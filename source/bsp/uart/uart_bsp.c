#include "uart_bsp.h"
#include "uart_api.h"
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "usart.h"

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{

	switch ((int)huart->Instance)
	{
	case (int)UART5:
		uart_reg_cheak(uart5_msg);
		uart5_msg->rx_msg->Len = Size;
		HAL_UARTEx_ReceiveToIdle_IT(uart5_msg->rx_msg->huart, uart5_msg->rx_msg->pBuffer, uart5_msg->rx_msg->Len * 2);
		uart5_msg->count++;
		uart5_msg->time = 0;
		break;
	case (int)UART7:
		uart_reg_cheak(uart7_msg);
		uart7_msg->rx_msg->Len = Size;
		HAL_UARTEx_ReceiveToIdle_IT(uart7_msg->rx_msg->huart, uart7_msg->rx_msg->pBuffer, uart7_msg->rx_msg->Len * 2);
		uart7_msg->count++;
		uart7_msg->time = 0;
		break;
	case (int)USART1:
		uart_reg_cheak(uart1_msg);
		uart1_msg->rx_msg->Len = Size;
		HAL_UARTEx_ReceiveToIdle_IT(uart1_msg->rx_msg->huart, uart1_msg->rx_msg->pBuffer, uart1_msg->rx_msg->Len * 2);
		uart1_msg->count++;
		uart1_msg->time = 0;
		break;
	case (int)USART2:
		uart_reg_cheak(uart2_msg);
		uart2_msg->rx_msg->Len = Size;
		HAL_UARTEx_ReceiveToIdle_IT(uart2_msg->rx_msg->huart, uart2_msg->rx_msg->pBuffer, uart2_msg->rx_msg->Len * 2);
		uart2_msg->count++;
		uart2_msg->time = 0;
		break;
	case (int)USART3:
		uart_reg_cheak(uart3_msg);
		uart3_msg->rx_msg->Len = Size;
		HAL_UARTEx_ReceiveToIdle_IT(uart3_msg->rx_msg->huart, uart3_msg->rx_msg->pBuffer, uart3_msg->rx_msg->Len * 2);
		uart3_msg->count++;
		uart3_msg->time = 0;
		break;
	default:
		break;
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{

	switch ((int)huart->Instance)
	{
	case (int)UART5:
		uart_reg_cheak(uart5_msg);
		HAL_UARTEx_ReceiveToIdle_IT(uart5_msg->rx_msg->huart, uart5_msg->rx_msg->pBuffer, uart5_msg->rx_msg->Len * 2);
		break;
	case (int)UART7:
		uart_reg_cheak(uart7_msg);
		HAL_UARTEx_ReceiveToIdle_IT(uart7_msg->rx_msg->huart, uart7_msg->rx_msg->pBuffer, uart7_msg->rx_msg->Len * 2);
		break;
	case (int)USART1:
		uart_reg_cheak(uart1_msg);
		HAL_UARTEx_ReceiveToIdle_IT(uart1_msg->rx_msg->huart, uart1_msg->rx_msg->pBuffer, uart1_msg->rx_msg->Len * 2);
		break;
	case (int)USART2:
		uart_reg_cheak(uart2_msg);
		HAL_UARTEx_ReceiveToIdle_IT(uart2_msg->rx_msg->huart, uart2_msg->rx_msg->pBuffer, uart2_msg->rx_msg->Len * 2);
		break;
	case (int)USART3:
		uart_reg_cheak(uart3_msg);
		HAL_UARTEx_ReceiveToIdle_IT(uart3_msg->rx_msg->huart, uart3_msg->rx_msg->pBuffer, uart3_msg->rx_msg->Len * 2);
		break;
	default:
		break;
	}
}
