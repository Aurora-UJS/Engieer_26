#include "uart_bsp.h"
#include "uart_api.h"
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "usart.h"


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{

	if (huart->Instance == UART5)
	{
		HAL_UARTEx_ReceiveToIdle_IT(uart5_msg->rx_msg->huart, uart5_msg->rx_msg->pBuffer, uart5_msg->rx_msg->Len * 2);
		uart5_msg->count++;
		uart5_msg->time = 0;
	}
	else if (huart->Instance == UART7)
	{
		HAL_UARTEx_ReceiveToIdle_IT(uart5_msg->rx_msg->huart, uart5_msg->rx_msg->pBuffer, uart5_msg->rx_msg->Len * 2);
		uart5_msg->count++;
		uart5_msg->time = 0;
	}
	else if (huart->Instance == USART1)
	{
		HAL_UARTEx_ReceiveToIdle_IT(uart1_msg->rx_msg->huart, uart1_msg->rx_msg->pBuffer, uart1_msg->rx_msg->Len * 2);
		uart1_msg->count++;
		uart1_msg->time = 0;
	}
	else if (huart->Instance == USART2)
	{
		HAL_UARTEx_ReceiveToIdle_IT(uart2_msg->rx_msg->huart, uart2_msg->rx_msg->pBuffer, uart2_msg->rx_msg->Len * 2);
		uart2_msg->count++;
		uart2_msg->time = 0;
	}
	else if (huart->Instance == USART3)
	{
		HAL_UARTEx_ReceiveToIdle_IT(uart3_msg->rx_msg->huart, uart3_msg->rx_msg->pBuffer, uart3_msg->rx_msg->Len * 2);
		uart3_msg->count++;
		uart3_msg->time = 0;
	}

}



void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == UART5)
	{
		HAL_UARTEx_ReceiveToIdle_IT(uart5_msg->rx_msg->huart, uart5_msg->rx_msg->pBuffer, uart5_msg->rx_msg->Len * 2);
	}
	else if (huart->Instance == UART7)
	{
		HAL_UARTEx_ReceiveToIdle_IT(uart5_msg->rx_msg->huart, uart5_msg->rx_msg->pBuffer, uart5_msg->rx_msg->Len * 2);
	}
	else if (huart->Instance == USART1)
	{
		HAL_UARTEx_ReceiveToIdle_IT(uart1_msg->rx_msg->huart, uart1_msg->rx_msg->pBuffer, uart1_msg->rx_msg->Len * 2);
	}
	else if (huart->Instance == USART2)
	{
		HAL_UARTEx_ReceiveToIdle_IT(uart2_msg->rx_msg->huart, uart2_msg->rx_msg->pBuffer, uart2_msg->rx_msg->Len * 2);
	}
	else if (huart->Instance == USART3)
	{
		HAL_UARTEx_ReceiveToIdle_IT(uart3_msg->rx_msg->huart, uart3_msg->rx_msg->pBuffer, uart3_msg->rx_msg->Len * 2);
	}
}
