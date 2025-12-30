#include "cmsis_os2.h"
#include "stm32h7xx_hal.h"
#include "usart.h"


uint8_t uart7_rx_byte;
void uart_test(void *argument)
{
    UNUSED(argument);

    const uint8_t msg[] = "Hello UART7\r\n";

    osDelay(100);

    HAL_UART_Receive_IT(&huart7, &uart7_rx_byte, 1);

    for (;;)
    {
        HAL_UART_Transmit(&huart7,
                          (uint8_t *)msg,
                          sizeof(msg) - 1,
                          100);

        osDelay(5000);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart7)
    {
        const uint8_t rx_msg[] = "receive\r\n";

        HAL_UART_Transmit_IT(&huart7,
                             (uint8_t *)rx_msg,
                             sizeof(rx_msg) - 1);

        HAL_UART_Receive_IT(&huart7, &uart7_rx_byte, 1);
    }
}