#include "cmsis_os2.h"
#include "stm32h7xx_hal.h"
#include "usart.h"

void uart_test(void *argument)
{
    UNUSED(argument);

    const uint8_t msg[] = "Hello UART7\r\n";

    /* 等待系统初始化完成 */
    osDelay(100);

    for (;;)
    {
        HAL_UART_Transmit(&huart7,
                          (uint8_t *)msg,
                          sizeof(msg) - 1,
                          HAL_MAX_DELAY);

        osDelay(1000);   // 1 秒发一次
    }
}