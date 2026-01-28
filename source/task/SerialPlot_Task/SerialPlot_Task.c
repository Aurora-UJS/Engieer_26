#include "cmsis_os2.h"
#include "usart.h"
#include <stdint.h>
#include <string.h>
#include "jointFollowAngle.h"
#include "uart_api.h"

uart_msg_t SerialPlot_tx_msg;
extern float joint_radian[6];
float CurrentRadian = 3.0f; // 测试代码记得删


void SerialPlot_tx_msg_init(void)
{
    static uint8_t temp_zero_buffer[9] = {0};
    SerialPlot_tx_msg.huart = &huart10;
    SerialPlot_tx_msg.pBuffer = temp_zero_buffer;
    SerialPlot_tx_msg.Len = strlen((char*) SerialPlot_tx_msg.pBuffer);
}


void SerialPlot(void *argument)
{
    UNUSED(argument);
    osDelay(10);
    SerialPlot_tx_msg_init();

    uint8_t frame[9];
    memset(frame, 0, sizeof(frame)); 
    frame[0] = 0xAB;
    float target_radian = 0.5f;
    while (1) {
        osDelay(500);
        memcpy(&frame[1], &CurrentRadian, sizeof(float));
        memcpy(&frame[5], &target_radian, sizeof(float));
        SerialPlot_tx_msg.pBuffer = frame;
        SerialPlot_tx_msg.Len = 9;
        uart_tx_send(&SerialPlot_tx_msg, HAL_MAX_DELAY);
    }
}
