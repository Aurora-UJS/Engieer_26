#include "cmsis_os2.h"
#include "usart.h"
#include <stdint.h>
#include <string.h>
#include "uart_api.h"


uart_msg_t SerialPlot_tx_msg;
extern float joint_radian[6];
extern float Current_Radian[6];
#define SerialPlotLen 25
void SerialPlot_tx_msg_init(void)
{
    static uint8_t temp_zero_buffer[SerialPlotLen] = {0};
    SerialPlot_tx_msg.huart = &huart10;
    SerialPlot_tx_msg.pBuffer = temp_zero_buffer;
    SerialPlot_tx_msg.Len = strlen((char*) SerialPlot_tx_msg.pBuffer);
}


void SerialPlot(void *argument)
{
    UNUSED(argument);
    osDelay(10);
    SerialPlot_tx_msg_init();

    uint8_t frame[SerialPlotLen];
    memset(frame, 0, sizeof(frame)); 
    frame[0] = 0xAB;
    while (1) {
        osDelay(30);
        memcpy(&frame[1], &Current_Radian[0], sizeof(float));
        memcpy(&frame[5], &Current_Radian[1], sizeof(float));
        memcpy(&frame[9], &Current_Radian[2], sizeof(float));
        memcpy(&frame[13], &Current_Radian[3], sizeof(float));
        memcpy(&frame[17], &Current_Radian[4], sizeof(float));
        memcpy(&frame[21], &Current_Radian[5], sizeof(float));
        SerialPlot_tx_msg.pBuffer = frame;
        SerialPlot_tx_msg.Len = SerialPlotLen;
        uart_tx_send(&SerialPlot_tx_msg, HAL_MAX_DELAY);
    }
}
