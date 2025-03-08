#include "vision_receive.h"
#include "uart_api.h"
#include "crc_api.h"
#include "string.h"


ReceivedPacketVision_t ReceivedPacketVision = {0};
void vision_receive_solve(uint8_t *rx_buf)
{
    if (rx_buf[0] == 0xA5 && Verify_CRC16_Check_Sum(rx_buf, 48))
    {
        if (Verify_CRC16_Check_Sum(rx_buf, 48))
        {
            memcpy(&ReceivedPacketVision, rx_buf, sizeof(ReceivedPacketVision_t));
        }
    }
}