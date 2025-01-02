#include "radarTool.h"
#include "main.h"
#include "IMUtool.h"
#include "string.h"
#include "usart.h"
#include "crc.h"
#define Pi 3.14159265358979323846f
ReceivePacketVision radarData; // Structure to hold the received data from the vision system
SendPacketTwist radarTwist; // Structure to hold the data to be sent to the motor system
SendPacketVision radarVisionData; // Structure to hold the data to be sent to the vision system
uint8_t rader_rx_buffer[RADAR_BUFFER_SIZE] = {0}; // Receive buffer for the radar data


void sendRadarData(void)
{
    radarData.header = 0x5A;
    radarData.detect_color = 0;
    radarData.reset_tracker = 0;
    radarData.reserved = 0;
    radarData.roll = 0;
    radarData.yaw = yaw_angle_now;
    // radarData.pitch = (6150 - moto_data[5].rotor_angle) * 2.0f * Pi / 8191.0f;
    
    Append_CRC16_Check_Sum((uint8_t*)&radarData, sizeof(radarData));
    HAL_UART_Transmit_IT(&huart7, (uint8_t*)&radarData, sizeof(radarData));
}
void receiveRadarData(uint8_t *rx_buffer)
{
    if (rx_buffer[0] == 0xA4)
    {
        memcpy(&radarTwist, rx_buffer, sizeof(SendPacketTwist));
    }
    else if (rx_buffer[0] == 0xA5)
    {
        memcpy(&radarVisionData, rx_buffer, sizeof(SendPacketVision));
    }
    
    memset(rx_buffer, 0, sizeof(rader_rx_buffer));
}
