#include "radarTool.h"
#include "main.h"
#include "IMUtool.h"
#include "string.h"
#include "usart.h"
#include "crc_api.h"
#define Pi 3.14159265358979323846f
 // Structure to hold the received data from the vision system
SendPacketTwist radarTwist; // Structure to hold the data to be sent to the motor system
ReceivedPacketVision_t radarVisionData; // Structure to hold the data to be sent to the vision system
uint8_t rader_rx_buffer[RADAR_BUFFER_SIZE] = {0}; // Receive buffer for the radar data


void receiveRadarData(uint8_t *rx_buffer)
{
    if (rx_buffer[0] == 0xA4)
    {
        memcpy(&radarTwist, rx_buffer, sizeof(SendPacketTwist));
    }
    else if (rx_buffer[0] == 0xA5)
    {
        memcpy(&radarVisionData, rx_buffer, sizeof(ReceivedPacketVision_t));
    }
    
    memset(rx_buffer, 0, RADAR_BUFFER_SIZE);
}
