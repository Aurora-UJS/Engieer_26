#include "radarTool.h"
#include "main.h"
#include "IMUtool.h"
#include "string.h"
#include "uart_api.h"
#include "crc_api.h"
#define Pi 3.14159265358979323846f
// Structure to hold the received data from the vision system
static uart_msg_t robot_state_msg = {0};
ReceivedPacketTwist_t radarTwist = {0}; // Structure to hold the data to be sent to the motor system
ReceivedPacketRobotControl_t robotControl = {0};

uint8_t rader_rx_buffer[RADAR_BUFFER_SIZE] = {0}; // Receive buffer for the radar data

void radar_msg_init(UART_HandleTypeDef *huart)
{
    robot_state_msg.huart = huart; // 绑定UART硬件接口
    robot_state_msg.Len = 0;       // 计算数据包长度
}

void receiveRadarData(uint8_t *rx_buffer)
{
    if (rx_buffer[0] == 0xA4)
    {
        memcpy(&radarTwist, rx_buffer, sizeof(ReceivedPacketTwist_t));
    }
    else if (rx_buffer[0] == 0xA3)
    {
        memcpy(&robotControl, rx_buffer, sizeof(ReceivedPacketRobotControl_t));
    }

    memset(rx_buffer, 0, RADAR_BUFFER_SIZE);
}

void sendAllRobotHPData(SendPacketAllRobotHP_t *sendAllRobotHP)
{
    if (sendAllRobotHP == NULL)
    {
        return;
    }

    sendAllRobotHP->header = 0x5B;

    Append_CRC16_Check_Sum((uint8_t *)sendAllRobotHP, sizeof(SendPacketAllRobotHP_t));
    HAL_UART_Transmit_IT(&robot_state_msg.huart, (uint8_t *)sendAllRobotHP, sizeof(SendPacketAllRobotHP_t));
}

void sendGameStatusData(SendPacketGameStatus_t *sendGameStatus)
{
    if (sendGameStatus == NULL)
    {
        return;
    }

    sendGameStatus->header = 0x5C;

    Append_CRC16_Check_Sum((uint8_t *)sendGameStatus, sizeof(SendPacketGameStatus_t));
    HAL_UART_Transmit_IT(&robot_state_msg.huart, (uint8_t *)sendGameStatus, sizeof(SendPacketGameStatus_t));
}

void sendRobotStatusData(SendPacketRobotStatus_t *sendRobotStatus)
{
    if (sendRobotStatus == NULL)
    {
        return;
    }

    sendRobotStatus->header = 0x5D;

    Append_CRC16_Check_Sum((uint8_t *)sendRobotStatus, sizeof(SendPacketRobotStatus_t));
    HAL_UART_Transmit_IT(&robot_state_msg.huart, (uint8_t *)sendRobotStatus, sizeof(SendPacketRobotStatus_t));
}