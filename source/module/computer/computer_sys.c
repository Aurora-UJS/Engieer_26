#include "main.h"
#include "config.h"
#include "computer_sys.h"
#include "uart_api.h"
#include "vision_receive.h"
#include "send_robot_state.h"
#include "radarTool.h"

static uart_msg_t computer_msg;
static uart_rx_t computer_rx_msg;
static uint8_t computer_rx_buf[COMPUTER_BUFFER_SIZE]; // 接收缓冲区
void computer_uart_init(UART_HandleTypeDef *huart)
{
    computer_rx_msg.rx_msg = &computer_msg;
    computer_msg.huart = huart;
    computer_msg.pBuffer = computer_rx_buf;
    computer_msg.Len = COMPUTER_BUFFER_SIZE;

    uart_rx_init(&computer_rx_msg);
    robot_state_msg_init(huart);
    radar_msg_init(huart);

}

void computer_receive_solve()
{
    static uint32_t computer_msg_last_count = 0;
    if (computer_rx_msg.count == computer_msg_last_count)
    {
        return;
    }
    computer_msg_last_count = computer_rx_msg.count;

    vision_receive_solve(computer_rx_buf);

#if radar_MOD == 1
    receiveRadarData(computer_rx_buf); //雷达数据接收
#endif
}