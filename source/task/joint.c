#include "main.h"
#include "usart.h"
#include "uart_api.h"
#include "cmsis_os2.h"

#define ANGLE_DATA_LENGTH 24  // 实际角度数据长度（6个角度 × 4字符）

// uart_msg_t Angle_tx_msg; 测试
// uint8_t testBuf[50] = {0};
uint8_t Angle_rx_msg_Buffer[256];
uart_rx_t Angle_msg; // 
uart_msg_t Angle_rx_msg;
uint8_t firstEnableFlag = 0;
uint8_t angle_data[ANGLE_DATA_LENGTH] = {0};

// ch1 右摇杆 左右 左-右+
// ch2 右摇杆 前后 前+后-
// ch3 左摇杆 左右 左-右+
// ch4 左摇杆 前后 前+后-
// sw1 左拨码开关 前1 中3 后2
// sw2 右拨码开关 前1 中3 后2
void Angle_Receive_Callback(uint8_t *buf, uint32_t len)
{
    UNUSED(buf);
    UNUSED(len);
    return;
}

/**
 * @brief 串口角度接收初始化
 *
 * @details
 * 1) 配置 UART7 的接收消息结构体（句柄/缓冲区/长度）
 * 2) 注册 UART7 接收回调钩子函数 uart7_rx_hook
 * 3) 启动 UART7 的空闲中断接收（ReceiveToIdle）
 *
 * @note
 * - 接收缓冲区为 Angle_rx_msg_Buffer
 * - 实际解析逻辑在 Angle_Receive_Callback() 中完成
 */
void angle_msg_rx_init(void)
{
    Angle_msg.rx_msg = &Angle_rx_msg;
    Angle_msg.rx_msg -> pBuffer = Angle_rx_msg_Buffer;
    Angle_msg.rx_msg -> huart = &huart7;
    Angle_msg.rx_msg -> Len = 256;
    uart7_rx_hook = Angle_Receive_Callback;
    uart_rx_init(&Angle_msg);
}



// 任务函数
void uart_Transmit_Angle(void *argment)
{
    UNUSED(argment);
    osDelay(10);
    while (1) {
        osDelay(10);
    }
}
float dm_angle_test[6] = {0}; 
