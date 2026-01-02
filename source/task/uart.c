#include "motor_DM.h"
#include "cmsis_os2.h"
#include "uart_api.h"
#include "usart.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <float.h>
#include <string.h>

uart_rx_t  testUart_msg;
uart_msg_t testUart_rx_msg;
// uart_msg_t testUart_tx_msg;
uart_msg_t feedbackData_msg;
uint8_t    testUartBuffer[18];
uint32_t   lastCount;

float dm_angle = 0;
float dm_tor = 0;

extern void (*uart7_rx_hook)(uint8_t *pData, uint32_t size);
static float dm_pos = 0;
void uart7_test_hook(uint8_t *buf, uint32_t len)
{
    dm_pos =  atoi(buf);
    HAL_UART_Transmit(&huart7, buf, len, 100);
}

void testUart_rx_init(void){
    testUart_msg.rx_msg           =  &testUart_rx_msg;
    testUart_msg.rx_msg-> pBuffer = testUartBuffer;
    testUart_msg.rx_msg-> huart   = &huart7; 
    testUart_msg.rx_msg-> Len     = 18;
    uart7_rx_hook                 = uart7_test_hook;
    uart_rx_init(&testUart_msg);
}
// void testUart_tx_init(void){
//     testUart_tx_msg.huart= &huart7;
//     testUart_tx_msg.pBuffer = (uint8_t *)"hello\r\n";
//     testUart_tx_msg.Len = strlen((char *) testUart_tx_msg.pBuffer);
// }


void uart_Refresh(void)
{
    if (testUart_msg.count != lastCount)
    {

        lastCount = testUart_msg.count;
    }
}
uint8_t buf[16];
float feedbackAngle = 0;
void feedbackData_init(void)
{
    feedbackAngle = dm_angle * 10000;
    sprintf((char *)buf, "%.2d\r\n",(int)feedbackAngle);
    feedbackData_msg.pBuffer = (uint8_t *)buf;
    feedbackData_msg.huart= &huart7;
    feedbackData_msg.Len = 10;
}
char feedback_buf[64] = {0};
void send_feedbackData(void)
{
    feedbackAngle = dm_angle * 10000;
    int len = sprintf((char *)buf, "%.2d\r\n",(int)feedbackAngle);

    feedbackData_msg.pBuffer = (uint8_t *)buf;
    feedbackData_msg.Len     = len;
}
void uart_test(void *arguments)
{
    UNUSED(arguments);
    osDelay(10);
    testUart_rx_init();
    // testUart_tx_init();
    feedbackData_init();
    osDelay(10);
    uart_tx_send_IT(&feedbackData_msg);
        // send_feedbackData();

    // uart_tx_send_IT(&feedbackData_msg);
    while (1) {
        // uart_tx_send_IT(&testUart_tx_msg);
        send_feedbackData();
        osDelay(500);
        uart_tx_send_IT(&feedbackData_msg);
    }
}

void motor_test(void *argument){
     UNUSED(argument);
    // 定义一个 DM_motor_t 类型的 motor 结构体
    DM_motor_t *DM_motor = pvPortMalloc(sizeof(DM_motor_t));

    // 设置 CAN 配置的 ID
    DM_motor->can_cfg.id = 0x001;

    // 设置电机消息的 CAN ID
    DM_motor->motor_msg.can_msg.id = 0x11;

    // 设置 CAN 端口
    DM_motor->can_cfg.port = CAN1_PORT;

    // 设置电机参数
    DM_motor->tmp.PMAX = 12.5;
    DM_motor->tmp.VMAX = 3;
    DM_motor->tmp.TMAX = 1;


    // 初始化电机
    Motor_DM_Init(DM_motor);

    // 启用电机
    Motor_DM_Enable(DM_motor);
    
    // 无限循环
    while (true)
    {
        // 刷新电机状态
        Motor_DM_Refresh(DM_motor);

        // 获取电机角度
        dm_angle = DM_motor->motor_msg.motor_angle;
        dm_tor   = DM_motor->motor_msg.torque_current;
        // 控制电机的位置和速度
        MIT_CtrlMotorDM(DM_motor, dm_pos, 0, 1.0f, 0.0f, 0.0f);

        osDelay(1);
    }   
}