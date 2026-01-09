#include "main.h"
#include "usart.h"
#include "uart_api.h"
#include "motor_DM.h"
#include "cmsis_os2.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "arm_math.h"
#include "tool.h"
#define JOINT_NUM 6

// uart_msg_t Angle_tx_msg; 测试
// uint8_t testBuf[50] = {0};
uint8_t Angle_rx_msg_Buffer[256];
DM_motor_t *joint_motor[JOINT_NUM];
uart_rx_t Angle_msg; // 
uart_msg_t Angle_rx_msg;
float joint_radian[6] = {0};
uint8_t firstEnableFlag = 0;


/**
 * @brief 角度接收回调
 * 
 * @param buf 接收缓冲数组
 * @param len 数组长度
 */
void Angle_Receive_Callback(uint8_t *buf, uint32_t len)
{
    // HAL_UART_Transmit(&huart7, buf, len, 100);  // 回传显示 测试
    // memcpy(testBuf, buf, sizeof(buf)); 测试
    if (firstEnableFlag == 0) {
        firstEnableFlag = 1;
    }
    else {
        for (int i = 0; i < 6; i++) {
            int tmp = 0;
            // 每个弧度占 4 个字符
            sscanf((const char*)&buf[i * 4], "%04d", &tmp);
            joint_radian[i] = tmp / 1000.0f;
            joint_radian[i] = joint_radian[i] - PI;
        }
    }
}

/**
 * @brief 串口角度接收初始化
 * 
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
// uart_msg_t testUart_tx_msg;
// void testUart_tx_init(void){
//     testUart_tx_msg.huart= &huart7;
//     testUart_tx_msg.pBuffer = (uint8_t *)"hello\r\n";
//     testUart_tx_msg.Len = strlen((char *) testUart_tx_msg.pBuffer);
// }

/**
 * @brief 电机初始化
 * 
 */
void joint_motor_init(void)
{
    for (int joint_index = 0; joint_index < JOINT_NUM; joint_index++) {
        joint_motor[joint_index] = pvPortMalloc(sizeof(DM_motor_t));
        //配置 can1
        joint_motor[joint_index]->can_cfg.port = CAN1_PORT;
        joint_motor[joint_index]->tmp.PMAX = 12.5f;
        joint_motor[joint_index]->tmp.VMAX = 3.0f;
        joint_motor[joint_index]->tmp.TMAX = 1.0f;
        // 配置can id
        joint_motor[joint_index]->can_cfg.id = 0x01 + joint_index;
        joint_motor[joint_index]->motor_msg.can_msg.id = 0x11+ joint_index;
        Motor_DM_Init(joint_motor[joint_index]);
    }
}

/**
 * @brief 电机数据更新
 * 
 */
void Joint_Motor_Refresh(void)
{
    for (int joint_index = 0; joint_index < JOINT_NUM; joint_index++) {
        Motor_DM_Refresh(joint_motor[joint_index]);
    }
}

/**
 * @brief 电机使能
 * 
 */
void Joint_Motor_Enable(void)
{
    for (int joint_index = 0; joint_index < JOINT_NUM; joint_index++) {
        // osDelay(100);
        Motor_DM_Enable(joint_motor[joint_index]);
    }
}

// 任务函数
void uart_Transmit_Angle(void *argment)
{
    UNUSED(argment);
    osDelay(10);
    angle_msg_rx_init();
    // testUart_tx_init();
    while (1) {
        // uart_tx_send_IT(&testUart_tx_msg);
        osDelay(10);
    }
}
float dm_angle_test[6] = {0}; 

void jointFollowAngle(void *argument)
{
    UNUSED(argument);
    joint_motor_init();
    // Motor_DM_Save_Zero(joint_motor[3]);
    // Motor_DM_Save_Zero(joint_motor[5]);
    osDelay(100);
    Joint_Motor_Enable(); // 使能所有电机
    while (1) {
        Joint_Motor_Refresh();

        // PosSpeed_CtrlMotorDM(joint_motor[0],joint_radian[0], 1);
        PosSpeed_CtrlMotorDM(joint_motor[1],joint_radian[1], 0.5);
        PosSpeed_CtrlMotorDM(joint_motor[2],-joint_radian[2], 0.5);
        PosSpeed_CtrlMotorDM(joint_motor[3],-joint_radian[3], 0.5);
        PosSpeed_CtrlMotorDM(joint_motor[4],-joint_radian[4], 0.5);
        PosSpeed_CtrlMotorDM(joint_motor[5],joint_radian[5], 0.5);
        osDelay(2);
    }
}
