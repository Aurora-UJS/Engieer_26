#include "dsp/fast_math_functions.h"
#include "main.h"
#include "usart.h"
#include "uart_api.h"
#include "motor_DM.h"
#include "cmsis_os2.h"
#include <stdio.h>
#include "arm_math.h"
#define JOINT_NUM 6

DM_motor_t *joint_motor[JOINT_NUM];
uart_rx_t Angle_msg;
uart_msg_t Angle_rx_msg;
uart_msg_t Angle_tx_msg;
uint8_t Angle_rx_msg_Buffer[256];
float joint_radian[6] = {0};

void Angle_Receive_Callback(uint8_t *buf, uint32_t len)
{
    // HAL_UART_Transmit(&huart7, buf, len, 100);  // 回传显示

    for (int i = 0; i < 6; i++) {
        int tmp = 0;
        // 每个弧度占 4 个字符
        sscanf((const char*)&buf[i * 4], "%04d", &tmp);
        joint_radian[i] = tmp / 100.0f;
        joint_radian[i] = joint_radian[i] - PI;
    }
}
void angle_msg_rx_init(void)
{
    Angle_msg.rx_msg = &Angle_rx_msg;
    Angle_msg.rx_msg -> pBuffer = Angle_rx_msg_Buffer;
    Angle_msg.rx_msg -> huart = &huart7;
    Angle_msg.rx_msg -> Len = 256;
    uart7_rx_hook = Angle_Receive_Callback;
    uart_rx_init(&Angle_msg);
}
uart_msg_t testUart_tx_msg;
void testUart_tx_init(void){
    testUart_tx_msg.huart= &huart7;
    testUart_tx_msg.pBuffer = (uint8_t *)"hello\r\n";
    testUart_tx_msg.Len = strlen((char *) testUart_tx_msg.pBuffer);
}

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
float dm_angle_test = 0;
void jointFollowAngle(void *argument)
{
    UNUSED(argument);
    joint_motor_init();
    Motor_DM_Enable(joint_motor[0]);
    while (1) {
        dm_angle_test = joint_motor[0]->motor_msg.motor_angle;
        Motor_DM_Refresh(joint_motor[0]);
        // Motor_DM_Refresh(joint_motor[1]);
        // Motor_DM_Refresh(joint_motor[2]);
        // Motor_DM_Refresh(joint_motor[3]);
        // Motor_DM_Refresh(joint_motor[4]);
        // Motor_DM_Refresh(joint_motor[5]);

        MIT_CtrlMotorDM(joint_motor[0],joint_radian[4], 0, 1, 0, 0);
        // MIT_CtrlMotorDM(joint_motor[1],joint_radian[1], 0, 1, 0, 0);
        // MIT_CtrlMotorDM(joint_motor[2],joint_radian[2], 0, 1, 0, 0);
        // MIT_CtrlMotorDM(joint_motor[3],joint_radian[3], 0, 1, 0, 0);
        // MIT_CtrlMotorDM(joint_motor[4],joint_radian[4], 0, 1, 0, 0);
        // MIT_CtrlMotorDM(joint_motor[5],joint_radian[5], 0, 1, 0, 0);
        osDelay(2);
    }
}
