#include "can_struct.h"
#include "main.h"
#include "stm32_hal_legacy.h"
#include "stm32h7xx_hal_def.h"
#include "usart.h"
#include "uart_api.h"
#include <stdlib.h>
#include "motor_DM.h"
#include "cmsis_os2.h"
#include <stdint.h>
#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include "arm_math.h"
#include "tool.h"
#include "DbusSys.h"
#define JOINT_NUM 7

#define ANGLE_DATA_LENGTH 24  // 实际角度数据长度（6个角度 × 4字符）

// uart_msg_t Angle_tx_msg; 测试
// uint8_t testBuf[50] = {0};
uint8_t Angle_rx_msg_Buffer[256];
DM_motor_t *joint_motor[JOINT_NUM];
uart_rx_t Angle_msg; // 
uart_msg_t Angle_rx_msg;
float joint_radian[6] = {0};
uint8_t firstEnableFlag = 0;
uint8_t angle_data[ANGLE_DATA_LENGTH] = {0};
extern rc_info_t remoter;

static uint8_t angle_digits[ANGLE_DATA_LENGTH];
static uint8_t angle_digits_len = 0;

// ch1 右摇杆 左右 左-右+
// ch2 右摇杆 前后 前+后-
// ch3 左摇杆 左右 左-右+
// ch4 左摇杆 前后 前+后-
// sw1 左拨码开关 前1 中3 后2
// sw2 右拨码开关 前1 中3 后2

/**
 * @brief UART7 接收回调：从串口字节流中提取并解析 24 位角度数据
 *
 * @param buf UART驱动回调提供的接收数据缓冲区指针
 * @param len 本次回调收到的数据长度（字节数）
 *
 * @details
 * - 本函数不依赖帧头/帧尾/CRC，仅扫描 buf 中的 ASCII 数字字符 '0'~'9'
 * - 连续收集到 24 个数字字符后，按 6 组 × 4 字符转换为 6 个关节角度
 * - 解析完成后更新 joint_radian[0..5]
 *
 * @warning
 * 若串口字节流中混入其它数字字符（例如打印日志），可能导致误拼接与误解析。
 */

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

/**
 * @brief 电机初始化
 * 
 */
void joint_motor_init(void)
{
    for (int joint_index = 0; joint_index < JOINT_NUM; joint_index++) {
        joint_motor[joint_index] = pvPortMalloc(sizeof(DM_motor_t));
        //配置 can2
        joint_motor[joint_index]->can_cfg.port = CAN2_PORT;
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
    // angle_msg_rx_init();  // 注释掉：UART7 已被 Referee_Task 使用
    // testUart_tx_init();
    while (1) {
        // uart_tx_send_IT(&testUart_tx_msg);
        osDelay(10);
    }
}
float dm_angle_test[6] = {0}; 
extern uint8_t CtrllerData[24];
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
        for (int i = 0; i < 6; i++) {
            int tmp = 0;
            // 每个弧度占 4 个字符
            sscanf((const char*)&CtrllerData[i * 4], "%04d", &tmp);
            joint_radian[i] = tmp / 1000.0f;
            joint_radian[i] = joint_radian[i] - PI;
        }

        // PosSpeed_CtrlMotorDM(joint_motor[0],joint_radian[0], 1);
        PosSpeed_CtrlMotorDM(joint_motor[1],limit(joint_radian[1], 0, 1.5), 0.5); // pitch轴控制
        osDelay(1);
        PosSpeed_CtrlMotorDM(joint_motor[2],limit(-joint_radian[2], 0, 3), 0.5); // pitch轴控制
        osDelay(1);
        PosSpeed_CtrlMotorDM(joint_motor[3],-joint_radian[3],  0.5); // roll轴控制
        osDelay(1);
        PosSpeed_CtrlMotorDM(joint_motor[4],limit(-joint_radian[4],-1.5, 1.5), 0.5); // pitch轴控制
        osDelay(1);
        PosSpeed_CtrlMotorDM(joint_motor[5],-joint_radian[5], 0.5); // roll轴控制
        osDelay(1);

        switch(remoter.sw1)
        {
            case 3:
                PosSpeed_CtrlMotorDM(joint_motor[6],0, 0.5); // 夹爪控制
                osDelay(1);
            break;
            //1是前，夹爪张开
            case 2:
                PosSpeed_CtrlMotorDM(joint_motor[6],1.2, 0.5); // 夹爪控制
                osDelay(1);
            break;
            //2是后，夹爪合拢

        }
    }
}
