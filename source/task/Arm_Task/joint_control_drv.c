// 关节控制驱动

#include "joint_control_drv.h"


float joint_radian[6] = {0}; 
DM_motor_t *Joint_Motor[JOINT_NUM];

/**
 * @brief 电机初始化
 * 
 */
void joint_motor_init(void)
{
    for (int joint_index = 0; joint_index < JOINT_NUM; joint_index++) {
        Joint_Motor[joint_index] = pvPortMalloc(sizeof(DM_motor_t));
        //配置 can2
        Joint_Motor[joint_index]->can_cfg.port = CAN2_PORT;
        Joint_Motor[joint_index]->tmp.PMAX = 12.5f;
        Joint_Motor[joint_index]->tmp.VMAX = 3.0f;
        Joint_Motor[joint_index]->tmp.TMAX = 1.0f;
        // 配置can id
        Joint_Motor[joint_index]->can_cfg.id = 0x01 + joint_index;
        Joint_Motor[joint_index]->motor_msg.can_msg.id = 0x11+ joint_index;
        Motor_DM_Init(Joint_Motor[joint_index]);
    }
}
/**
 * @brief 电机数据更新
 * 
 */
void Joint_Motor_Refresh(void)
{
    for (int joint_index = 0; joint_index < JOINT_NUM; joint_index++) {
        Motor_DM_Refresh(Joint_Motor[joint_index]);
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
        Motor_DM_Enable(Joint_Motor[joint_index]);
    }
}
