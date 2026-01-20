// joint_control_drv.c: 关节控制驱动

#include "joint_control_drv.h"
#include "can_struct.h"
#include "motor_DM.h"
#include "portable.h"


float joint_radian[6] = {0}; 
DM_motor_t *Joint_Motor[JOINT_NUM];

/**
 * @brief 电机数据更新
 * 
 */
void Joint_Motor_Refresh(Joint_t *Joint)
{
    for (int joint_index = 0; joint_index < JOINT_NUM; joint_index++) {
        Motor_DM_Refresh(Joint[joint_index].joint_motor);
    }
}

/**
 * @brief 末端执行器信息更新
 * 
 */
void EndEffector_Motor_Refresh(Joint_t *endeffector){
    Motor_DM_Refresh(endeffector->joint_motor);
}

/**
 * @brief 电机使能
 * 
 */
void Joint_Motor_Enable(Joint_t *Joint)
{
    for (int joint_index = 0; joint_index < JOINT_NUM; joint_index++) {
        Motor_DM_Enable(Joint[joint_index].joint_motor);
    }
}
/**
 * @brief 末端执行使能
 * 
 */

void EndEffector_Motor_Enable(Joint_t *endeffector){
    Motor_DM_Enable(endeffector->joint_motor);
}

void endeffector_init(Joint_t *endeffector)
{
    endeffector_motor_init(endeffector);
}
void endeffector_motor_init(Joint_t *endeffector){
    endeffector->joint_motor = pvPortMalloc(sizeof(DM_motor_t)) ;
    endeffector->joint_motor->can_cfg.id = 0x07;
    endeffector->joint_motor->motor_msg.can_msg.id= 0x07;
    endeffector->joint_motor->can_cfg.port = CAN2_PORT;

    Motor_DM_Init(endeffector->joint_motor);
}
void joint_motor_init(Joint_t *Joint){
    for (int joint_index = 0; joint_index < JOINT_NUM; joint_index++) {

        Joint[joint_index].joint_motor = pvPortMalloc(sizeof(DM_motor_t));
        // Slave id 初始化 
        Joint[joint_index].joint_motor->can_cfg.id = 0x01 + joint_index;
        // Master id 初始化
        Joint[joint_index].joint_motor->motor_msg.can_msg.id = 0x11+ joint_index;
        
        // PMAX,VMAX,TMAX 初始化
        Joint[joint_index].joint_motor->tmp.PMAX = 12.5f;
        Joint[joint_index].joint_motor->tmp.VMAX = 3.0f;
        Joint[joint_index].joint_motor->tmp.TMAX = 1.0f;

        // CAN Port初始化
        Joint[joint_index].joint_motor->can_cfg.port = can_port_map[joint_index];
    }


    // 初始化电机
    for (int joint_index = 0; joint_index < JOINT_NUM; joint_index++) {
        Motor_DM_Init(Joint[joint_index].joint_motor);
    }    

}
void joint_dof_init(Joint_t *Joint){
    for (int joint_index = 0; joint_index < JOINT_NUM; joint_index++) {
        Joint[joint_index].dof = joint_dof_map[joint_index];
    }
}
void joint_init(Joint_t *Joint){

    // Joint 自由度类型初始化
    joint_dof_init(Joint);

    // 关节电机初始化
    joint_motor_init(Joint);

}
// /**
//  * @brief 电机初始化
//  * 
//  */
// void joint_motor_init(void)
// {
//     for (int joint_index = 0; joint_index < JOINT_NUM; joint_index++) {
//         Joint_Motor[joint_index] = pvPortMalloc(sizeof(DM_motor_t));
//         //配置 can2
//         Joint_Motor[joint_index]->can_cfg.port = CAN2_PORT;
//         Joint_Motor[joint_index]->tmp.PMAX = 12.5f;
//         Joint_Motor[joint_index]->tmp.VMAX = 3.0f;
//         Joint_Motor[joint_index]->tmp.TMAX = 1.0f;
//         // 配置can id
//         Joint_Motor[joint_index]->can_cfg.id = 0x01 + joint_index;
//         Joint_Motor[joint_index]->motor_msg.can_msg.id = 0x11+ joint_index;
//         Motor_DM_Init(Joint_Motor[joint_index]);
//     }
// }
