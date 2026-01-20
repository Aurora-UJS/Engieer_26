#include "jointFollowAngle.h"
DM_motor_t *joint_motor[JOINT_NUM];
extern uint8_t CtrllerData[24];
float joint_radian[6] = {0};
extern rc_info_t remoter;

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
