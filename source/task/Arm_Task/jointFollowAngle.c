// jointFollowAngle.c 关节跟随角度运动处理函数

#include "joint_control_drv.h"

extern float joint_radian[6];
extern DM_motor_t *Joint_Motor[JOINT_NUM];
extern float joint_radian[6];

void jointFollowAngle(void *argument)
{
    UNUSED(argument);
    joint_motor_init();
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
        // PosSpeed_CtrlMotorDM(Joint_Motor[0],joint_radian[0], 1);
        PosSpeed_CtrlMotorDM(Joint_Motor[1],limit(joint_radian[1], 0, 1.5), 0.5); // pitch轴控制
        osDelay(1);
        PosSpeed_CtrlMotorDM(Joint_Motor[2],limit(-joint_radian[2], 0, 3), 0.5); // pitch轴控制
        osDelay(1);
        PosSpeed_CtrlMotorDM(Joint_Motor[3],-joint_radian[3],  0.5); // roll轴控制
        osDelay(1);
        PosSpeed_CtrlMotorDM(Joint_Motor[4],limit(-joint_radian[4],-1.5, 1.5), 0.5); // pitch轴控制
        osDelay(1);
        PosSpeed_CtrlMotorDM(Joint_Motor[5],-joint_radian[5], 0.5); // roll轴控制
        osDelay(1);

        switch(remoter.sw1)
        {
            case 3:
                PosSpeed_CtrlMotorDM(Joint_Motor[6],0, 0.5); // 夹爪控制
                osDelay(1);
            break;
            //1是前，夹爪张开
            case 2:
                PosSpeed_CtrlMotorDM(Joint_Motor[6],1.2, 0.5); // 夹爪控制
                osDelay(1);
            break;
            //2是后，夹爪合拢

        }
    }
}
