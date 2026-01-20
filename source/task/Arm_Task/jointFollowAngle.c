// jointFollowAngle.c 关节跟随角度运动处理函数

#include "ee_control_drv.h"
#include "joint_control_drv.h"

extern float joint_radian[6];
extern DM_motor_t *Joint_Motor[JOINT_NUM];

// ch1 右摇杆 左右 左-右+
// ch2 右摇杆 前后 前+后-
// ch3 左摇杆 左右 左-右+
// ch4 左摇杆 前后 前+后-
// sw1 左拨码开关 前1 中3 后2
// sw2 右拨码开关 前1 中3 后2

void jointFollowAngle(void *argument)
{
    Joint_t Joint[JOINT_NUM];
    endEffector_t EndEffector;

    
    UNUSED(argument);
    joint_init(Joint);
    endeffector_init(&EndEffector);

    osDelay(100);
    Joint_Motor_Enable(Joint); // 使能所有关节电机
    EndEffector_Motor_Enable(&EndEffector);

    while (1) {
        Joint_Motor_Refresh(Joint);
        EndEffector_Motor_Refresh(&EndEffector);
        for (int i = 0; i < 6; i++) {
            int tmp = 0;
            // 每个弧度占 4 个字符
            sscanf((const char*)&CtrllerData[i * 4], "%04d", &tmp);
            joint_radian[i] = tmp / 1000.0f;
            joint_radian[i] = joint_radian[i] - PI;
        }

        // PosSpeed_CtrlMotorDM(Joint_Motor[0],joint_radian[0], 1);
        PosSpeed_CtrlMotorDM(Joint[1].joint_motor,limit(joint_radian[1], 0, 1.5), 0.5); // pitch轴控制
        osDelay(1);
        PosSpeed_CtrlMotorDM(Joint[2].joint_motor,limit(-joint_radian[2], 0, 3), 0.5); // pitch轴控制
        osDelay(1);
        PosSpeed_CtrlMotorDM(Joint[3].joint_motor,-joint_radian[3],  0.5); // roll轴控制
        osDelay(1);
        PosSpeed_CtrlMotorDM(Joint[4].joint_motor,limit(-joint_radian[4],-1.5, 1.5), 0.5); // pitch轴控制
        osDelay(1);
        PosSpeed_CtrlMotorDM(Joint[5].joint_motor,-joint_radian[5], 0.5); // roll轴控制
        osDelay(1);

        switch(remoter.sw1)
        {
            case 3:
                PosSpeed_CtrlMotorDM(EndEffector.endEffector_motor,0, 0.5); // 夹爪控制
                osDelay(1);
            break;
            //1是前，夹爪张开
            case 2:
                PosSpeed_CtrlMotorDM(EndEffector.endEffector_motor,1.2, 0.5); // 夹爪控制
                osDelay(1);
            break;
            //2是后，夹爪合拢

        }
    }
}
