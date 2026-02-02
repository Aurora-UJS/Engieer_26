#include "test_can3_arm_task.h"
#include "cmsis_os2.h"
#include "joint_control_drv.h"
#include "jointFollowAngle.h"
#include "motor_DM.h"
extern target_point_t Target_Point[6];
extern Joint_t Joint[JOINT_NUM];
void test_CAN3_ARM(void *argument){
   UNUSED(argument) ;
    while (true) {
        PosSpeed_CtrlMotorDM(Joint[0].joint_motor, Target_Point[0].target_joint_radian, Target_Point[0].velocity);
        PosSpeed_CtrlMotorDM(Joint[1].joint_motor, Target_Point[1].target_joint_radian, Target_Point[1].velocity);
        osDelay(1);
    }
}
