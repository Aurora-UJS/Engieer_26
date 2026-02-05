#include "IMU_Task.h"
#include "dm_imu.h"
#include "arm_math_types.h"
#include "cmsis_os2.h"

IMU_data_t IMU_data;
int64_t IMU_data_time=0;

void IMU_Task(void *argument)
{
    UNUSED(argument);
    imu = pvPortMalloc(sizeof(DM_imu_t));
    imu->can_cfg.port = CAN3_PORT;
    imu->can_cfg.id = 0x21;
    imu->can_cfg.len = FDCAN_DLC_BYTES_4;
    imu->can_cfg.port=CAN3_PORT;
    imu->imu_msg.can_msg.id=0x22; //这个取决于上位机设置的msgid
    DM_IMU_Init(imu);
    osDelay(100);
    // 开始无限循环，持续对IMU发送命令
    for(;;)
    {
		IMU_RequestData(imu,0x21,3);    //请求欧拉角数据
        IMU_data.Pitch = imu->Angles.pitch;
        IMU_data.Yaw = imu->Angles.yaw;
        IMU_data.Roll = imu->Angles.roll;
		IMU_Refresh(imu);
        IMU_data_time++;
        osDelay(10);
    }
}
