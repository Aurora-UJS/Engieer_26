#ifndef __DM_IMU_H
#define __DM_IMU_H

#include "main.h"
#include "can_struct.h"

#define ACCEL_CAN_MAX (58.8f)
#define ACCEL_CAN_MIN	(-58.8f)
#define GYRO_CAN_MAX	(34.88f)
#define GYRO_CAN_MIN	(-34.88f)
#define PITCH_CAN_MAX	(90.0f)
#define PITCH_CAN_MIN	(-90.0f)
#define ROLL_CAN_MAX	(180.0f)
#define ROLL_CAN_MIN	(-180.0f)
#define YAW_CAN_MAX		(180.0f)
#define YAW_CAN_MIN 	(-180.0f)
#define TEMP_MIN			(0.0f)
#define TEMP_MAX			(60.0f)
#define Quaternion_MIN	(-1.0f)
#define Quaternion_MAX	(1.0f)



typedef struct
{

	struct
	{
		float gyro[3];
		float accel[3];
		float cur_temp;
	} GyroscopeDataPacket;
	 struct
	{
		float q[4];
	} Quarternion;

	 struct
	{
		float pitch;
		float roll;
		float yaw;
	} Angles;
	struct 
	{
	    can_msg_t can_msg;    
	} imu_msg;
	can_msg_t can_cfg;

}DM_imu_t;


extern	DM_imu_t *imu;

void IMU_UpdateData(uint8_t* pData);
void IMU_RequestData(DM_imu_t *imu,uint16_t can_id,uint8_t reg);
void DM_IMU_Init(DM_imu_t *imu);
void IMU_Refresh(DM_imu_t *imu);

#endif