#include "dm_imu.h"
#include "can_api.h"
#include "fdcan.h"
#include "motor_DM.h"
#include "uint_float_convert.h"

DM_imu_t *imu;


void DM_IMU_Init(DM_imu_t *imu)
{
	 imu->imu_msg.can_msg.port=imu->can_cfg.port;
	 can_msg_add_item(&imu->imu_msg.can_msg);

}

/**
 * @brief 获取陀螺仪数据。
 * 
 * @param canid 根据自己设置的canid来请求陀螺仪的数据。
 * @param reg reg可选用值为1 2 3 4分别对应 加速度数据 角加速度数据 欧拉角数据 四元数数据。
 * @return 空
 */
void IMU_RequestData(DM_imu_t *imu,uint16_t can_id,uint8_t reg)
{

	imu->can_cfg.data[0] = (uint8_t)can_id;
	imu->can_cfg.data[1] = (uint8_t)(can_id>>8);
	imu->can_cfg.data[2] = reg;
	imu->can_cfg.data[3] = 0xCC;
	can_msg_send_classical(&imu->can_cfg);

}


void IMU_UpdateAccel(uint8_t* pData)
{
	uint16_t accel[3];
	
	accel[0]=pData[3]<<8|pData[2];
	accel[1]=pData[5]<<8|pData[4];
	accel[2]=pData[7]<<8|pData[6];
	
	imu->GyroscopeDataPacket.accel[0]=uint_to_float(accel[0],ACCEL_CAN_MIN,ACCEL_CAN_MAX,16);
	imu->GyroscopeDataPacket.accel[1]=uint_to_float(accel[1],ACCEL_CAN_MIN,ACCEL_CAN_MAX,16);
	imu->GyroscopeDataPacket.accel[2]=uint_to_float(accel[2],ACCEL_CAN_MIN,ACCEL_CAN_MAX,16);
	
}

void IMU_UpdateGyro(uint8_t* pData)
{
	uint16_t gyro[3];
	gyro[0]=pData[3]<<8|pData[2];
	gyro[1]=pData[5]<<8|pData[4];
	gyro[2]=pData[7]<<8|pData[6];

	imu->GyroscopeDataPacket.gyro[0]=uint_to_float(gyro[0],GYRO_CAN_MIN,GYRO_CAN_MAX,16);
	imu->GyroscopeDataPacket.gyro[1]=uint_to_float(gyro[1],GYRO_CAN_MIN,GYRO_CAN_MAX,16);
	imu->GyroscopeDataPacket.gyro[2]=uint_to_float(gyro[2],GYRO_CAN_MIN,GYRO_CAN_MAX,16);
}


void IMU_UpdateEuler(uint8_t* pData)
{
	int euler[3];
	
	euler[0]=pData[3]<<8|pData[2];
	euler[1]=pData[5]<<8|pData[4];
	euler[2]=pData[7]<<8|pData[6];
	
	imu->Angles.pitch=uint_to_float(euler[0],PITCH_CAN_MIN,PITCH_CAN_MAX,16);
	imu->Angles.yaw=uint_to_float(euler[1],YAW_CAN_MIN,YAW_CAN_MAX,16);
	imu->Angles.roll=uint_to_float(euler[2],ROLL_CAN_MIN,ROLL_CAN_MAX,16);

}

void IMU_UpdateDataPacket(uint8_t* pData)
{
	IMU_UpdateData(pData);
}


void IMU_UpdateQuaternion(uint8_t* pData)
{
	int w = pData[1]<<6| ((pData[2]&0xF8)>>2);
	int x = (pData[2]&0x03)<<12|(pData[3]<<4)|((pData[4]&0xF0)>>4);
	int y = (pData[4]&0x0F)<<10|(pData[5]<<2)|(pData[6]&0xC0)>>6;
	int z = (pData[6]&0x3F)<<8|pData[7];
	
	imu->Quarternion.q[0]= uint_to_float(w,Quaternion_MIN,Quaternion_MAX,14);
	imu->Quarternion.q[1] = uint_to_float(x,Quaternion_MIN,Quaternion_MAX,14);
	imu->Quarternion.q[2] = uint_to_float(y,Quaternion_MIN,Quaternion_MAX,14);
	imu->Quarternion.q[3] = uint_to_float(z,Quaternion_MIN,Quaternion_MAX,14);
}



void IMU_Refresh(DM_imu_t *imu)
{
	switch(imu->imu_msg.can_msg.data[0])
	{
		case 1:
			IMU_UpdateAccel(imu->imu_msg.can_msg.data);
			break;
		case 2:
			IMU_UpdateGyro(imu->imu_msg.can_msg.data);
			break;
		case 3:
			IMU_UpdateEuler(imu->imu_msg.can_msg.data);
			break;
		case 4:
			IMU_UpdateQuaternion(imu->imu_msg.can_msg.data);
			break;
	}
}