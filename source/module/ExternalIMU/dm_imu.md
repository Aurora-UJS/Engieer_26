# 达秒IMU模块使用说明

## 1. 示例代码

```C
{
   	 imu = pvPortMalloc(sizeof(DM_imu_t));

     imu->can_cfg.port = CAN3_PORT;

     imu->can_cfg.id = 0x6FF;

  	 imu->can_cfg.len = FDCAN_DLC_BYTES_4;

	 imu->can_cfg.port=CAN3_PORT;

	 imu->imu_msg.can_msg.id=0x11; //这个取决于上位机设置的msgid


    DM_IMU_Init(imu);
    // 开始无限循环，持续对IMU发送命令
    while (1)
    {
			tick_ms++;
			
			if(tick_ms%3==0)
			{
				IMU_RequestData(imu,0x01,1) ;   //请求加速度数据
			}
			else if(tick_ms%2==0)
			{
				IMU_RequestData(imu,0x01,2);  //请求角速度数据
			}
			else if(tick_ms%1==0)
			{
				IMU_RequestData(imu,0x01,3)    //请求欧拉角数据
			}
			//！！！！这三种请求依据情况来选用
			if(tick_ms>1000)
			tick_ms=0;
			IMU_Refresh(imu);

    }
}

```
