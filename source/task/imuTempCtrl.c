
#include "usart.h"
#include "tim.h"
#include "IMUtool.h"
#include "cmsis_os2.h"



float angle[4];
float yaw_update;
float a,b;
uint8_t count;




void IMU_TempCtrlTask(void const * argument)
{
    osDelay(500);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    Mahony_Init(1000);

    while(1)
    {

	
		IMUsys();	

		osDelay(1);	
    }
}

