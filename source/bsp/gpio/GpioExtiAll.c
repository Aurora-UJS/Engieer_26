#include "GpioExtiAll.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"

extern osSemaphoreId imuBinarySem01Handle;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == ACC_INT_Pin)
    {
        osSemaphoreRelease(imuBinarySem01Handle);
    }
    else if(GPIO_Pin == GYRO_INT_Pin)
    {

    }
}