
#include "usart.h"
#include "tim.h"
#include "IMUtool.h"
#include "cmsis_os2.h"
#include "BMI088driver.h"
float angle[4];
float yaw_update;
float a, b;
uint8_t count;

void IMU_TempCtrlTask(void const *argument)
{
  UNUSED(argument);
  osDelay(500);
  Mahony_Init(1000);
  BMI088_init();
  while (1)
  {
    IMUsys();

    osDelay(1);
  }
}
