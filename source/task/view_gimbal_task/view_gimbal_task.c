#include "cmsis_os2.h"
#include "tim.h"

void View_Gimbal_Task(void *argument){
  UNUSED(argument);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  int ecd = 1500;
  while(1)
  {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ecd);
    osDelay(10);
  }
}
