#include "cmsis_os2.h"
#include "tim.h"

void View_Gimbal_Task(void *argument){
  UNUSED(argument);
  __HAL_TIM_MOE_ENABLE(&htim1); 
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  while(1)
  {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 1500);
    osDelay(2);
  }
}
