#include "cmsis_os2.h"

#include "servo_drv.h"


void View_Gimbal_Task(void *argument){
  UNUSED(argument);
  servo_t view_gimbal_yaw,view_gimbal_pitch;
  
  servo_init(&view_gimbal_yaw, &htim1, TIM_CHANNEL_1);
  servo_init(&view_gimbal_pitch, &htim1, TIM_CHANNEL_3);

  servo_setPos(&view_gimbal_pitch, 180);
  servo_setPos(&view_gimbal_yaw, 180);
  while(1)
  {
    servo_drive(&view_gimbal_pitch);
    servo_drive(&view_gimbal_yaw);
    osDelay(10);
  }
}
