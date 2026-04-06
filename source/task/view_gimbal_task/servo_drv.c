#include "servo_drv.h"
static inline int pos_to_duty(float position){
  return (int)(500.0f + position * 2000.0f / 180.0f);
}
void servo_init(servo_t *servo,TIM_HandleTypeDef *htim, uint32_t Channel){
  servo->htim = htim;
  servo->Channel = Channel;
  HAL_TIM_PWM_Start(&htim1, Channel);
}


void servo_drive(servo_t* servo){
    __HAL_TIM_SET_COMPARE(servo->htim, servo->Channel, pos_to_duty(servo->position));
}

void servo_setPos(servo_t* servo,float position){
  servo->position = position;
}
