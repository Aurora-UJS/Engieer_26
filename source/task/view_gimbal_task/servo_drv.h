#ifndef SERVO_DRV_H
#include "tim.h"
#include "stm32h7xx_hal_tim.h"
typedef struct {
  TIM_HandleTypeDef *htim; 
  uint32_t Channel;
  int16_t position; 
} servo_t;
void servo_init(servo_t *servo,TIM_HandleTypeDef *htim, uint32_t Channel);
void servo_drive(servo_t* servo);
void servo_setPos(servo_t* servo,float position);
#endif // !SERVO_DRV_H
