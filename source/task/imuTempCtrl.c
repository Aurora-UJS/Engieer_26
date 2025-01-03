
#include "usart.h"
#include "tim.h"
#include "IMUtool.h"
#include "cmsis_os2.h"
#include "DbusSys.h"
#include "usart.h"
#include "BMI088driver.h"
#include "motor_DJI.h"
#include "chassismovement.h"
#include"chassis_Calculation.h"  

float angle[4];
float yaw_update;
float a, b;
uint8_t count;

void IMU_TempCtrlTask(void const *argument)
{
  (void)argument; // UNUSED(argument)
  osDelay(500);
  while (BMI088_init())
    ;
    
  DJI_motor_init();
  chassis_pid_init();
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  Mahony_Init(1000);
  DBus_Init();
  while (1)
  {
    if (remoter.sw1 == 3 && remoter.sw2 == 1)
    {

      chassis_vector_to_mecanum_wheel_speed();
      chassis_move();
    }
    if (remoter.sw2 == 2)
    {
      set_motor_voltage(DJMotor_chassis, 0, 0, 0, 0); // chassis_stop();
    }
    IMUsys();
    DBus_Refresh();
    Motor_DJI_Refresh(DJMotor_chassis);



    osDelay(1);
  }
}
