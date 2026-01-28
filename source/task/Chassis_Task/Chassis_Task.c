#include "Chassis_Task.h"
#include "chassis_drive.h"
#include "rising_ctrl.h"
#include "DbusSys.h"
#include "arm_math_types.h"
#include "cmsis_os2.h"
#include "tool.h"
#include <stdint.h>

// ch1 右摇杆 左右 左-右+
// ch2 右摇杆 前后 前+后-
// ch3 左摇杆 左右 左-右+
// ch4 左摇杆 前后 前+后-
// sw1 左拨码开关 前1 中3 后2
// sw2 右拨码开关 前1 中3 后2

extern rc_info_t remoter;
float32_t target_speed_test[4];
float32_t current_speed_test[4];

static volatile uint8_t s_chassis_force_poweroff = 0;

void Chassis_ForcePowerOff(uint8_t enable)
{
  s_chassis_force_poweroff = (enable != 0U) ? 1U : 0U;
}

uint8_t Chassis_IsForcePowerOff(void)
{
  return s_chassis_force_poweroff;
}

/**
 * @brief 根据遥控器拨码开关获取底盘模式
 *
 * @param backdata 遥控器数据指针
 * @return 当前底盘模式
 */
static uint8_t Chassis_Mode_Get(rc_info_t *backdata)
{
  if (Chassis_IsForcePowerOff() != 0U) {
    return Chassis_PowerOff;
  }

  uint8_t Chassis_Mode;
  switch (backdata->sw2)
  {
    case 1:
      Chassis_Mode = Chassis_Upstairs;
      break;
    case 2:
      Chassis_Mode = Chassis_PowerOff;
      break;
    case 3:
      Chassis_Mode = Chassis_Normal;
      break;
    default:
      Chassis_Mode = Chassis_PowerOff;
  }
  return Chassis_Mode;
}

/**
 * @brief 底盘任务入口函数
 *
 * 负责：读取遥控器、切换模式、调用底盘/抬升控制模块下发电机指令。
 */
void Chassis_Task(void *argument) 
{
  /* USER CODE Chassis_Task */
  (void)argument;
  osDelay(200);

  Chassis_Drive_Init();
  Rising_Ctrl_Init();

  /* Infinite loop */
  for (;;) 
  {
    switch (Chassis_Mode_Get(&remoter)) 
    {
      case Chassis_PowerOff:
        Chassis_Stop();
        Rising_Stop();
        break;

      case Chassis_Normal:
        Chassis_Normal_Mode(&remoter);
        Rising_Normal_Mode(&remoter);
        break;

      case Chassis_Upstairs:
        Chassis_Upstairs_Mode(&remoter);
        Rising_Upstairs_Mode(&remoter);
        break;

      default:
        break;
    }
    osDelay(2);
  }
  /* USER CODE END Chassis_Task */
}
