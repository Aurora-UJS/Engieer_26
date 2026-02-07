#include "Chassis_Task.h"
#include "chassis_drive.h"
#include "rising_ctrl.h"
#include "DBusSys.h"
#include "Referee_Task.h"
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
extern keyboard_t kb_info;
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

  uint8_t last_chassis_mode = Chassis_Mode_Get(&remoter);
  uint8_t last_ctrl_logic_mode = Engineer_Mode.Ctrl_Logic_Mode;
  uint8_t last_keyboard_chassis_ctrl_mode = Engineer_Mode.Chassis_Ctrl_Mode;
  uint8_t arm_ctrl_mode_saved = Engineer_Mode.Arm_Ctrl_Mode;
  uint8_t arm_ctrl_mode_saved_valid = 0U;

  /* Infinite loop */
  for (;;) 
  {
    const uint8_t chassis_mode = Chassis_Mode_Get(&remoter);
    if (chassis_mode != last_chassis_mode) {
      /* 底盘模式切换瞬间：清零抬升 DM（IMU 闭环）PID，防止切换冲击 */
      Rising_Reset_DmImuPid();
      last_chassis_mode = chassis_mode;
    }

    /* 断电模式：底盘与抬升全部停转 */
    if (chassis_mode == Chassis_PowerOff) {
      Chassis_Stop();
      Rising_Stop();
      osDelay(2);
      continue;
    }

    /* 从键盘逻辑切回遥控器逻辑时，如果曾经强制切到 ARM_CTRL_MODE_Rising，需要恢复旧的 arm 模式 */
    if ((last_ctrl_logic_mode == CTRL_LOGIC_MODE_Keyboard) &&
        (Engineer_Mode.Ctrl_Logic_Mode == CTRL_LOGIC_MODE_DBUS) &&
        (arm_ctrl_mode_saved_valid != 0U)) {
      Engineer_Mode.Arm_Ctrl_Mode = arm_ctrl_mode_saved;
      arm_ctrl_mode_saved_valid = 0U;
    }
    last_ctrl_logic_mode = Engineer_Mode.Ctrl_Logic_Mode;

    /* 一级状态机：选择遥控器（DBUS）控制 or 键盘（Keyboard）控制 */
    if (Engineer_Mode.Ctrl_Logic_Mode == CTRL_LOGIC_MODE_DBUS) {
      /* DBUS 模式：保持现有控制逻辑不变 */
      switch (chassis_mode) 
      {
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
    } else {
      /* Keyboard 模式：二级状态机 - 根据 Engineer_Mode.Chassis_Ctrl_Mode 决定 Normal/Rising */
      if (Engineer_Mode.Chassis_Ctrl_Mode != last_keyboard_chassis_ctrl_mode) {
        if (Engineer_Mode.Chassis_Ctrl_Mode == CHASSIS_CTRL_MODE_Rising) {
          /* 进入 Rising 底盘控制：记录当前 Arm 模式，并强制切换到 Rising */
          arm_ctrl_mode_saved = Engineer_Mode.Arm_Ctrl_Mode;
          arm_ctrl_mode_saved_valid = 1U;
          Engineer_Mode.Arm_Ctrl_Mode = ARM_CTRL_MODE_Rising;
        } else if ((last_keyboard_chassis_ctrl_mode == CHASSIS_CTRL_MODE_Rising) && (arm_ctrl_mode_saved_valid != 0U)) {
          /* 退出 Rising 底盘控制：恢复进入前记录的 Arm 模式 */
          Engineer_Mode.Arm_Ctrl_Mode = arm_ctrl_mode_saved;
          arm_ctrl_mode_saved_valid = 0U;
        }
        last_keyboard_chassis_ctrl_mode = Engineer_Mode.Chassis_Ctrl_Mode;
      }

      if (Engineer_Mode.Chassis_Ctrl_Mode == CHASSIS_CTRL_MODE_Normal) {
        /* Normal：底盘由 WASD + 鼠标控制，抬升保持关闭 */
        Chassis_Keyboard_Mode(&kb_info, 0U);
        Rising_Normal_Mode(&remoter);
      } else {
        /* Rising：底盘由 WASD 控制，禁止 yaw（wz），抬升动作保持原有 Rising_Upstairs_Mode 逻辑 */
        Chassis_Keyboard_Mode(&kb_info, 1U);
        rc_info_t rising_rc = remoter;
        rising_rc.ch2 = 200;
        Rising_Upstairs_Mode(&rising_rc);
      }
    }
    osDelay(2);
  }
  /* USER CODE END Chassis_Task */
}
