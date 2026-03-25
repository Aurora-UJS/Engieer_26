#include "Chassis_Task.h"
#include "chassis_drive.h"
#include "rising_ctrl.h"
#include "DBusSys.h"
#include "Referee_Task.h"
#include "arm_math_types.h"
#include "cmsis_os2.h"
#include "tool.h"
#include <stdint.h>

/* 抬升测试历程开关：
 * 0：关闭（保持原有底盘/抬升控制逻辑）
 * 1：模式1（拨码切到抬升后：先固定底盘速度一段时间，随后恢复遥控器全权控制；抬升 3508 可固定最大；DM 逻辑保持不变）
 * 2：模式2（保留模式1全部流程，并在固定底盘驱动阶段结束后：切 Chassis_Ctrl_Mode=Rising 持续一段时间，再切回 Normal 持续一段时间）
 */
#define CHASSIS_RISING_TEST_TRAJECTORY_ENABLE 2

/* 抬升测试历程参数（仅在 CHASSIS_RISING_TEST_TRAJECTORY_ENABLE=1 时生效） */
/* 抬升阶段持续时间（单位：ms）：只执行抬升动作（3508 固定最大、DM 逻辑不变），底盘不动 */
#define CHASSIS_RISING_TEST_LIFT_DURATION_MS 1600U
/* 抬升阶段底盘固定速度比例：最大速度的 (NUM/DEN)，默认 2/3 */
#define CHASSIS_RISING_TEST_LIFT_CHASSIS_SPEED_RATIO_NUM 100
#define CHASSIS_RISING_TEST_LIFT_CHASSIS_SPEED_RATIO_DEN 100
/* 抬升阶段抬升3508速度：通过固定 remoter.ch2 实现，默认 Remoter_CHMAX（速度最大） */
#define CHASSIS_RISING_TEST_LIFT_RISING_RC_CH2 Remoter_CHMAX
/* 抬升结束后切到Normal模式的停顿时间（单位：ms）：底盘停住0.2s后再进入固定速度驱动 */
#define CHASSIS_RISING_TEST_TRANSITION_DURATION_MS 0U
/* 抬升完成后的“底盘驱动阶段”持续时间（单位：ms）：底盘固定速度驱动 */
#define CHASSIS_RISING_TEST_DRIVE_DURATION_MS 600U
/* 底盘驱动阶段固定速度比例：最大速度的 (NUM/DEN)，默认 1/3 */
#define CHASSIS_RISING_TEST_DRIVE_SPEED_RATIO_NUM 70
#define CHASSIS_RISING_TEST_DRIVE_SPEED_RATIO_DEN 100

/* 抬升测试历程模式2参数（仅在 CHASSIS_RISING_TEST_TRAJECTORY_ENABLE=2 时生效）
 * 与模式1完全解耦，便于单独调参，不会影响模式1现有数据。
 */
#define CHASSIS_RISING_TEST2_LIFT_DURATION_MS 1600U
#define CHASSIS_RISING_TEST2_LIFT_CHASSIS_SPEED_RATIO_NUM 100
#define CHASSIS_RISING_TEST2_LIFT_CHASSIS_SPEED_RATIO_DEN 100
#define CHASSIS_RISING_TEST2_LIFT_RISING_RC_CH2 Remoter_CHMAX
#define CHASSIS_RISING_TEST2_TRANSITION_DURATION_MS 0U
#define CHASSIS_RISING_TEST2_DRIVE_DURATION_MS 600U
#define CHASSIS_RISING_TEST2_DRIVE_SPEED_RATIO_NUM 70
#define CHASSIS_RISING_TEST2_DRIVE_SPEED_RATIO_DEN 100
#define CHASSIS_RISING_TEST2_CTRL_RISING_DURATION_MS 1800U
#define CHASSIS_RISING_TEST2_CTRL_NORMAL_DURATION_MS 600U

#if (CHASSIS_RISING_TEST_TRAJECTORY_ENABLE == 1)
#define CHASSIS_RISING_TEST_ACTIVE_LIFT_DURATION_MS CHASSIS_RISING_TEST_LIFT_DURATION_MS
#define CHASSIS_RISING_TEST_ACTIVE_LIFT_CHASSIS_SPEED_RATIO_NUM CHASSIS_RISING_TEST_LIFT_CHASSIS_SPEED_RATIO_NUM
#define CHASSIS_RISING_TEST_ACTIVE_LIFT_CHASSIS_SPEED_RATIO_DEN CHASSIS_RISING_TEST_LIFT_CHASSIS_SPEED_RATIO_DEN
#define CHASSIS_RISING_TEST_ACTIVE_LIFT_RISING_RC_CH2 CHASSIS_RISING_TEST_LIFT_RISING_RC_CH2
#define CHASSIS_RISING_TEST_ACTIVE_TRANSITION_DURATION_MS CHASSIS_RISING_TEST_TRANSITION_DURATION_MS
#define CHASSIS_RISING_TEST_ACTIVE_DRIVE_DURATION_MS CHASSIS_RISING_TEST_DRIVE_DURATION_MS
#define CHASSIS_RISING_TEST_ACTIVE_DRIVE_SPEED_RATIO_NUM CHASSIS_RISING_TEST_DRIVE_SPEED_RATIO_NUM
#define CHASSIS_RISING_TEST_ACTIVE_DRIVE_SPEED_RATIO_DEN CHASSIS_RISING_TEST_DRIVE_SPEED_RATIO_DEN
#elif (CHASSIS_RISING_TEST_TRAJECTORY_ENABLE == 2)
#define CHASSIS_RISING_TEST_ACTIVE_LIFT_DURATION_MS CHASSIS_RISING_TEST2_LIFT_DURATION_MS
#define CHASSIS_RISING_TEST_ACTIVE_LIFT_CHASSIS_SPEED_RATIO_NUM CHASSIS_RISING_TEST2_LIFT_CHASSIS_SPEED_RATIO_NUM
#define CHASSIS_RISING_TEST_ACTIVE_LIFT_CHASSIS_SPEED_RATIO_DEN CHASSIS_RISING_TEST2_LIFT_CHASSIS_SPEED_RATIO_DEN
#define CHASSIS_RISING_TEST_ACTIVE_LIFT_RISING_RC_CH2 CHASSIS_RISING_TEST2_LIFT_RISING_RC_CH2
#define CHASSIS_RISING_TEST_ACTIVE_TRANSITION_DURATION_MS CHASSIS_RISING_TEST2_TRANSITION_DURATION_MS
#define CHASSIS_RISING_TEST_ACTIVE_DRIVE_DURATION_MS CHASSIS_RISING_TEST2_DRIVE_DURATION_MS
#define CHASSIS_RISING_TEST_ACTIVE_DRIVE_SPEED_RATIO_NUM CHASSIS_RISING_TEST2_DRIVE_SPEED_RATIO_NUM
#define CHASSIS_RISING_TEST_ACTIVE_DRIVE_SPEED_RATIO_DEN CHASSIS_RISING_TEST2_DRIVE_SPEED_RATIO_DEN
#endif

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
 * 功能说明：
 * 1. DBUS模式：通过遥控器摇杆控制底盘运动（ch1/ch2/ch3）
 * 2. Keyboard模式：通过键盘WASD控制底盘平移，鼠标X轴控制旋转
 * 3. 支持Normal和Upstairs两种底盘模式
 * 4. 支持抬升测试历程（需sw1==1触发）
 *
 * @param argument FreeRTOS任务参数（未使用）
 */
void Chassis_Task(void *argument) 
{
  /* USER CODE Chassis_Task */
  (void)argument;
  osDelay(200);

  Chassis_Drive_Init();
  Rising_Ctrl_Init();

  /* 说明：这里的“raw_chassis_mode”来自拨码开关；
   * 当开启测试历程时，会在切到抬升后自动按“抬升阶段 -> 底盘驱动阶段 -> 遥控器全权控制”执行。
   */
#if (CHASSIS_RISING_TEST_TRAJECTORY_ENABLE != 0)
  typedef enum {
    RISING_TEST_STATE_IDLE = 0,
    RISING_TEST_STATE_LIFTING,
    RISING_TEST_STATE_TRANSITION,
    RISING_TEST_STATE_DRIVING,
    RISING_TEST_STATE_CTRL_RISING,
    RISING_TEST_STATE_CTRL_NORMAL,
    RISING_TEST_STATE_PASSTHROUGH,
  } Rising_Test_State_t;

  Rising_Test_State_t rising_test_state = RISING_TEST_STATE_IDLE;
  uint32_t rising_test_start_tick = 0U;
  const uint32_t rising_test_lift_duration_ticks =
      (uint32_t)((CHASSIS_RISING_TEST_ACTIVE_LIFT_DURATION_MS * osKernelGetTickFreq()) / 1000U);
  const uint32_t rising_test_transition_duration_ticks =
      (uint32_t)((CHASSIS_RISING_TEST_ACTIVE_TRANSITION_DURATION_MS * osKernelGetTickFreq()) / 1000U);
  const uint32_t rising_test_drive_duration_ticks =
      (uint32_t)((CHASSIS_RISING_TEST_ACTIVE_DRIVE_DURATION_MS * osKernelGetTickFreq()) / 1000U);
#if (CHASSIS_RISING_TEST_TRAJECTORY_ENABLE == 2)
  const uint32_t rising_test_ctrl_rising_duration_ticks =
      (uint32_t)((CHASSIS_RISING_TEST2_CTRL_RISING_DURATION_MS * osKernelGetTickFreq()) / 1000U);
  const uint32_t rising_test_ctrl_normal_duration_ticks =
      (uint32_t)((CHASSIS_RISING_TEST2_CTRL_NORMAL_DURATION_MS * osKernelGetTickFreq()) / 1000U);
  uint8_t rising_test_ctrl_mode_override_active = 0U;
  uint8_t rising_test_saved_chassis_ctrl_mode = Engineer_Mode.Chassis_Ctrl_Mode;
#endif

  uint8_t last_raw_chassis_mode = Chassis_Mode_Get(&remoter);
  uint8_t last_effective_chassis_mode = last_raw_chassis_mode;
#else
  uint8_t last_chassis_mode = Chassis_Mode_Get(&remoter);
#endif
  uint8_t last_ctrl_logic_mode = Engineer_Mode.Ctrl_Logic_Mode;
  uint8_t last_keyboard_chassis_ctrl_mode = Engineer_Mode.Chassis_Ctrl_Mode;
  uint8_t arm_ctrl_mode_saved = Engineer_Mode.Arm_Ctrl_Mode;
  uint8_t arm_ctrl_mode_saved_valid = 0U;

  /* Infinite loop */
  for (;;) 
  {
    const uint8_t chassis_mode = Chassis_Mode_Get(&remoter);

#if (CHASSIS_RISING_TEST_TRAJECTORY_ENABLE != 0)
    /* 抬升测试历程：
     * 触发条件：拨码从非抬升切到抬升（上升沿）且 sw1 == 1
     * 行为：
     * 1) 抬升阶段：抬升 3508 固定最大、DM 逻辑保持不变，底盘不动，持续一段时间
     * 2) 抬升结束后：底盘模式强制切回 Normal，并停住0.2s
     * 3) 停顿结束后：底盘固定速度驱动一段时间，再恢复Normal保持
     * 注意：如果 sw1 != 1，则不会触发测试历程，按照测试历程关闭的逻辑执行
     */
    const uint8_t raw_chassis_mode = chassis_mode;

#if (CHASSIS_RISING_TEST_TRAJECTORY_ENABLE == 2)
    if ((last_ctrl_logic_mode == CTRL_LOGIC_MODE_DBUS) &&
        (Engineer_Mode.Ctrl_Logic_Mode != CTRL_LOGIC_MODE_DBUS) &&
        (rising_test_ctrl_mode_override_active != 0U)) {
      Engineer_Mode.Chassis_Ctrl_Mode = rising_test_saved_chassis_ctrl_mode;
      last_keyboard_chassis_ctrl_mode = Engineer_Mode.Chassis_Ctrl_Mode;
      rising_test_ctrl_mode_override_active = 0U;
      rising_test_state = RISING_TEST_STATE_IDLE;
    }
#endif

    if (Engineer_Mode.Ctrl_Logic_Mode == CTRL_LOGIC_MODE_DBUS) {
      /* 只有当 sw1 == 1 时才允许触发测试历程 */
      if ((remoter.sw1 == 1) && (raw_chassis_mode == Chassis_Upstairs) && (last_raw_chassis_mode != Chassis_Upstairs)) {
        rising_test_state = RISING_TEST_STATE_LIFTING;
        rising_test_start_tick = osKernelGetTickCount();
#if (CHASSIS_RISING_TEST_TRAJECTORY_ENABLE == 2)
        if (rising_test_ctrl_mode_override_active != 0U) {
          Engineer_Mode.Chassis_Ctrl_Mode = rising_test_saved_chassis_ctrl_mode;
          last_keyboard_chassis_ctrl_mode = Engineer_Mode.Chassis_Ctrl_Mode;
          rising_test_ctrl_mode_override_active = 0U;
        }
#endif
      }

      /* 如果 sw1 != 1 或者拨码不在抬升位置，则退出测试历程 */
      if ((remoter.sw1 != 1) || (raw_chassis_mode != Chassis_Upstairs)) {
        rising_test_state = RISING_TEST_STATE_IDLE;
#if (CHASSIS_RISING_TEST_TRAJECTORY_ENABLE == 2)
        if (rising_test_ctrl_mode_override_active != 0U) {
          Engineer_Mode.Chassis_Ctrl_Mode = rising_test_saved_chassis_ctrl_mode;
          last_keyboard_chassis_ctrl_mode = Engineer_Mode.Chassis_Ctrl_Mode;
          rising_test_ctrl_mode_override_active = 0U;
        }
#endif
      } else {
        const uint32_t now = osKernelGetTickCount();
        if (rising_test_state == RISING_TEST_STATE_LIFTING) {
          if ((uint32_t)(now - rising_test_start_tick) >= rising_test_lift_duration_ticks) {
            rising_test_state = RISING_TEST_STATE_TRANSITION;
            rising_test_start_tick = now;
          }
        } else if (rising_test_state == RISING_TEST_STATE_TRANSITION) {
          if ((uint32_t)(now - rising_test_start_tick) >= rising_test_transition_duration_ticks) {
            rising_test_state = RISING_TEST_STATE_DRIVING;
            rising_test_start_tick = now;
          }
        } else if (rising_test_state == RISING_TEST_STATE_DRIVING) {
          if ((uint32_t)(now - rising_test_start_tick) >= rising_test_drive_duration_ticks) {
#if (CHASSIS_RISING_TEST_TRAJECTORY_ENABLE == 2)
            rising_test_state = RISING_TEST_STATE_CTRL_RISING;
            rising_test_start_tick = now;
#else
            rising_test_state = RISING_TEST_STATE_PASSTHROUGH;
#endif
          }
#if (CHASSIS_RISING_TEST_TRAJECTORY_ENABLE == 2)
        } else if (rising_test_state == RISING_TEST_STATE_CTRL_RISING) {
          if ((uint32_t)(now - rising_test_start_tick) >= rising_test_ctrl_rising_duration_ticks) {
            rising_test_state = RISING_TEST_STATE_CTRL_NORMAL;
            rising_test_start_tick = now;
          }
        } else if (rising_test_state == RISING_TEST_STATE_CTRL_NORMAL) {
          if ((uint32_t)(now - rising_test_start_tick) >= rising_test_ctrl_normal_duration_ticks) {
            rising_test_state = RISING_TEST_STATE_PASSTHROUGH;
          }
#endif
        }
      }

#if (CHASSIS_RISING_TEST_TRAJECTORY_ENABLE == 2)
      if ((rising_test_state == RISING_TEST_STATE_CTRL_RISING) ||
          (rising_test_state == RISING_TEST_STATE_CTRL_NORMAL)) {
        if (rising_test_ctrl_mode_override_active == 0U) {
          rising_test_saved_chassis_ctrl_mode = Engineer_Mode.Chassis_Ctrl_Mode;
          rising_test_ctrl_mode_override_active = 1U;
        }

        if (rising_test_state == RISING_TEST_STATE_CTRL_RISING) {
          Engineer_Mode.Chassis_Ctrl_Mode = CHASSIS_CTRL_MODE_Rising;
        } else {
          Engineer_Mode.Chassis_Ctrl_Mode = CHASSIS_CTRL_MODE_Normal;
        }

        last_keyboard_chassis_ctrl_mode = Engineer_Mode.Chassis_Ctrl_Mode;
      } else if (rising_test_ctrl_mode_override_active != 0U) {
        Engineer_Mode.Chassis_Ctrl_Mode = rising_test_saved_chassis_ctrl_mode;
        last_keyboard_chassis_ctrl_mode = Engineer_Mode.Chassis_Ctrl_Mode;
        rising_test_ctrl_mode_override_active = 0U;
      }
#endif
    }
    last_raw_chassis_mode = raw_chassis_mode;

    /* 抬升结束后（进入 TRANSITION / DRIVING / PASSTHROUGH），即使拨码仍处于抬升，也强制切回 Normal */
    uint8_t effective_chassis_mode = raw_chassis_mode;
    if ((raw_chassis_mode == Chassis_Upstairs) &&
        ((rising_test_state == RISING_TEST_STATE_TRANSITION) ||
         (rising_test_state == RISING_TEST_STATE_DRIVING) ||
         (rising_test_state == RISING_TEST_STATE_CTRL_RISING) ||
         (rising_test_state == RISING_TEST_STATE_CTRL_NORMAL) ||
         (rising_test_state == RISING_TEST_STATE_PASSTHROUGH))) {
      effective_chassis_mode = Chassis_Normal;
    }

    if (effective_chassis_mode != last_effective_chassis_mode) {
      /* 底盘模式切换瞬间：清零抬升 DM（IMU 闭环）PID，防止切换冲击 */
      Rising_Reset_DmImuPid();
      last_effective_chassis_mode = effective_chassis_mode;
    }
#else
    if (chassis_mode != last_chassis_mode) {
      /* 底盘模式切换瞬间：清零抬升 DM（IMU 闭环）PID，防止切换冲击 */
      Rising_Reset_DmImuPid();
      last_chassis_mode = chassis_mode;
    }
#endif

    /* 断电模式：底盘与抬升全部停转
     * 注意：无论DBUS还是Keyboard模式，PowerOff都会立即停止所有电机
     */
    if (chassis_mode == Chassis_PowerOff) {
      Chassis_Stop();
      Rising_Stop();
      osDelay(2);
      continue;
    }

    /* 控制逻辑模式切换处理：
     * 从Keyboard模式切回DBUS模式时，如果之前强制切换了机械臂模式，需要恢复
     * 这是为了保证模式切换不会影响机械臂的控制状态
     */
    if ((last_ctrl_logic_mode == CTRL_LOGIC_MODE_Keyboard) &&
        (Engineer_Mode.Ctrl_Logic_Mode == CTRL_LOGIC_MODE_DBUS) &&
        (arm_ctrl_mode_saved_valid != 0U)) {
      Engineer_Mode.Arm_Ctrl_Mode = arm_ctrl_mode_saved;
      arm_ctrl_mode_saved_valid = 0U;
    }
    last_ctrl_logic_mode = Engineer_Mode.Ctrl_Logic_Mode;

    /* ========== 一级状态机：控制源选择 ==========
     * CTRL_LOGIC_MODE_DBUS: 使用遥控器摇杆控制（remoter.ch1/ch2/ch3）
     * CTRL_LOGIC_MODE_Keyboard: 使用键盘鼠标控制（WASD + 鼠标）
     */
    if (Engineer_Mode.Ctrl_Logic_Mode == CTRL_LOGIC_MODE_DBUS) {
      /* ===== DBUS模式：遥控器摇杆控制 ===== */
      /* 底盘运动由remoter的ch1(左右)/ch2(前后)/ch3(旋转)控制 */
      switch (
#if (CHASSIS_RISING_TEST_TRAJECTORY_ENABLE != 0)
              effective_chassis_mode
#else
              chassis_mode
#endif
      ) 
      {
        case Chassis_Normal:
          /* Normal模式：底盘四轮全向移动，抬升机构停转 */
          if ((chassis_mode == Chassis_Upstairs) && (rising_test_state == RISING_TEST_STATE_TRANSITION)) {
            /* 两阶段之间停0.2s：底盘已切到Normal并停住，抬升保持Normal角度 */
            rc_info_t chassis_rc = remoter;
            chassis_rc.ch1 = 0;
            chassis_rc.ch2 = 0;
            chassis_rc.ch3 = 0;
            Chassis_Normal_Mode(&chassis_rc);
            Rising_Normal_Hold_Mode();
          } else if ((chassis_mode == Chassis_Upstairs) && (rising_test_state == RISING_TEST_STATE_DRIVING)) {
            /* 测试历程-驱动阶段：底盘固定速度前进 */
            rc_info_t chassis_rc = remoter;
            chassis_rc.ch1 = 0;
            chassis_rc.ch2 = (int16_t)((Remoter_CHMAX * CHASSIS_RISING_TEST_ACTIVE_DRIVE_SPEED_RATIO_NUM) /
                                       CHASSIS_RISING_TEST_ACTIVE_DRIVE_SPEED_RATIO_DEN);
            Chassis_Normal_Mode(&chassis_rc);

            /* 第一阶段结束后切回Normal保持：3508平滑收零，DM锁定Normal角度 */
            Rising_Normal_Hold_Mode();
          } else if ((chassis_mode == Chassis_Upstairs) &&
                     ((rising_test_state == RISING_TEST_STATE_CTRL_RISING) ||
                      (rising_test_state == RISING_TEST_STATE_CTRL_NORMAL))) {
            /* 模式2附加阶段：仅切换控制模式标志位，底盘/抬升执行保持与模式1结束后一致 */
            Chassis_Normal_Mode(&remoter);
            Rising_Normal_Hold_Mode();
          } else if ((chassis_mode == Chassis_Upstairs) && (rising_test_state == RISING_TEST_STATE_PASSTHROUGH)) {
            /* 测试历程结束后保持Normal：抬升维持Normal角度，直到下次重新进入Rising */
            Chassis_Normal_Mode(&remoter);
            Rising_Normal_Hold_Mode();
          } else {
            /* 正常情况：直接使用遥控器数据控制底盘 */
            Chassis_Normal_Mode(&remoter);
            /* 抬升机构保持停转状态 */
            Rising_Normal_Mode(&remoter);
          }
          break;

        case Chassis_Upstairs:
          /* Upstairs模式：底盘前后移动，抬升机构联动，DM电机IMU闭环 */
#if (CHASSIS_RISING_TEST_TRAJECTORY_ENABLE != 0)
          if (rising_test_state == RISING_TEST_STATE_LIFTING) {
            /* 测试历程-抬升阶段：底盘固定速度，抬升3508最大速度 */
            rc_info_t chassis_rc = remoter;
            chassis_rc.ch1 = 0;
            chassis_rc.ch2 = (int16_t)((Remoter_CHMAX * CHASSIS_RISING_TEST_ACTIVE_LIFT_CHASSIS_SPEED_RATIO_NUM) /
                                       CHASSIS_RISING_TEST_ACTIVE_LIFT_CHASSIS_SPEED_RATIO_DEN);
            Chassis_Upstairs_Mode(&chassis_rc);

            /* 抬升 3508 固定最大值；DM 抬升电机逻辑保持 Rising_Upstairs_Mode 内部不变 */
            rc_info_t rising_rc = remoter;
            rising_rc.ch2 = (int16_t)(CHASSIS_RISING_TEST_ACTIVE_LIFT_RISING_RC_CH2);
            Rising_Upstairs_Mode(&rising_rc);
          } else {
            /* 非测试历程或测试历程结束：正常Upstairs控制 */
            Chassis_Upstairs_Mode(&remoter);
            Rising_Upstairs_Mode(&remoter);
          }
#else
          Chassis_Upstairs_Mode(&remoter);
          Rising_Upstairs_Mode(&remoter);
#endif
          break;

        default:
          break;
      }
    } else {
      /* ===== Keyboard模式：键盘鼠标控制 =====
       * 底盘运动由WASD键控制平移，鼠标X轴控制旋转
       * 二级状态机：根据Engineer_Mode.Chassis_Ctrl_Mode决定Normal/Rising
       */
      /* 键盘模式下的底盘控制模式切换处理 */
      if (Engineer_Mode.Chassis_Ctrl_Mode != last_keyboard_chassis_ctrl_mode) {
        if (Engineer_Mode.Chassis_Ctrl_Mode == CHASSIS_CTRL_MODE_Rising) {
          /* 进入Rising模式：保存当前机械臂模式，并强制切换到Rising状态 */
          arm_ctrl_mode_saved = Engineer_Mode.Arm_Ctrl_Mode;
          arm_ctrl_mode_saved_valid = 1U;
          Engineer_Mode.Arm_Ctrl_Mode = ARM_CTRL_MODE_Rising;
        } else if ((last_keyboard_chassis_ctrl_mode == CHASSIS_CTRL_MODE_Rising) && (arm_ctrl_mode_saved_valid != 0U)) {
          /* 退出Rising模式：恢复之前保存的机械臂模式 */
          Engineer_Mode.Arm_Ctrl_Mode = arm_ctrl_mode_saved;
          arm_ctrl_mode_saved_valid = 0U;
        }
        last_keyboard_chassis_ctrl_mode = Engineer_Mode.Chassis_Ctrl_Mode;
      }

      /* 获取当前激活的键盘数据源
       * 通过修改 Referee_Task.h 中的 USE_REMOTER_KEYBOARD 宏切换：
       * - USE_REMOTER_KEYBOARD = 0: 使用裁判系统的键盘数据(kb_info)
       * - USE_REMOTER_KEYBOARD = 1: 使用遥控器DBUS协议的键盘数据(remoter.keyboard)
       */
#if (USE_REMOTER_KEYBOARD != 0)
      const keyboard_t *active_kb = &remoter.keyboard;
#else
      const keyboard_t *active_kb = &kb_info;
#endif

      if (Engineer_Mode.Chassis_Ctrl_Mode == CHASSIS_CTRL_MODE_Normal) {
        /* Normal模式：
         * - 底盘：WASD控制平移，鼠标X轴控制旋转（允许yaw）
         * - 抬升：保持停转状态
         */
        Chassis_Keyboard_Mode(active_kb, 0U);  // disable_yaw = 0，允许旋转
        Rising_Normal_Mode(&remoter);
      } else {
        /* Rising模式：
         * - 底盘：WASD控制平移，禁止旋转（disable_yaw = 1）
         * - 抬升：启动抬升机构，ch2=200提供固定速度
         */
        Chassis_Keyboard_Mode(active_kb, 1U);  // disable_yaw = 1，禁止旋转
        rc_info_t rising_rc = remoter;
        rising_rc.ch2 = 200;  // 抬升固定速度
        Rising_Upstairs_Mode(&rising_rc);
      }
    }
    osDelay(2);
  }
  /* USER CODE END Chassis_Task */
}
