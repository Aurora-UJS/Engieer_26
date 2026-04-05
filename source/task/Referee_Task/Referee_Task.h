#ifndef REFEREE_TASK_H
#define REFEREE_TASK_H

#include "cmsis_os2.h"
#include "referee_protocol.h"
#include "arm_math.h"
#include "keyBoard.h"
#include <stdint.h>

#define CHASSIS_CTRL_MODE_Normal 0
#define CHASSIS_CTRL_MODE_Rising 1
#define ARM_CTRL_MODE_CustomCtrl 0
#define ARM_CTRL_MODE_AutoCtrl 1
#define ARM_CTRL_MODE_Rising 2
#define CTRL_LOGIC_MODE_DBUS 0
#define CTRL_LOGIC_MODE_Keyboard 1

/**
 * @brief 键盘数据源选择宏定义
 * @note 修改此宏来切换键盘数据源：
 *       0 = 使用裁判系统的键盘数据 (kb_info) - 当前使用
 *       1 = 使用遥控器DBUS的键盘数据 (remoter.keyboard)
 */
#define USE_REMOTER_KEYBOARD 0

/**
 * @brief 键盘数据源枚举
 * @note 用于选择使用裁判系统的键盘数据还是遥控器(DBUS)的键盘数据
 */
typedef enum
{
    KEYBOARD_SOURCE_REFEREE = 0,  // 使用裁判系统的键盘数据
    KEYBOARD_SOURCE_REMOTER = 1   // 使用遥控器(DBUS)的键盘数据
} Keyboard_Source_t;

void Referee_Task(void *argument);

void Referee_KeyboardEdgeDetect(const keyboard_t *kb);

void Referee_OnKeyboardKeyPressed(uint8_t key);

/**
 * @brief 获取当前激活的键盘数据指针
 * @return 指向当前激活键盘数据的指针 (根据 USE_REMOTER_KEYBOARD 宏自动选择)
 * @note 该函数根据 USE_REMOTER_KEYBOARD 宏返回对应的键盘数据
 *       - USE_REMOTER_KEYBOARD = 0: 返回裁判系统的 kb_info
 *       - USE_REMOTER_KEYBOARD = 1: 返回遥控器的 remoter.keyboard
 */
const keyboard_t* Referee_GetActiveKeyboard(void);
typedef struct
{
    uint8_t Chassis_Ctrl_Mode;
    uint8_t Arm_Ctrl_Mode;
    uint8_t Ctrl_Logic_Mode;
} Engineer_Mode_t;

extern Engineer_Mode_t Engineer_Mode;

#endif
