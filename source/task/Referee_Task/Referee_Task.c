#include <stdio.h>
#include "Referee_Task.h"
#include "referee_api.h"
#include "cmsis_os2.h"
#include "referee_protocol.h"
#include "arm_math.h"
#include "DBusSys.h"
#include <stdint.h>
#include "arm_referee.h"

custom_controller_info_t *Controller_Msg;

/**
 * @brief 工程机器人模式全局变量
 * @note 初始化为DBUS模式，Normal底盘控制，CustomCtrl机械臂控制
 *       可通过按键B切换Ctrl_Logic_Mode（DBUS ↔ Keyboard）
 */
Engineer_Mode_t Engineer_Mode = {
    .Chassis_Ctrl_Mode = CHASSIS_CTRL_MODE_Normal,  // 底盘Normal模式
    .Arm_Ctrl_Mode = ARM_CTRL_MODE_CustomCtrl,      // 机械臂自定义控制
    .Ctrl_Logic_Mode = CTRL_LOGIC_MODE_DBUS         // 默认DBUS模式
};

extern rc_info_t remoter;  // 遥控器数据，定义在 DBusSys.c
extern keyboard_t kb_info;  // 裁判系统键盘数据


/**
 * @brief 获取当前激活的键盘数据指针
 * @return 指向当前激活键盘数据的指针 (根据 USE_REMOTER_KEYBOARD 宏自动选择)
 * @note 该函数根据 Referee_Task.h 中的 USE_REMOTER_KEYBOARD 宏返回对应的键盘数据
 *       - USE_REMOTER_KEYBOARD = 0: 返回裁判系统的 kb_info
 *       - USE_REMOTER_KEYBOARD = 1: 返回遥控器的 remoter.keyboard
 */
const keyboard_t* Referee_GetActiveKeyboard(void)
{
#if (USE_REMOTER_KEYBOARD != 0)
    return &remoter.keyboard;
#else
    return &kb_info;
#endif
}

void Referee_OnKeyboardKeyPressed(uint8_t key)
{
    if (key == (uint8_t)'B') 
    {
        //这里写你要执行的操作（B 从 0->1 的瞬间触发）
        Engineer_Mode.Ctrl_Logic_Mode = (Engineer_Mode.Ctrl_Logic_Mode == CTRL_LOGIC_MODE_DBUS) ? CTRL_LOGIC_MODE_Keyboard : CTRL_LOGIC_MODE_DBUS;
    }
    if (key == (uint8_t)'C') 
    {
        //这里写你要执行的操作（C 从 0->1 的瞬间触发）
        Engineer_Mode.Chassis_Ctrl_Mode = (Engineer_Mode.Chassis_Ctrl_Mode == CHASSIS_CTRL_MODE_Normal) ? CHASSIS_CTRL_MODE_Rising : CHASSIS_CTRL_MODE_Normal;
    }

    Arm_Keyboard_Manager(key, Referee_GetActiveKeyboard());

}

void Referee_KeyboardEdgeDetect(const keyboard_t *kb)
{
    static keyboard_t last_kb;
    static uint8_t inited = 0;

    if (kb == NULL) {
        return;
    }

    if (inited == 0U) {
        last_kb = *kb;
        inited = 1U;
        return;
    }

    if ((kb->key_code.bit.Q != 0U) && (last_kb.key_code.bit.Q == 0U)) { Referee_OnKeyboardKeyPressed((uint8_t)'Q'); }
    if ((kb->key_code.bit.E != 0U) && (last_kb.key_code.bit.E == 0U)) { Referee_OnKeyboardKeyPressed((uint8_t)'E'); }
    if ((kb->key_code.bit.R != 0U) && (last_kb.key_code.bit.R == 0U)) { Referee_OnKeyboardKeyPressed((uint8_t)'R'); }
    if ((kb->key_code.bit.F != 0U) && (last_kb.key_code.bit.F == 0U)) { Referee_OnKeyboardKeyPressed((uint8_t)'F'); }
    if ((kb->key_code.bit.G != 0U) && (last_kb.key_code.bit.G == 0U)) { Referee_OnKeyboardKeyPressed((uint8_t)'G'); }
    if ((kb->key_code.bit.Z != 0U) && (last_kb.key_code.bit.Z == 0U)) { Referee_OnKeyboardKeyPressed((uint8_t)'Z'); }
    if ((kb->key_code.bit.X != 0U) && (last_kb.key_code.bit.X == 0U)) { Referee_OnKeyboardKeyPressed((uint8_t)'X'); }
    if ((kb->key_code.bit.C != 0U) && (last_kb.key_code.bit.C == 0U)) { Referee_OnKeyboardKeyPressed((uint8_t)'C'); }
    if ((kb->key_code.bit.V != 0U) && (last_kb.key_code.bit.V == 0U)) { Referee_OnKeyboardKeyPressed((uint8_t)'V'); }
    if ((kb->key_code.bit.B != 0U) && (last_kb.key_code.bit.B == 0U)) { Referee_OnKeyboardKeyPressed((uint8_t)'B'); }

    last_kb = *kb;
}

/**
 * @brief 裁判系统任务
 * @note 执行键盘边沿检测，键盘数据源由 USE_REMOTER_KEYBOARD 宏控制
 */
void Referee_Task(void *argument)
{
    UNUSED(argument);
    ctrller_init(&huart7);
    
    for(;;)
    {
        Controller_Msg = get_custom_controller_msg();
        
        // 获取键盘数据并进行边沿检测
        Referee_KeyboardEdgeDetect(Referee_GetActiveKeyboard());
        
        osDelay(2);
    }
}
