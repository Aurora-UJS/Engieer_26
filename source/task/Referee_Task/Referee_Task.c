#include <stdio.h>
#include "Referee_Task.h"
#include "referee_api.h"
#include "cmsis_os2.h"
#include "referee_protocol.h"
#include "arm_math.h"
#include <stdint.h>

custom_controller_info_t *Controller_Msg;
Engineer_Mode_t Engineer_Mode;
extern keyboard_t kb_info;

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

void Referee_Task(void *argument)
{
    UNUSED(argument);
    ctrller_init(&huart7);
    keyboard_t kb_info_temp = kb_info;
    for(;;)
    {
        
        Controller_Msg = get_custom_controller_msg();
        kb_info_temp = kb_info;
        Referee_KeyboardEdgeDetect(&kb_info_temp);
        osDelay(2);
    }
}
