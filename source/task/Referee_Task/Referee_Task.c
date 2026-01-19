#include <stdio.h>
#include "Referee_Task.h"
#include "referee_api.h"
#include "cmsis_os2.h"
#include "referee_protocol.h"
#include "arm_math.h"
#include <stdint.h>

custom_controller_info_t *Controller_Msg;

extern uint8_t CtrllerData[24];
void Referee_Task(void *argument)
{
    UNUSED(argument);
    ctrller_init(&huart7);
    for(;;)
    {
        Controller_Msg = get_custom_controller_msg();
        osDelay(2);
    }
}
