#include "main.h"
#include "cmsis_os2.h"
#include "DBusSys.h"
#include "motor_DM.h"

DM_motor_t *DM_motor_8009_Down, *DM_motor_8009_Up;
void Remoter_Task(void const *argument)
{
    UNUSED(argument);
    osDelay(1000);
    DBus_Init();
    while (1)
    {
        DBus_Refresh();

        osDelay(10);
    }
}
