#include "main.h"
#include "cmsis_os2.h"
#include "DBusSys.h"

void Remoter_Task(void const *argument)
{
    UNUSED(argument);
    osDelay(1000);
    DBusSys_Init();
    while (1)
    {
        DBus_Refresh();
        osDelay(10);
    }
}
