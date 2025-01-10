#include "main.h"
#include "cmsis_os2.h"
#include "DBusSys.h"
#include "motor_DM.h"
#include "motor_DJI.h"
#include "PIDtool.h"
#include "omni_mecanum_kinematics.h"
void Remoter_Task(void const *argument)
{
    UNUSED(argument);
    osDelay(100);
    DBus_Init();

    while (1)
    {
        DBus_Refresh();
        osDelay(1);
    }
}
