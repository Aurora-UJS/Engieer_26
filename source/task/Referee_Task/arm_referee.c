#include "arm_state_machine.h"
#include "trajectory_publisher_drv.h"

void Arm_Keyboard_Manager(uint8_t key)
{

    if (key == (uint8_t)'Q') 
    {
        Arm_Current_Control_Mode = Arm_Traj_Mode;
        cmd_place_get = Command_getRight; 
    }

    if (key == (uint8_t)'E')
    {
        if (Arm_Current_Control_Mode == Arm_Rising_Mode) {
            Arm_Current_Control_Mode = Arm_Custom_Controller_Follow_Mode;
        } else {
            Arm_Current_Control_Mode = Arm_Rising_Mode;
        }
    }
    
    
}
