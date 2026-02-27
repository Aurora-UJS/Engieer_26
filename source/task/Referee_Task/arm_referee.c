#include "arm_state_machine.h"
#include "trajectory_publisher_drv.h"

void Arm_Keyboard_Manager(uint8_t key){

    if (key == (uint8_t)'Q') {
        Arm_Current_Control_Mode = Arm_Traj_Mode;
        cmd_place_get = Command_getRight; 
    }
    
    
}
