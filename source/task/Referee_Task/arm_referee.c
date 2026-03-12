#include "DBusSys.h"
#include "arm_state_machine.h"
#include "trajectory_publisher_drv.h"
#include <stdint.h>
#include "cmsis_os2.h"

extern osTimerId_t traj_timer_id;
extern rc_info_t remoter;
static Command_Place_And_Get_t place_and_get_command_table[] ={
  Command_getRight,
  Command_getLeft,
  Command_getBackLeft,
  Command_getBackRight,
  Command_placeBackRight,
  Command_placeBackLeft,
  Command_placeRight,
  Command_placeLeft,
};
#define COMMAND_TABLE_SIZE 8
static int keyboard_command_index = -1;

void Arm_Keyboard_Manager(uint8_t key) {

  if (key == (uint8_t)'Q') {
    Arm_Current_Control_Mode = Arm_Custom_Controller_Follow_Mode;
  }
  if (key == (uint8_t)'E' ) {
    Arm_Current_Control_Mode = Arm_Traj_Mode;
    keyboard_command_index++;
        if (keyboard_command_index >= COMMAND_TABLE_SIZE || keyboard_command_index <0)
            keyboard_command_index = 0;
        cmd_place_get = place_and_get_command_table[keyboard_command_index];
    traj_point_index = 0;
    osTimerStart(traj_timer_id, 5); // <-- 确保定时器启动
    }
  if (key == (uint8_t)'R') {
    Arm_Current_Control_Mode = Arm_IDLE_Mode;
  }

   
/*   if (key == (uint8_t)'E') {
    Arm_Current_Control_Mode = Arm_Traj_Mode;
    cmd_place_get = Command_getRight;
  }

  if (key == (uint8_t)'R') {
    Arm_Current_Control_Mode = Arm_Traj_Mode;
    cmd_place_get = Command_placeLeft;
  }

  if (key == (uint8_t)'T') {
    Arm_Current_Control_Mode = Arm_Traj_Mode;
    cmd_place_get = Command_placeRight;
  } */
}
