#include "DBusSys.h"
#include "arm_state_machine.h"
#include "trajectory_publisher_drv.h"
#include <stdint.h>
#include "cmsis_os2.h"

extern osTimerId_t traj_timer_id;
extern rc_info_t remoter;
static Command_Place_And_Get_t get_command_table[] ={
  Command_getRight,
  Command_getLeft
};
static Command_Place_And_Get_t place_command_table[] = {
  Command_placeLeft,
  Command_placeRight,
};
#define COMMAND_TABLE_SIZE 2
static int place_key_command_index = 0;
static int get_key_command_index = 0;

void Arm_Keyboard_Manager(uint8_t key) {

  if (key == (uint8_t)'Q') {
    Arm_Current_Control_Mode = Arm_Traj_Mode;
      get_key_command_index = (get_key_command_index + 1) % COMMAND_TABLE_SIZE;
      cmd_place_get = get_command_table[get_key_command_index];
      traj_point_index = 0;
      osTimerStop(traj_timer_id);
      osTimerStart(traj_timer_id, 5);
  }
  if (key == (uint8_t)'E' ) {
    Arm_Current_Control_Mode = Arm_Traj_Mode;
    place_key_command_index = (place_key_command_index + 1) % COMMAND_TABLE_SIZE;
    cmd_place_get = place_command_table[place_key_command_index];
    traj_point_index = 0;
    osTimerStop(traj_timer_id);
    osTimerStart(traj_timer_id, 5); 
    }
  if (key == (uint8_t)'F') {
    Arm_Current_Control_Mode = Arm_Custom_Controller_Follow_Mode;
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
