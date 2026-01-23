
#include "arm_state_machine_task.h"
#include "ee_control_drv.h"

extern rc_info_t remoter;

// ch1 右摇杆 左右 左-右+
// ch2 右摇杆 前后 前+后-
// ch3 左摇杆 左右 左-右+
// ch4 左摇杆 前后 前+后-
// sw1 左拨码开关 前1 中3 后2
// sw2 右拨码开关 前1 中3 后2

#define MAX_INPUTS 10

uint8_t rc_map_table[MAX_INPUTS];

gripper_control_mode_t Gripper_Current_Control_Mode = GRIPPER_IDLE_MODE;
arm_control_mode_t Arm_Current_Control_Mode = Arm_IDLE_Mode;

void Remoter_Input_To_Control_Mode_Register(rc_input_id_t input,
                                            uint8_t control_mode) {
  if (input < MAX_INPUTS) {
    rc_map_table[input] = control_mode;
  }
}

static inline rc_input_id_t RC_Get_Input_ID(rc_info_t *rc) {

  switch (rc->sw1) {
  case 1:
    return SW1_UP;
  case 2:
    return SW1_DOWN;
  case 3:
    return SW1_MID;
  default:
    break;
  }

  switch (rc->sw2) {
  case 1:
    return SW2_UP;
  case 2:
    return SW2_DOWN;
  case 3:
    return SW2_MID;
  default:
    break;
  }
  return SW_DEFAULT;
}

void Remoter_Dispatcher(rc_info_t *remoter) {
  rc_input_id_t input_id = RC_Get_Input_ID(remoter);
  Arm_Current_Control_Mode = (arm_control_mode_t)rc_map_table[input_id];
}
void remoter_map_init(void) {}

void Arm_State_Machine_Task(void *argument) {
  UNUSED(argument);
  Remoter_Dispatcher(&remoter);

  while (1) {
    osDelay(10);
  }
}
