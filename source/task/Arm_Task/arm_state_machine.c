#include "arm_state_machine.h"
#include "arm_handle.h"

gripper_control_mode_t Gripper_Current_Control_Mode = GRIPPER_IDLE_MODE;
arm_control_mode_t Arm_Current_Control_Mode = Arm_Traj_Mode;


/**
 * @brief 夹爪状态机
 *
 * @param EndEffector
 */
void Gripper_Control_Mode_Manager(endEffector_t *EndEffector) {
  switch (Gripper_Current_Control_Mode) {
  case GRIPPER_IDLE_MODE:
    Gripper_Current_Control_Mode = GRIPPER_OPEN_MODE;
    break;
  case GRIPPER_OPEN_MODE:
    Gripper_Open(EndEffector);
    break;
  case GRIPPER_CLOSE_MODE:
    Gripper_Close(EndEffector);
    break;
  }
}

/**
 * @brief 关节状态机
 *
 * @param Joint
 */
void Joint_Control_Mode_Manager(Joint_t *Joint) {
  switch (Arm_Current_Control_Mode) {
  case Arm_IDLE_Mode:

    Arm_Current_Control_Mode = Arm_Custom_Controller_Follow_Mode;

    break;

  case Arm_Transition_Mode:

    Arm_Transition_Handle(Joint, Mannal_Joint_Radian);

    break;
  case Arm_Custom_Controller_Follow_Mode:

    Arm_Custom_Controller_Follow_Handle();

    break;

  case Arm_Frozen_Mode:
    Arm_Frozen_Handle();
    break;

  case Arm_Set_Radian:
    
    break;
  case Arm_Traj_Mode:
    Arm_Traj_Handle();
  }
}
