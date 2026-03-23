#include "arm_state_machine.h"
#include "arm_handle.h"
#include "ee_control_drv.h"
#include "joint_control_drv.h"
#include "arm_debug.h"
gripper_control_mode_t Gripper_Current_Control_Mode = GRIPPER_IDLE_MODE;

#if TRAJ_DEBUG
arm_control_mode_t Arm_Current_Control_Mode = Arm_Traj_Mode;
#else 
arm_control_mode_t Arm_Current_Control_Mode = Arm_Custom_Controller_Follow_Mode;
#endif


float Rising_Joint_Radian[6] = {0,1.4,1.3,0,0.4,0};

const float Rising_Velcoity[6] = {
  0,0.4,0.5,0,0.4,0
};

static float Zero_Joint_Radian[6] = { 0 ,0, 0 ,0, 0, 0};
static float Zero_Velocity[6] ={
  0,0.4,0.3,0,0.1,0
};

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
  case GRIPPER_SPECI_MODE:
    Gripper_Speci(EndEffector);
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
  case Arm_Rising_Mode:
        Point_Publisher(Target_Point, Rising_Joint_Radian, Rising_Velcoity);
    break;
  case Arm_IDLE_Mode:

    // Arm_Current_Control_Mode = Arm_Custom_Controller_Follow_Mode;

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
    break;
  case Arm_Zero_Mode:
    Point_Publisher(Target_Point, Zero_Joint_Radian, Zero_Velocity);
    break;
  }
}
