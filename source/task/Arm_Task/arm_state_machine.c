#include "arm_state_machine.h"
#include "cmsis_os2.h"
#include "joint_control_drv.h"
#include "motor_DM.h"

float Rising_Joint_Radian[6] = {0,1.6,1.5,0,0,0};
extern osThreadId_t Trajectory_PublisherHandle;
const float Zero_Velcoity[6] = {0, 0, 0, 0, 0, 0};
const float Default_Velcoity[6] = {
    JOINT_DEFAULT_VELOCITY, JOINT_DEFAULT_VELOCITY, JOINT_DEFAULT_VELOCITY,
    JOINT_DEFAULT_VELOCITY, JOINT_DEFAULT_VELOCITY, JOINT_DEFAULT_VELOCITY,
};

gripper_control_mode_t Gripper_Current_Control_Mode = GRIPPER_IDLE_MODE;
arm_control_mode_t Arm_Current_Control_Mode = Arm_IDLE_Mode;

target_point_t traj_test[256];

void Arm_Traj_Handle(void) {

  static uint8_t traj_started = 0;
  if (0 == traj_started) {
    osThreadFlagsSet(Trajectory_PublisherHandle, TRAJ_START_FLAG);
    traj_started = 1;
  }
}
void Arm_Transition_Handle(Joint_t *Joint, const float *transition_radian) {
  if (true == Arm_At_Target(Joint, transition_radian)) {
    // Arm_Current_Control_Mode = Arm_IDLE_Mode;
  } else {
  }
}
/**
 * @brief 夹爪状态机
 *
 * @param EndEffector
 */
void Gripper_Control_Mode_Mangner(endEffector_t *EndEffector) {
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
void Joint_Control_Mode_Mangner(Joint_t *Joint) {
    switch(remoter.sw1){
      case 1:
        Arm_Current_Control_Mode =   Arm_Rising_Mode;
      break;
      default:
       Arm_Current_Control_Mode = Arm_Custom_Controller_Follow_Mode;
      break;
    }
  switch (Arm_Current_Control_Mode) {
  case Arm_Rising_Mode:
        Point_Publisher(Target_Point, Rising_Joint_Radian, Default_Velcoity);
    break;
  case Arm_IDLE_Mode:

    Arm_Current_Control_Mode = Arm_Custom_Controller_Follow_Mode;
    break;

  case Arm_Transition_Mode:

    Arm_Transition_Handle(Joint, Mannal_Joint_Radian);
    break;
  case Arm_Custom_Controller_Follow_Mode:

    Parse_ControllerData_To_CtrllerRadian(CtrllerData, Ctrller_Joint_Radian);

    CtrllerData_To_InputRadian_Converter(Ctrller_Joint_Radian);

    memcpy(Target_Joint_Radian, Ctrller_Joint_Radian,
           sizeof(Ctrller_Joint_Radian));

    Point_Publisher(Target_Point, Target_Joint_Radian, Default_Velcoity);

    break;

  case Arm_Frozen_Mode:
    Point_Publisher(Target_Point, Target_Joint_Radian, Zero_Velcoity);
    break;

  case Arm_Set_Radian:
    break;
  case Arm_Traj_Mode:
    Arm_Traj_Handle();
  }
}
