#include "arm_state_machine.h"

extern arm_control_mode_t Arm_Current_Control_Mode;

extern gripper_control_mode_t Gripper_Current_Control_Mode;

extern float Ctrller_Joint_Radian[6];
extern DM_motor_t *Joint_Motor[JOINT_NUM];
extern float Mannal_Joint_Radian[6];
extern target_point_t Target_Point[6];
extern float Target_Joint_Radian[6];
extern const float Default_Velcoity[6];

gripper_control_mode_t Gripper_Current_Control_Mode = GRIPPER_IDLE_MODE;
arm_control_mode_t Arm_Current_Control_Mode = Arm_IDLE_Mode;

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
  switch (Arm_Current_Control_Mode) {
  case Arm_IDLE_Mode:
    Arm_Current_Control_Mode = Arm_Custom_Controller_Follow_Mode;
    break;
  case Arm_TRANSITION_Mode:
    Arm_Transition_Handle(Joint, Mannal_Joint_Radian);
    break;
  case Arm_Custom_Controller_Follow_Mode:

    Parse_ControllerData_To_CtrllerRadian(CtrllerData, Ctrller_Joint_Radian);

    CtrllerData_To_InputRadian_Converter(Ctrller_Joint_Radian);

    memcpy(Target_Joint_Radian, Ctrller_Joint_Radian,
           sizeof(Ctrller_Joint_Radian));

    Point_Generator(Target_Point, Target_Joint_Radian, Default_Velcoity);
    // Joint_Custom_State_Motor_Ctrl(Joint, Ctrller_Joint_Radian);
    break;
  case Arm_Frozen_Mode:
    break;

  case Arm_Set_Radian:
    Joint_Mannal_State_Motor_Ctrl(Joint, Mannal_Joint_Radian);
    break;
  }
}
