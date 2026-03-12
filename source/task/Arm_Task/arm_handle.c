#include "arm_handle.h"
#include "joint_control_drv.h"

static const float Zero_Velocity[6] = {0, 0, 0, 0, 0, 0};
static const float Default_Velocity[6] = {
    JOINT_DEFAULT_VELOCITY, JOINT_DEFAULT_VELOCITY, JOINT_DEFAULT_VELOCITY,
    JOINT_DEFAULT_VELOCITY, JOINT_DEFAULT_VELOCITY, JOINT_DEFAULT_VELOCITY,
};
/**
 * @brief 关节角度解算
 *
 * @param CtrllerData 控制器数据
 * @param joint_radian 弧度数组
 */
void Parse_ControllerData_To_CtrllerRadian(const uint8_t *CtrllerData,
                                           float *joint_radian) {
  for (int i = 0; i < 6; i++) {
    int tmp = 0;
    // 每个关节弧度占 4 个字符
    sscanf((const char *)&CtrllerData[i * 4], "%04d", &tmp);
    joint_radian[i] = tmp / 1000.0f;
    if (i==2) {
      joint_radian[i] -= PI + 0.2f; // 偏移 PI
    }
    else {
      joint_radian[i] -= PI; // 偏移 PI
    }
  }
  switch (CtrllerData[25] - '0') {
  case 0:
    Gripper_Current_Control_Mode = GRIPPER_OPEN_MODE;
    break;
  case 1:
    Gripper_Current_Control_Mode = GRIPPER_CLOSE_MODE;
    break;
  }
}

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

void Arm_Custom_Controller_Follow_Handle(void) {
  Parse_ControllerData_To_CtrllerRadian(CtrllerData, Ctrller_Joint_Radian);

  CtrllerData_To_InputRadian_Converter(Ctrller_Joint_Radian);

  memcpy(Target_Joint_Radian, Ctrller_Joint_Radian,
         sizeof(Ctrller_Joint_Radian));

  Point_Publisher(Target_Point, Target_Joint_Radian, Default_Velocity);
}
void Arm_Frozen_Handle(void) {
  Point_Publisher(Target_Point, Target_Joint_Radian, Zero_Velocity);
}
