#include "arm_handle.h"

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
    joint_radian[i] -= PI; // 偏移 PI
  }
  switch (CtrllerData[25]-'0') {
    case 0: Gripper_Current_Control_Mode = GRIPPER_OPEN_MODE;
            break;
    case 1: Gripper_Current_Control_Mode = GRIPPER_CLOSE_MODE;
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
