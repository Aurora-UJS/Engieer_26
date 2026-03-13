#include "arm_handle.h"
#include "joint_control_drv.h"
#include <string.h>

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
static uint8_t last_gripper_cmd = 0;
float j6_debug = 0;
float j6_direct_debug = 0;
void Parse_ControllerData_To_CtrllerRadian(const uint8_t *CtrllerData,
                                           float *joint_radian) {
  float j6_direct = (CtrllerData[26]-'0' == 0)?(1):(-1);
  j6_direct_debug =j6_direct; 
  for (int i = 0; i < 6; i++) {
    int tmp = 0;
    float temp_joint_radian= 0;
    // 每个关节弧度占 4 个字符
    tmp =
    (CtrllerData[i*4]   - '0') * 1000 +
    (CtrllerData[i*4+1] - '0') * 100  +
    (CtrllerData[i*4+2] - '0') * 10   +
    (CtrllerData[i*4+3] - '0');
    // joint_radian[i] = tmp / 1000.0f;
    temp_joint_radian = tmp * 0.001f;
    
    if (i==2) {
      temp_joint_radian -= PI + 0.2f;
      // joint_radian[i] -= PI + 0.2f; // 偏移 PI
    }
    else {
      temp_joint_radian -= PI;
      // joint_radian[i] -= PI; // 偏移 PI
    }
    if (i == 5)
    {
      j6_debug = temp_joint_radian;
      temp_joint_radian *= j6_direct; // j6暂时通过数据解包来反向处理
    }
    joint_radian[i] = temp_joint_radian;
  }

  uint8_t current = CtrllerData[25] - '0';

    if (current != last_gripper_cmd)
    {
        endEffector_Toggle();
    }
  last_gripper_cmd = current;

  
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
float test_parse_radian[6] = {0}; 
void Arm_Custom_Controller_Follow_Handle(void) {

  Parse_ControllerData_To_CtrllerRadian(CtrllerData, Ctrller_Joint_Radian);
  memcpy(test_parse_radian, Ctrller_Joint_Radian, 6);
  CtrllerData_To_InputRadian_Converter(Ctrller_Joint_Radian);

  memcpy(Target_Joint_Radian, Ctrller_Joint_Radian,
         sizeof(Ctrller_Joint_Radian));

  Point_Publisher(Target_Point, Target_Joint_Radian, Default_Velocity);
}
void Arm_Frozen_Handle(void) {
  Point_Publisher(Target_Point, Target_Joint_Radian, Zero_Velocity);
}
