#include "trajectory_publisher_drv.h"

/**
 * @brief 右轨迹发布函数
 *
 * @param Trajectory
 * @param point_index
 */
void Trajectory_Publisher_right(const float Trajectory[][JOINT_NUM],
                                int point_index) {
  Target_Point[0].velocity = Traj_Vel[point_index][0];
  Target_Point[0].target_joint_radian = (-1.0f) * Trajectory[point_index][0];
  for (int joint_index = 1; joint_index < JOINT_NUM; joint_index++) {
    if (point_index >= 1200) {
      Target_Point[2].target_joint_radian = Trajectory[point_index][2] + 0.35f;
      Target_Point[4].target_joint_radian = Trajectory[point_index][4] + 0.35f;
      Target_Point[1].target_joint_radian = Trajectory[point_index][1];
      Target_Point[3].target_joint_radian = Trajectory[point_index][3];
      Target_Point[5].target_joint_radian = Trajectory[point_index][5];
    } else {
      Target_Point[joint_index].target_joint_radian =
          Trajectory[point_index][joint_index];
    }
    Target_Point[joint_index].velocity =
        Traj_Vel[point_index][joint_index] * (-2.0f);
  }
}

/**
 * @brief 左轨迹发布函数
 *
 * @param Trajectory
 * @param point_index
 */
void Trajectory_Publisher(const float Trajectory[][JOINT_NUM],
                          int point_index) {
  for (int joint_index = 0; joint_index < JOINT_NUM; joint_index++) {
    if (point_index >= 1250) {
      Target_Point[2].target_joint_radian = Trajectory[point_index][2] + 0.35f;
      Target_Point[4].target_joint_radian = Trajectory[point_index][4] + 0.35f;
      Target_Point[0].target_joint_radian = Trajectory[point_index][0];
      Target_Point[1].target_joint_radian = Trajectory[point_index][1];
      Target_Point[3].target_joint_radian = Trajectory[point_index][3];
      Target_Point[5].target_joint_radian = Trajectory[point_index][5];
    } else {
      Target_Point[joint_index].target_joint_radian =
          Trajectory[point_index][joint_index];
    }
    Target_Point[joint_index].velocity =
        Traj_Vel[point_index][joint_index] * 2.0f;
  }
}

/**
 * @brief 取左矿函数
 *
 */
void getLeft(void) {
  if (traj_point_index <= ARM_PREPARE_TIME_TICKS) {
    Target_Point[0].target_joint_radian = -0.79f;
    Target_Point[0].velocity = 0.5f;
    Target_Point[4].target_joint_radian = 0.0f;
    Target_Point[4].velocity = 0.5f;
  } else if (traj_point_index >= (ARM_PREPARE_TIME_TICKS + TRAJECTORY_NUM)) {
    target_point_init(Target_Point);
  } else {
    Trajectory_Publisher(Trajectory, traj_point_index - ARM_PREPARE_TIME_TICKS);
  }
  if (traj_point_index == ENDEFFECTOR_CLOSE_TIME_TICKS) {
    Gripper_Current_Control_Mode = GRIPPER_CLOSE_MODE;
  }
  traj_point_index++;
}

/**
 * @brief 取右矿函数
 *
 */
void getRight(void) {
  if (traj_point_index <= ARM_PREPARE_TIME_TICKS) {
    Target_Point[0].target_joint_radian = 0.76f;
    Target_Point[0].velocity = 0.5f;
    Target_Point[4].target_joint_radian = 0.0f;
    Target_Point[4].velocity = 0.5f;
  } else if (traj_point_index >=
             (ARM_PREPARE_TIME_TICKS + ARM_PICK_TIME_TICKS)) {
    target_point_init(Target_Point);
  } else {
    Trajectory_Publisher_right(Trajectory,
                               traj_point_index - ARM_PREPARE_TIME_TICKS);
  }
  if (traj_point_index == ENDEFFECTOR_CLOSE_TIME_TICKS) {
    Gripper_Current_Control_Mode = GRIPPER_CLOSE_MODE;
  }
  traj_point_index++;
}


/**
 * @brief 左放置函数
 *
 */
void placeLeft(void) {
  if (traj_point_index <= 2000) {
    Gripper_Current_Control_Mode = GRIPPER_CLOSE_MODE;
  } else {
    Gripper_Current_Control_Mode = GRIPPER_OPEN_MODE;
  }
  if (traj_point_index < 500) {
    Target_Point[1].target_joint_radian = 0.49139;
    Target_Point[2].target_joint_radian = 0.46;
    Target_Point[3].target_joint_radian = 0.0;
    Target_Point[4].target_joint_radian = 0.2;
    Target_Point[5].target_joint_radian = 0.00;
  }
  if (traj_point_index >= 500 && traj_point_index <= 1500) {
    Target_Point[0].target_joint_radian = -0.79;
    Target_Point[0].velocity = 1.5;
  } else if (traj_point_index > 1500 && traj_point_index < 2300) {
    Target_Point[2].velocity = 0.5;
    Target_Point[2].target_joint_radian = 0.26;
  } else if (traj_point_index >= 2300 && traj_point_index <= 2800) {
    Target_Point[1].velocity = 0.2;
    Target_Point[1].target_joint_radian = 0;

    Target_Point[2].target_joint_radian = -0.05;
    Target_Point[2].velocity = 0.6;
  } else if (traj_point_index > 2800) {
    Target_Point[0].target_joint_radian = 0;
    Target_Point[0].velocity = 1.5;
  }
  traj_point_index++;
}

/**
 * @brief 右放置函数
 *
 */
void placeRight(void) {
  if (traj_point_index <= 2000) {
    Gripper_Current_Control_Mode = GRIPPER_CLOSE_MODE;
  } else {
    Gripper_Current_Control_Mode = GRIPPER_OPEN_MODE;
  }
  if (traj_point_index >= 500 && traj_point_index <= 1500) {
    Target_Point[0].target_joint_radian = 0.79;
    Target_Point[1].target_joint_radian = 0.49139;
    Target_Point[2].target_joint_radian = 0.46;
    Target_Point[3].target_joint_radian = 0.0;
    Target_Point[4].target_joint_radian = 0.2;
    Target_Point[5].target_joint_radian = 0.00;
    Target_Point[0].velocity = 0.2;
    for (int joint_index = 1; joint_index < JOINT_NUM; joint_index++) {
      Target_Point[joint_index].velocity = JOINT_DEFAULT_VELOCITY;
    }
  } else if (traj_point_index > 1500 && traj_point_index < 2300) {
    Target_Point[2].velocity = 0.2;
    Target_Point[2].target_joint_radian = 0.2;
  } else if (traj_point_index >= 2300 && traj_point_index <= 2800) {

    Target_Point[1].velocity = 0.2;
    Target_Point[1].target_joint_radian = 0;

    Target_Point[2].target_joint_radian = -0.05;
    Target_Point[2].velocity = 0.6;
  } else if (traj_point_index > 2800) {
    Target_Point[0].target_joint_radian = 0;
    Target_Point[0].velocity = 0.3;
  }
  traj_point_index++;
}
