#ifndef ARM_HANDLE_H
#define ARM_HANDLE_H

#include "trajectory_publisher_drv.h"

extern osThreadId_t Trajectory_PublisherHandle;
extern target_point_t Target_Point[6];
extern float Target_Joint_Radian[6];

void Arm_Traj_Handle(void);
void Arm_Transition_Handle(Joint_t*, const float*);
void Arm_Frozen_Handle(void);
void Arm_Custom_Controller_Follow_Handle(void);
#endif
