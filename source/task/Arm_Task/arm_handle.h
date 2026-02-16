#ifndef ARM_HANDLE_H
#define ARM_HANDLE_H

#include "trajectory_publisher_drv.h"

extern osThreadId_t Trajectory_PublisherHandle;
void Arm_Traj_Handle(void);
void Arm_Transition_Handle(Joint_t*, const float*);
#endif
