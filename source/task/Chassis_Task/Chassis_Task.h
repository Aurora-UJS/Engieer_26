#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "usart.h"
#include "tim.h"
#include "IMUtool.h"
#include "cmsis_os2.h"
#include "omni_mecanum_kinematics.h"
#include "motion_state.h"
#include <stdint.h>
#include "DbusSys.h"
#include "motor_DJI.h"
#include "motor_DM.h"
#include "PIDtool.h"

#define Max_Velocity 2
#define Max_Rising_Motor_Velocity 4

#define Track_R 0.05
#define Steel_R 0.15
#define Reduction_Ratio 3519/187
#define Motor_Wheel_Trans 6.677 * 0.0001
#define Turning_Forward_Feedback -1

#define Chassis_PowerOff 0
#define Chassis_Normal 1
#define Chassis_Upstairs 2

#define Chassis_Motor_3508_ZQ  0   
#define Chassis_Motor_3508_ZH  1           
#define Chassis_Motor_3508_YH  2
#define Chassis_Motor_3508_YQ  3 

#define Rising_Motor_3508_Left  0
#define Rising_Motor_3508_Right  1 

#define Chassis_Motor_3508_ZQ_id  0x201   
#define Chassis_Motor_3508_ZH_id  0x202           
#define Chassis_Motor_3508_YH_id  0x203
#define Chassis_Motor_3508_YQ_id  0x204 

#define Rising_Motor_3508_Left_id  0x205
#define Rising_Motor_3508_Right_id   0x206     

#define Chassis_Motor_ALL_id 0x200
#define Rising_Motor_ALL_id 0x1FF

#define Chassis_3508_PID_kp 7000
#define Chassis_3508_PID_ki 0.0000f
#define Chassis_3508_PID_kd 0
#define Chassis_3508_PID_Maxout 16384
#define Chassis_3508_PID_Maxiout 8192

#define Rising_3508_PID_kp 7000
#define Rising_3508_PID_ki 0
#define Rising_3508_PID_kd 0
#define Rising_3508_PID_Maxout 16384
#define Rising_3508_PID_Maxiout 8192

#define DM_MITMode_ID 0x000
#define DM_PosVelMode_ID 0x100
#define DM_VelMode_ID 0x200
#define DM_EMITMode_ID 0x300

#define DM_l0010l_Master_ID_Left 0x11
#define DM_l0010l_Master_ID_Right 0x12
#define DM_l0010l_CAN_ID_Left 0x01
#define DM_l0010l_CAN_ID_Right 0x02

void Chassis_Task(void *argument);

void Motor_Init_DJI(DJI_motor_t **Chassis_Motor,DJI_motor_t **Rising_Motor);
uint8_t Chassis_Mode_Get(rc_info_t *backdata);

void Chassis_Motor_SendControl_DJI(DJI_motor_t *DJMotor,int16_t output[]);
void Chassis_Motor_TargetVelocity(float32_t Target_Velocity[],rc_info_t remoter);
void Chassis_3508_PID_Init(pid_type_def pid[]);
void Chassis_3508_PID_Calculate(pid_type_def pid[],float32_t target_speed[],DJI_motor_t *motor,int16_t output[]);

void Rising_Motor_SendControl_DJI(DJI_motor_t *DJMotor,int16_t output[]);
void Rising_Motor_TargetVelocity(float32_t Target_Velocity[],rc_info_t remoter);
void Rising_3508_PID_Init(pid_type_def pid[]);
void Rising_3508_PID_Calculate(pid_type_def pid[],float32_t target_speed[],DJI_motor_t *motor,int16_t output[]);
void Motor_Init_DM(DM_motor_t **Rising_Motor_L,DM_motor_t **Rising_Motor_R);

#endif // !CHASSIS_TASK_H
