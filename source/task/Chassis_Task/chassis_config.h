#ifndef CHASSIS_CONFIG_H
#define CHASSIS_CONFIG_H

#define Max_Velocity 3

#define Max_Rising_Motor_Velocity 3.0f
#define Max_Rising_DM_angle 0.83f
#define Rising_DM_ZeroPoint 0.08f
#define Rising_DM_Velocity 1.5f

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

#define Chassis_3508_PID_kp 5000
#define Chassis_3508_PID_ki 0.0f
#define Chassis_3508_PID_kd 0.0f
#define Chassis_3508_PID_Maxout 16384
#define Chassis_3508_PID_Maxiout 8192

#define Rising_3508_PID_kp 9000
#define Rising_3508_PID_ki 0.0001f
#define Rising_3508_PID_kd 0.0f
#define Rising_3508_PID_Maxout 16384
#define Rising_3508_PID_Maxiout 8192

#define Rising_DM_PID_kp 1.2f
#define Rising_DM_PID_ki 0.004f
#define Rising_DM_PID_kd 0.02f
#define Rising_DM_PID_Maxout 0.85f
#define Rising_DM_PID_Maxiout 0.8f

#define DM_l0010l_Master_ID_Left 0x13
#define DM_l0010l_Master_ID_Right 0x14
#define DM_l0010l_CAN_ID_Left 0x03
#define DM_l0010l_CAN_ID_Right 0x04

#endif /* CHASSIS_CONFIG_H */
