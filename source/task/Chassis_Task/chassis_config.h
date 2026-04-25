#ifndef CHASSIS_CONFIG_H
#define CHASSIS_CONFIG_H

#define Max_Velocity 3
#define Chassis_Keyboard_Shift_Speed_Ratio 0.3f
#define Chassis_Keyboard_MouseYaw_Deadzone 500
#define Chassis_Keyboard_MouseYaw_Sensitivity 0.5f
#define Chassis_Keyboard_MouseYaw_Polarity -1.0f

#define Max_Rising_Motor_Velocity 1.8f
#define Max_Rising_DM_angle 0.9f
#define Rising_DM_ZeroPoint 0.05f
#define Rising_DM_Velocity 1.8f
#define Rising_DM_ImuTarget_SwitchDeltaThreshold 0.2f
#define Rising_DM_ImuTarget_Fallback 0.18f

#define Track_R 0.05
#define Steel_R 0.15
#define Reduction_Ratio 3519/187
#define Motor_Wheel_Trans 6.677 * 0.0001
#define Turning_Forward_Feedback -1

#define CHASSIS_RISING_BEHAVIOR_DEFAULT 0U
#define CHASSIS_RISING_KEYBOARD_RC_CH2 200

#define CHASSIS_RISING_SINGLE_LIFT_DURATION_MS 2000U
#define CHASSIS_RISING_SINGLE_LIFT_CHASSIS_SPEED_RATIO_NUM 70
#define CHASSIS_RISING_SINGLE_LIFT_CHASSIS_SPEED_RATIO_DEN 100
#define CHASSIS_RISING_SINGLE_LIFT_RISING_RC_CH2 Remoter_CHMAX
#define CHASSIS_RISING_SINGLE_TRANSITION_DURATION_MS 0U
#define CHASSIS_RISING_SINGLE_DRIVE_DURATION_MS 800U
#define CHASSIS_RISING_SINGLE_DRIVE_SPEED_RATIO_NUM 100
#define CHASSIS_RISING_SINGLE_DRIVE_SPEED_RATIO_DEN 100

#define CHASSIS_RISING_DOUBLE_LIFT_DURATION_MS 1600U
#define CHASSIS_RISING_DOUBLE_LIFT_CHASSIS_SPEED_RATIO_NUM 100
#define CHASSIS_RISING_DOUBLE_LIFT_CHASSIS_SPEED_RATIO_DEN 100
#define CHASSIS_RISING_DOUBLE_LIFT_RISING_RC_CH2 Remoter_CHMAX
#define CHASSIS_RISING_DOUBLE_TRANSITION_DURATION_MS 0U
#define CHASSIS_RISING_DOUBLE_DRIVE_DURATION_MS 600U
#define CHASSIS_RISING_DOUBLE_DRIVE_SPEED_RATIO_NUM 70
#define CHASSIS_RISING_DOUBLE_DRIVE_SPEED_RATIO_DEN 100
#define CHASSIS_RISING_DOUBLE_RISING_HOLD_DURATION_MS 1800U
#define CHASSIS_RISING_DOUBLE_NORMAL_HOLD_DURATION_MS 600U

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

#define Chassis_PowerLimit_Enable_Default 1U
#define Chassis_PowerLimit_UserMax_Default 110.0f
#define Chassis_PowerModel_TorqueCoeff_Default 1.99688994e-6f
#define Chassis_PowerModel_K1_Default 1.23e-07f
#define Chassis_PowerModel_K2_Default 1.453e-07f
#define Chassis_PowerModel_K3_Default 4.081f
#define Chassis_PowerModel_GlobalScale_Default 1.6f
#define Chassis_PowerScale_Attack_Default 1.0f
#define Chassis_PowerScale_Release_Default 0.1f

#define Chassis_PowerCalc_Group_Chassis 0U
#define Chassis_PowerCalc_Group_Rising 1U
#define Chassis_PowerCalc_Group_Count 2U
#define Chassis_PowerCalc_Enable_Default 1U

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
