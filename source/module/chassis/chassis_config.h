#ifndef __CHASSIS_CONFIG_H__
#define __CHASSIS_CONFIG_H__


#define MOTOR1   0
#define MOTOR2   1
#define MOTOR3   2
#define MOTOR4   3



//底盘设置旋转速度，设置前后左右轮不同设定速度的比例分权 0为在几何中心，不需要补偿
#define CHASSIS_WZ_SET_SCALE 0.1f

#define MOTOR_DISTANCE_TO_CENTER 1.2f 

#define Chassis_Max_Speed  10000		//最大速度
#define Chassis_Shift_Max_Speed 660	//                                                                                          
#define Chassis_RC_Max_Speed   660		//
#define Multiple               1  //目标值乘的倍数

#define Chassis1_Motor_Speed_Kp 12
#define Chassis1_Motor_Speed_Ki 0
#define Chassis1_Motor_Speed_Kd 0
#define Chassis1_Motor_Speed_Maxout 5000
#define Chassis1_Motor_Speed_IMaxout 600

#define Chassis2_Motor_Speed_Kp 12
#define Chassis2_Motor_Speed_Ki 0
#define Chassis2_Motor_Speed_Kd 0
#define Chassis2_Motor_Speed_Maxout 5000
#define Chassis2_Motor_Speed_IMaxout 600

#define Chassis3_Motor_Speed_Kp 12
#define Chassis3_Motor_Speed_Ki 0
#define Chassis3_Motor_Speed_Kd 0
#define Chassis3_Motor_Speed_Maxout 5000
#define Chassis3_Motor_Speed_IMaxout 600

#define Chassis4_Motor_Speed_Kp 12
#define Chassis4_Motor_Speed_Ki 0
#define Chassis4_Motor_Speed_Kd 0
#define Chassis4_Motor_Speed_Maxout 5000
#define Chassis4_Motor_Speed_IMaxout 600






#endif 
