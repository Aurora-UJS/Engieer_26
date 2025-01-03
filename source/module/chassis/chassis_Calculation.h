#ifndef __CHASSIS_CACULATION_H__
#define __CHASSIS_CACULATION_H__

#include "main.h"


#define MOTOR1   0
#define MOTOR2   1
#define MOTOR3   2
#define MOTOR4   3

#define x_speed   0
#define y_speed   1
#define z_speed   2

//底盘设置旋转速度，设置前后左右轮不同设定速度的比例分权 0为在几何中心，不需要补偿
#define CHASSIS_WZ_SET_SCALE 0.1f

#define MOTOR_DISTANCE_TO_CENTER 1.2f 

#define Chassis_Max_Speed  660		//最大速度
#define Chassis_Shift_Max_Speed 660	//                                                                                          
#define Chassis_RC_Max_Speed   5000		//
#define Multiple               5     //目标值乘的倍数

extern float MOTOR_speed_out[4];
extern float yaw;
extern short motion[3];

double mx_sin(double rad);
double my_sin(double rad);
double my_cos(double rad);

void chassis_vector_to_mecanum_wheel_speed(void);


#endif 
