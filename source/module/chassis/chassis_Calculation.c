

#include "stm32h7xx.h" 
#include "math.h"
#include "chassis_Calculation.h"
#include <stdint.h>

#include "DbusSys.h"

#define ONE_PI   (3.14159265)



float MOTOR_speed_out[4];//存放解算后四个电机输出数值，作为PID的电机输出期望值
short motion[3];         //将遥控器数据对应为x，y，z三轴的数据值
float angle_to_radian = 0.01745f;//角度转弧度



double mx_sin(double rad)
{   
	double sine;
	if (rad < 0)
		sine = rad * (1.27323954f + 0.405284735f * rad);
	else
		sine = rad * (1.27323954f - 0.405284735f * rad);
	if (sine < 0)
		sine = sine*(-0.225f * (sine + 1) + 1);
	else
		sine = sine * (0.225f *( sine - 1) + 1);
	return sine;
}

double my_sin(double rad)
{
	int8_t flag = 1;
	
	while(rad > 2*ONE_PI)
	{
		rad = rad -  2*ONE_PI;
	}
	if (rad >= ONE_PI)
	{
		rad -= ONE_PI;
		flag = -1;
	}

	return mx_sin(rad) * flag;
}

double my_cos(double rad)
{
	int8_t flag = 1;
	rad += ONE_PI/2.0;
   
	while(rad > 2*ONE_PI)
	{
		rad = rad -  2*ONE_PI;
	}
	if (rad >= ONE_PI)
	{
		flag = -1;
		rad -= ONE_PI;
	}
	return my_sin(rad)*flag;
}

/**
 * @brief 将底盘运动向量转换为麦克纳姆轮速度
 * 
 * 该函数根据遥控器输入的通道值计算底盘的运动向量，并将其转换为四个麦克纳姆轮的速度输出。
 * 由于云台靠前，在旋转时，前面两轮（0号和1号）的旋转速度变慢，后面两轮（2号和3号）的旋转速度变快。
 * 
 * @param 无
 * @return 无
 */
void chassis_vector_to_mecanum_wheel_speed()
{
    // 旋转的时候， 由于云台靠前，所以是前面两轮 0 ，1 旋转的速度变慢， 后面两轮 2,3 旋转的速度变快
    // 将遥控器的通道值转换为底盘运动向量的x速度分量
    motion[x_speed] = (float)(remoter.ch1)/660.000f * Chassis_RC_Max_Speed;
    // 将遥控器的通道值转换为底盘运动向量的y速度分量
    motion[y_speed] = (float)(remoter.ch2)/660.000f * Chassis_RC_Max_Speed;
    // 将遥控器的通道值转换为底盘运动向量的z速度分量，并取反
    motion[z_speed] = (float)(remoter.ch3)/660.000f * Chassis_RC_Max_Speed*(-1);

    // 计算1号麦克纳姆轮（左前）的速度输出
    MOTOR_speed_out[MOTOR1] = motion[x_speed] + motion[y_speed] + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * motion[z_speed];//1号左前
    // 计算2号麦克纳姆轮（左后）的速度输出
    MOTOR_speed_out[MOTOR2] = motion[x_speed] - motion[y_speed] + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * motion[z_speed];//2号左后
    // 计算3号麦克纳姆轮（右后）的速度输出
    MOTOR_speed_out[MOTOR3] = -motion[x_speed] - motion[y_speed] + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * motion[z_speed];//3号右后
    // 计算4号麦克纳姆轮（右前）的速度输出
    MOTOR_speed_out[MOTOR4] = -motion[x_speed] + motion[y_speed] + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * motion[z_speed];//4号右前
}




