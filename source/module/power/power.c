#include "power.h"
//#include "arm_math.h"
#include "PIDtool.h"
#include "can_bsp.h"
#include "motor_DJI.h"
#include "omni_mecanum_kinematics.h"

void power_init(power * power_control)
{
  power_control->initial_total_power = 0;//初始总功率控制
  power_control->chassis_max_power = 0; //底盘最大输出功率
  power_control-> nomal_power = 4.081;     // 静态功率——此值为车辆在静态时的功率，用于计算车辆在运动时的功率,此处的值要进行更换
  power_control->toque_coefficient = 1.99688994e-6f; // (20/16384)*(0.3)*(187/3591)/9.55 此值要根据电机参数计算
}

//extern cap_measure_t cap_measure; // 电容测量数据


void chassis_power_control(pid_type_def *pid_3508_speed_chassis[4],can_motor_t *chassis_Rx1[4],power *power_control)
{
   float a = 1.23e-07;						 // k1，此值根据拟合值更改
   float k2 = 1.453e-07;					 // k2，同理
   float initial_chassis_give_power[4];//地盘实时功率
   float initial_chassis_total_power = 0;//底盘总功率控制
   float scaled_chassis_give_power[4];//缩放后的功率
   

   //以下是只针对于底盘的功率控制
	for (uint8_t i = 0; i < 4; i++) //计算初始底盘电机功率和总电机功率,此处最重要的是机械功率——即为第一项
	{
		initial_chassis_give_power[i] = pid_3508_speed_chassis[i]->Pout * power_control->toque_coefficient * chassis_Rx1[i]->motor_speed  +
								k2 * chassis_Rx1[i]->motor_speed * chassis_Rx1[i]->motor_speed +
								a * pid_3508_speed_chassis[i]->Pout * pid_3508_speed_chassis[i]->Pout + power_control->nomal_power;

		if (initial_chassis_give_power[i] < 0) //负功率不包含（过渡）
			continue;
		initial_chassis_total_power += initial_chassis_give_power[i];//此处为总功率
	}

	if (initial_chassis_total_power > power_control->chassis_max_power) //功率判断，确定是否大于最大功率
	{
		float power_scale = power_control->chassis_max_power / initial_chassis_total_power;//功率缩放比例
		for (uint8_t i = 0; i < 4; i++)
		{
			scaled_chassis_give_power[i] = initial_chassis_give_power[i] * power_scale; //缩放的功率
			if (scaled_chassis_give_power[i] < 0)
			{
				continue;
			}
            //此后的内容为力矩的逆解，但此处是以逆解的电流值输出的
			float b = power_control->toque_coefficient * chassis_Rx1[i]->motor_speed;
			float c = k2 * chassis_Rx1[i]->motor_speed * chassis_Rx1[i]->motor_speed - scaled_chassis_give_power[i] + power_control->nomal_power;

			if (pid_3508_speed_chassis[i]->Pout > 0) // Selection of the calculation formula according to the direction of the original moment
			{
				float temp = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
				if (temp > 16000)
				{
					pid_3508_speed_chassis[i]->Pout = 16000;
				}
				else
					pid_3508_speed_chassis[i]->Pout = temp;
			}
			else
			{
				float temp = (-b - sqrt(b * b - 4 * a * c)) / (2 * a);
				if (temp < -16000)
				{
					pid_3508_speed_chassis[i]->Pout = -16000;
				}
				else
					pid_3508_speed_chassis[i]->Pout = temp;
			}
		}
	}



}



