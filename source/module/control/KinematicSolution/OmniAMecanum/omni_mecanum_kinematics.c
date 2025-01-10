

#include "stm32h7xx.h" 
#include "arm_math.h"
#include "omni_mecanum_kinematics.h"
#include <stdint.h>
#include "motion_state.h"

/**
 * @brief 计算全向轮（麦克纳姆轮）的运动学
 * 
 * 该函数根据机器人的运动矢量（x方向速度、y方向速度和旋转速度）计算四个麦克纳姆轮的电机速度输出。
 * 这使得机器人可以根据给定的运动矢量以全向方式移动。
 * 
 * @param motion_vector 指向包含机器人运动矢量的结构体指针，包括x、y方向速度和旋转速度wz
 * @param motor_speed_out 指向用于存储计算出的四个电机速度输出的数组
 */
void omni_mecanum_kinematics(basic_vector_t *motion_vector,float *motor_speed_out)
{

    // 计算1号麦克纳姆轮（左前）的速度输出
    motor_speed_out[MOTORLF] = motion_vector->x + motion_vector->y + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * motion_vector->wz;//1号左前
    // 计算2号麦克纳姆轮（左后）的速度输出
    motor_speed_out[MOTORLB] = motion_vector->x - motion_vector->y + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * motion_vector->wz;//2号左后
    // 计算3号麦克纳姆轮（右后）的速度输出
    motor_speed_out[MOTORRB] = -motion_vector->x - motion_vector->y + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * motion_vector->wz;//3号右后
    // 计算4号麦克纳姆轮（右前）的速度输出
    motor_speed_out[MOTORRF] = -motion_vector->x + motion_vector->y + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * motion_vector->wz;//4号右前
}


