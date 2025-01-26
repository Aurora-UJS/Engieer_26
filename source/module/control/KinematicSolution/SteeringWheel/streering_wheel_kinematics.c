#include "streering_wheel_kinematics.h"
#include "arm_math.h"

/**
 * 计算转向轮的运动学特性
 * 
 * 根据机器人的运动向量和电机的角度向量，计算四个轮子（左前、右前、左后、右后）的线速度和角速度
 * 这是通过将机器人的全局运动分解到每个轮子的局部坐标系中来实现的
 * https://blog.csdn.net/weixin_73037889/article/details/131052481
 * 
 * @param motion_vector 指向机器人运动向量的指针，包含机器人的线速度（x, y）和角速度（wz）
 * @param motor_angleV_out 指向电机角度向量的指针，用于输出每个轮子的角速度和线速度
 * @return 返回计算后的电机角度向量
 */
basic_angleV_t steering_wheel_kinematics(basic_vector_t *motion_vector, basic_angleV_t *motor_angleV_out)
{
    // 计算左前轮的角速度和线速度
    motor_angleV_out[MOTORLF].v = sqrtf(powf(motion_vector->y - motion_vector->wz * cosf(PI / 4), 2) + powf(motion_vector->x - motion_vector->wz * sinf(PI / 4), 2));
    motor_angleV_out[MOTORLF].angle = atan2f(motion_vector->y - motion_vector->wz * sinf(PI / 4), motion_vector->x - motion_vector->wz * cosf(PI / 4)) - PI / 4;

    // 计算右前轮的角速度和线速度
    motor_angleV_out[MOTORRF].v = sqrtf(powf(motion_vector->y - motion_vector->wz * cosf(PI / 4), 2) + powf(motion_vector->x + motion_vector->wz * sinf(PI / 4), 2));
    motor_angleV_out[MOTORRF].angle = atan2f(motion_vector->y - motion_vector->wz * sinf(PI / 4), motion_vector->x + motion_vector->wz * cosf(PI / 4)) - PI / 4;

    // 计算左后轮的角速度和线速度
    motor_angleV_out[MOTORLB].v = sqrtf(powf(motion_vector->y + motion_vector->wz * cosf(PI / 4), 2) + powf(motion_vector->x + motion_vector->wz * sinf(PI / 4), 2));
    motor_angleV_out[MOTORLB].angle = atan2f(motion_vector->y + motion_vector->wz * sinf(PI / 4), motion_vector->x + motion_vector->wz * cosf(PI / 4)) - PI / 4;

    // 计算右后轮的角速度和线速度
    motor_angleV_out[MOTORRB].v = sqrtf(powf(motion_vector->y + motion_vector->wz * cosf(PI / 4), 2) + powf(motion_vector->x - motion_vector->wz * sinf(PI / 4), 2));
    motor_angleV_out[MOTORRB].angle = atan2f(motion_vector->y + motion_vector->wz * sinf(PI / 4), motion_vector->x - motion_vector->wz * cosf(PI / 4)) - PI / 4;

    // 返回电机角度向量
    return *motor_angleV_out;
}
