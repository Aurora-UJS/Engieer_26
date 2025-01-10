#ifndef MOTION_STATE_H_
#define MOTION_STATE_H_

#include "arm_math.h"
#include "main.h"

#define MOTORLF   0
#define MOTORLB   1
#define MOTORRB   2
#define MOTORRF   3

/**
 * @brief 基本运动矢量结构体
 * 
 * 该结构体用于表示机器人的基本运动矢量，包括x方向速度、y方向速度和旋转速度。
 * 
 * @param x x方向的速度
 * @param y y方向的速度
 * @param wz 旋转速度
 */
typedef struct 
{
    float x;
    float y;
    float wz;
} basic_vector_t;

#endif // !MOTION_STATE_H_