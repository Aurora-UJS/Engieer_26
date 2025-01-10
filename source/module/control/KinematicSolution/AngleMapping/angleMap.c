#include "angleMap.h"
#include "arm_math.h"
#include "motion_state.h"

/**
 * @brief 执行坐标系变换，将输入向量从一个参考系变换到另一个参考系。
 *
 * 该函数根据给定的角度对输入的二维向量进行旋转，并保持角速度分量不变。
 * 变换公式为：
 *     x' = x * cos(angle) - y * sin(angle)
 *     y' = x * sin(angle) + y * cos(angle)
 *     wz' = wz (保持不变)
 *
 * @param basic 输入的基本向量，包含x, y坐标和角速度wz。
 * @param angle 旋转角度（弧度制）。
 * @return 返回变换后的基本向量。
 */
basic_vector_t RefFrame_Transform_Calc(const basic_vector_t basic, float angle)
{
    basic_vector_t result;
    result.x = basic.x * cos(angle) - basic.y * sin(angle);
    result.y = basic.x * sin(angle) + basic.y * cos(angle);
    result.wz = basic.wz;
    return result;
}