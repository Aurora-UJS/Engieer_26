#ifndef UINT_FLOAT_CONVERT_H
#define UINT_FLOAT_CONVERT_H



/**
 * @brief 浮点数转换为无符号整数
 * 
 * @param x_float 待转换的浮点数
 * @param x_min 范围最小值
 * @param x_max 范围最大值
 * @param bits 目标无符号整数的位数 (1-32)
 * @param result 转换结果输出参数

 */
int float_to_uint(float x_float, float x_min, float x_max, int bits);

/**
 * @brief 无符号整数转换为浮点数
 * 
 * @param x_int 待转换的无符号整数
 * @param x_min 范围最小值
 * @param x_max 范围最大值
 * @param bits 无符号整数的位数 (1-32)
 * @param result 转换结果输出参数
 */
float uint_to_float(unsigned int x_int, float x_min, float x_max, int bits);


#endif // MOTOR_DJI_H