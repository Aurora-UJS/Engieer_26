#ifndef IMUTOOL_H
#define IMUTOOL_H
#include "main.h"
#include "MahonyAHRS.h"

typedef struct
{
    struct 
    {
        float x;
        float y;
        float z;
    }accel;
    struct 
    {
        float x;
        float y;
        float z;
    }gyro;
    struct 
    {
        float pitch;
        float roll;
        float yaw;
    }angles;
    float temp;

}bmi088_data_t;

extern bmi088_data_t bmi088_data;

extern uint8_t attitude_flag; /*姿态数据标志位*/

extern float yaw_angle_now;

void temple(void);
void IMUsys(void);

#endif // !IMUTOOL_H