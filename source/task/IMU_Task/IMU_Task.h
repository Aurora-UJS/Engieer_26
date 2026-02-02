#ifndef IMU_TASK_H
#define IMU_TASK_H

#include "arm_math_types.h"

#include <stdint.h>
typedef struct
{
    float32_t Yaw;
    float32_t Pitch;
    float32_t Roll;
}IMU_data_t;

extern IMU_data_t IMU_data;
extern int64_t IMU_data_time;

void IMU_Task(void *argument);
#endif /* IMU_TASK_H */
