#include "tool.h"
#include "stdint.h"
#include "string.h"

float circule_compensate(float target, float current, float circule_num)
{
    float temp = target - current;
    if (temp > circule_num / 2.0f)
    {
        temp = -circule_num;
    }
    else if (temp < -circule_num / 2.0f)
    {
        temp = circule_num;
    }
    else
    {
        temp = 0;
    }
    return temp;
}