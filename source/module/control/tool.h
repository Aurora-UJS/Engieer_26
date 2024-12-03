#ifndef TOOL
#define TOOL

#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

#define   Pi 3.14159265358979323846

float circule_compensate(float target, float current, float circule_num);

#endif // !TOOL

