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
float map(float value, float in_min, float in_max, float out_min, float out_max);
float limit(float value, float min, float max);
#endif // !TOOL

