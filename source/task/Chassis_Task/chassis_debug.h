#ifndef CHASSIS_DEBUG_H
#define CHASSIS_DEBUG_H

#include "chassis_config.h"
#include "arm_math_types.h"
#include <stdint.h>

typedef struct
{
    float32_t chassis_target_speed_3508[4];
    float32_t chassis_actual_speed_3508[4];
    float32_t chassis_output_raw_3508[4];
    float32_t chassis_output_3508[4];
    float32_t chassis_power_motor_estimate_3508[4];
    float32_t chassis_power_motor_limited_estimate_3508[4];
    float32_t rising_power_motor_estimate_3508[2];
    float32_t rising_power_motor_limited_estimate_3508[2];
    float32_t chassis_power_total_estimate;
    float32_t chassis_power_total_limited_estimate;
    float32_t chassis_power_referee_limit;
    float32_t chassis_power_referee_actual;
    float32_t chassis_power_buffer_energy;
    float32_t chassis_power_limit_user_max;
    float32_t chassis_power_limit_effective_max;
    float32_t chassis_power_alloc_limit;
    float32_t chassis_power_scale;
    float32_t chassis_power_model_torque_coeff;
    float32_t chassis_power_model_k1;
    float32_t chassis_power_model_k2;
    float32_t chassis_power_model_k3;
    float32_t chassis_power_model_global_scale;
    float32_t chassis_power_scale_attack;
    float32_t chassis_power_scale_release;
    uint8_t chassis_power_limit_enable;
    uint8_t chassis_power_calc_enable[Chassis_PowerCalc_Group_Count];

    float32_t rising_target_speed_3508[2];
    float32_t rising_actual_speed_3508[2];
    float32_t rising_output_3508[2];
    float32_t rising_dm_pid_output[2];

    float32_t rising_target_angle_dm_l;
    float32_t rising_target_angle_dm_r;
    float32_t rising_actual_angle_dm_l;
    float32_t rising_actual_angle_dm_r;
} Chassis_Debug_t;

extern volatile Chassis_Debug_t g_chassis_debug;

#endif /* CHASSIS_DEBUG_H */
