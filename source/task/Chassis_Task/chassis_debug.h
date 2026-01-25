 #ifndef CHASSIS_DEBUG_H
 #define CHASSIS_DEBUG_H

 #include "arm_math_types.h"

 typedef struct
 {
     float32_t chassis_target_speed_3508[4];
     float32_t chassis_actual_speed_3508[4];

     float32_t rising_target_speed_3508[2];
     float32_t rising_actual_speed_3508[2];

     float32_t rising_target_angle_dm_l;
     float32_t rising_target_angle_dm_r;
     float32_t rising_actual_angle_dm_l;
     float32_t rising_actual_angle_dm_r;
 } Chassis_Debug_t;

 extern volatile Chassis_Debug_t g_chassis_debug;

 #endif /* CHASSIS_DEBUG_H */
