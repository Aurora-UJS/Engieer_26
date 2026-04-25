#include "chassis_debug.h"
#include "chassis_config.h"

volatile Chassis_Debug_t g_chassis_debug = {
    .chassis_power_limit_enable = Chassis_PowerLimit_Enable_Default,
    .chassis_power_limit_user_max = Chassis_PowerLimit_UserMax_Default,
    .chassis_power_limit_effective_max = Chassis_PowerLimit_UserMax_Default,
    .chassis_power_alloc_limit = Chassis_PowerLimit_UserMax_Default,
    .chassis_power_scale = 1.0f,
    .chassis_power_model_torque_coeff = Chassis_PowerModel_TorqueCoeff_Default,
    .chassis_power_model_k1 = Chassis_PowerModel_K1_Default,
    .chassis_power_model_k2 = Chassis_PowerModel_K2_Default,
    .chassis_power_model_k3 = Chassis_PowerModel_K3_Default,
    .chassis_power_model_global_scale = Chassis_PowerModel_GlobalScale_Default,
    .chassis_power_scale_attack = Chassis_PowerScale_Attack_Default,
    .chassis_power_scale_release = Chassis_PowerScale_Release_Default,
    .chassis_power_calc_enable = {
        [Chassis_PowerCalc_Group_Chassis] = 1,
        [Chassis_PowerCalc_Group_Rising] = 1,
    },
};
