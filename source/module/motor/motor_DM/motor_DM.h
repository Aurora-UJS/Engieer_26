#ifndef __MOTOR_DM_H__
#define __MOTOR_DM_H__

#include "main.h"
#include "motor_basic.h"


#define MIT_MODE 			0x000
#define POS_MODE			0x100
#define SPD_MODE			0x200
#define PSI_MODE		  	0x300

#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f

#define hcan_t FDCAN_HandleTypeDef
typedef struct
{
	
    uint32_t MST_ID;	
    float    PMAX;		
    float    VMAX;		
    float    TMAX;		
} esc_inf_t;

typedef enum
{
    Motor_DM_DISABLE = 0,
    MOTOR_DM_ENABLE = 1,
    MOTOR_DM_ERROR_OVER_VOLTAGE = 8,
    MOTOR_DM_ERROR_LESS_VOLTAGE = 9,
    MOTOR_DM_ERROR_OVER_CURRENT = 0xA,
    MOTOR_DM_ERROR_OVER_TEMPERATURE_MOS = 0xB,
    MOTOR_DM_ERROR_OVER_TEMPERATURE_MOTOR = 0xC,
    MOTOR_DM_ERROR_LOSS_COMMUNICATION = 0xD,
    MOTOR_DM_ERROR_OVER_LOAD = 0xE,
}DM_motor_error_t;

typedef struct
{
    can_motor_t motor_msg;//电机数据包
    can_msg_t can_cfg; //CAN发送数据配置
    DM_motor_error_t error_code; //电机错误码
    uint8_t mode; //电机模式
    esc_inf_t tmp;
}DM_motor_t;

void Motor_DM_Init(DM_motor_t *motor);
void Motor_DM_Refresh(DM_motor_t *motor);
void Motor_DM_Enable(DM_motor_t *motor);
void Motor_DM_Disable(DM_motor_t *motor);
void Motor_DM_Save_Zero(DM_motor_t *motor);
void Motor_DM_Clear_Error(DM_motor_t *motor);
void Speed_CtrlMotorDM(DM_motor_t *motor, float vel);
void PosSpeed_CtrlMotorDM(DM_motor_t *motor, float _pos, float _vel);
void MIT_CtrlMotorDM( DM_motor_t *motor,  float pos, float vel,float kp, float kd, float tor);


#endif /* __MOTOR_DM_H__ */