#ifndef MOTORTOOL
#define MOTORTOOL

#define FEEDBACK_ID_BASE      0x200

#include "main.h"

typedef struct
{
    uint16_t can_id;
    int16_t  set_voltage;
    uint16_t rotor_angle;
    int16_t  rotor_speed;
    int16_t  torque_current;
    uint8_t  temp;
}moto_info_t;

extern moto_info_t moto_data[16];
extern uint8_t can5FIFO_flag;
extern uint8_t Data_Enable[8];		//达妙电机使能命令
extern uint8_t Data_Failure[8];		//电机失能命令
extern uint8_t Data_Save_zero[8];	    //电机保存零点命令

void can_output(FDCAN_HandleTypeDef *hcan, uint16_t ID, int16_t v1, int16_t v2, int16_t v3, int16_t v4);
void PosSpeed_CtrlMotor(FDCAN_HandleTypeDef *hcan, uint16_t ID, float _pos, float _vel);
void Speed_CtrlMotor(FDCAN_HandleTypeDef *hcan, uint16_t ID, float _vel);
void can_input_RM( uint16_t ID, uint8_t *rx_data);
void can_input_dm( uint16_t ID, uint8_t *rx_data);


#endif // !MOTORTOOL