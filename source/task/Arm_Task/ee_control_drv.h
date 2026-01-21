#ifndef EE_CONTROL_DRVH
#define EE_CONTROL_DRVH

#include "motor_DM.h"

typedef struct endEffector_t{
    DM_motor_t *endEffector_motor;
} endEffector_t;

typedef enum{
    GPIPER_STATE_IDLE = 0,
    GRIPPER_STATE_OPEN,
    GPIPER_STATE_CLOSE
}GPIPER_STATE ;
/** @brief 末端执行器初始化*/
void endeffector_init(endEffector_t *endeffector);

/** @brief 末端执行器电机初始化*/
void endeffector_motor_init(endEffector_t *endeffector);

/** @brief 末端执行器信息更新 */
void EndEffector_Motor_Refresh(endEffector_t *endeffector);

/** @brief 末端执行器电机使能 */
void EndEffector_Motor_Enable(endEffector_t *endeffector);

#endif
