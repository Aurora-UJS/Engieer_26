#ifndef EE_CONTROL_DRVH
#define EE_CONTROL_DRVH

#include "cmsis_os2.h"
#include "motor_DM.h"

typedef struct endEffector_t{
    DM_motor_t *endEffector_motor;
} endEffector_t;


#define GRIPPER_OPEN_RADIAN (0.0f)
#define GRIPPER_CLOSE_RADION (1.2f)
#define GRIPPER_VEL (1.5f)

/** @brief 末端执行器初始化*/
void endEffector_init(endEffector_t *endeffector);

/** @brief 末端执行器电机初始化*/
void endEffector_motor_init(endEffector_t *endeffector);

/** @brief 末端执行器信息更新 */
void EndEffector_Motor_Refresh(endEffector_t *endeffector);

/** @brief 末端执行器电机使能 */
void EndEffector_Motor_Enable(endEffector_t *endeffector);

/** @brief 夹爪打开 */
void Gripper_Open(endEffector_t *endeffector);

/** @brief 夹爪关闭 */
void Gripper_Close(endEffector_t *endEffector);
#endif
