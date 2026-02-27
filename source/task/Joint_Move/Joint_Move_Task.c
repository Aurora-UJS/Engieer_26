#include "can_struct.h"
#include "cmsis_os2.h"
#include "jointFollowAngle.h"
#include "motor_DM.h"
#include "portable.h"

extern target_point_t Target_Point[6];
extern Joint_t Joint[JOINT_NUM];
__attribute__((deprecated))
void Joint1_Move_Task(void *argument) {
  Joint[0].joint_motor = pvPortMalloc(sizeof(DM_motor_t));
  Joint[0].joint_motor->can_cfg.id = 0x01;
  Joint[0].joint_motor->motor_msg.can_msg.id = 0x11;

  Joint[0].joint_motor->tmp.PMAX = 12.5f;
  Joint[0].joint_motor->tmp.VMAX = 3.0f;
  Joint[0].joint_motor->tmp.TMAX = 1.0f;

  Joint[0].joint_motor->can_cfg.port = CAN3_PORT;
  Motor_DM_Init(Joint[0].joint_motor);

  osDelay(100);
  Motor_DM_Enable(Joint[0].joint_motor);
  UNUSED(argument);
  while (1) {
    /* for (int joint_index = 0; joint_index < JOINT_NUM; joint_index++) {
      PosSpeed_CtrlMotorDM(Joint[joint_index].joint_motor,
                           Target_Point[joint_index].target_joint_radian,
                           Target_Point[joint_index].velocity);
      osDelay(2);
    } */


/*     Joint_Motor_PosSpeed_Ctrl(&Joint[1],Target_Point[1]);
    osDelay(2);

    Joint_Motor_PosSpeed_Ctrl(&Joint[4],Target_Point[4]);
    osDelay(2);


    Joint_Motor_PosSpeed_Ctrl(&Joint[2],Target_Point[2]);
    osDelay(2);



    Joint_Motor_PosSpeed_Ctrl(&Joint[0],Target_Point[0]);
    osDelay(2);

    Joint_Motor_PosSpeed_Ctrl(&Joint[3],Target_Point[3]);
    osDelay(2);

    Joint_Motor_PosSpeed_Ctrl(&Joint[5],Target_Point[5]);
    osDelay(2); */
  }
}

__attribute__((deprecated))
void Joint2_Move_Task(void *argument) {
  Joint[1].joint_motor = pvPortMalloc(sizeof(DM_motor_t));
  Joint[1].joint_motor->can_cfg.id = 0x02;
  Joint[1].joint_motor->motor_msg.can_msg.id = 0x12;

  Joint[1].joint_motor->tmp.PMAX = 12.5f;
  Joint[1].joint_motor->tmp.VMAX = 3.0f;
  Joint[1].joint_motor->tmp.TMAX = 1.0f;

  Joint[1].joint_motor->can_cfg.port = CAN3_PORT;
  Motor_DM_Init(Joint[1].joint_motor);

  osDelay(100);
  Motor_DM_Enable(Joint[1].joint_motor);
  UNUSED(argument);

  while (1) {
    // PosSpeed_CtrlMotorDM(Joint[1].joint_motor,
    //                      Target_Point[1].target_joint_radian,
    //                      Target_Point[1].velocity);
    osDelay(3);
  }
}

__attribute__((deprecated))
void Joint3_Move_Task(void *argument) {
  Joint[2].joint_motor = pvPortMalloc(sizeof(DM_motor_t));
  Joint[2].joint_motor->can_cfg.id = 0x03;
  Joint[2].joint_motor->motor_msg.can_msg.id = 0x13;

  Joint[2].joint_motor->tmp.PMAX = 12.5f;
  Joint[2].joint_motor->tmp.VMAX = 3.0f;
  Joint[2].joint_motor->tmp.TMAX = 1.0f;

  Joint[2].joint_motor->can_cfg.port = CAN2_PORT;
  Motor_DM_Init(Joint[2].joint_motor);

  osDelay(100);
  Motor_DM_Enable(Joint[2].joint_motor);
  UNUSED(argument);

  while (1) {
    // PosSpeed_CtrlMotorDM(Joint[2].joint_motor,
    //                      Target_Point[2].target_joint_radian,
    //                      Target_Point[2].velocity);
    osDelay(3);
  }
}

__attribute__((deprecated))
void Joint4_Move_Task(void *argument) {
  Joint[3].joint_motor = pvPortMalloc(sizeof(DM_motor_t));
  Joint[3].joint_motor->can_cfg.id = 0x04;
  Joint[3].joint_motor->motor_msg.can_msg.id = 0x14;

  Joint[3].joint_motor->tmp.PMAX = 12.5f;
  Joint[3].joint_motor->tmp.VMAX = 3.0f;
  Joint[3].joint_motor->tmp.TMAX = 1.0f;

  Joint[3].joint_motor->can_cfg.port = CAN2_PORT;
  Motor_DM_Init(Joint[3].joint_motor);

  osDelay(100);
  Motor_DM_Enable(Joint[3].joint_motor);
  UNUSED(argument);

  while (1) {
    // PosSpeed_CtrlMotorDM(Joint[3].joint_motor,
    //                      Target_Point[3].target_joint_radian,
    //                      Target_Point[3].velocity);
    osDelay(3);
  }
}

__attribute__((deprecated))
void Joint5_Move_Task(void *argument) {
  Joint[4].joint_motor = pvPortMalloc(sizeof(DM_motor_t));
  Joint[4].joint_motor->can_cfg.id = 0x05;
  Joint[4].joint_motor->motor_msg.can_msg.id = 0x15;

  Joint[4].joint_motor->tmp.PMAX = 12.5f;
  Joint[4].joint_motor->tmp.VMAX = 3.0f;
  Joint[4].joint_motor->tmp.TMAX = 1.0f;

  Joint[4].joint_motor->can_cfg.port = CAN2_PORT;
  Motor_DM_Init(Joint[4].joint_motor);

  osDelay(100);
  Motor_DM_Enable(Joint[4].joint_motor);
  UNUSED(argument);

  while (1) {
    // PosSpeed_CtrlMotorDM(Joint[4].joint_motor,
    //                      Target_Point[4].target_joint_radian,
    //                      Target_Point[4].velocity);
    osDelay(3);
  }
}

__attribute__((deprecated))
void Joint6_Move_Task(void *argument) {
  Joint[5].joint_motor = pvPortMalloc(sizeof(DM_motor_t));
  Joint[5].joint_motor->can_cfg.id = 0x06;
  Joint[5].joint_motor->motor_msg.can_msg.id = 0x16;

  Joint[5].joint_motor->tmp.PMAX = 12.5f;
  Joint[5].joint_motor->tmp.VMAX = 3.0f;
  Joint[5].joint_motor->tmp.TMAX = 1.0f;

  Joint[5].joint_motor->can_cfg.port = CAN2_PORT;
  Motor_DM_Init(Joint[5].joint_motor);

  osDelay(100);
  Motor_DM_Enable(Joint[5].joint_motor);
  UNUSED(argument);

  while (1) {
    // PosSpeed_CtrlMotorDM(Joint[5].joint_motor,
    //                      Target_Point[5].target_joint_radian,
    //                      Target_Point[5].velocity);
    osDelay(3);
  }
}
