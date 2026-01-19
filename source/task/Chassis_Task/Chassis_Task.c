
#include "Chassis_Task.h"
#include "DbusSys.h"
#include "IMUtool.h"
#include "PIDtool.h"
#include "arm_math_types.h"
#include "cmsis_os2.h"
#include "motion_state.h"
#include "motor_DJI.h"
#include "motor_DM.h"
#include "omni_mecanum_kinematics.h"
#include "tim.h"
#include "tool.h"
#include "usart.h"
#include <stdint.h>
#include <string.h>


// ch1 右摇杆 左右 左-右+
// ch2 右摇杆 前后 前+后-
// ch3 左摇杆 左右 左-右+
// ch4 左摇杆 前后 前+后-
// sw1 左拨码开关 前1 中3 后2
// sw2 右拨码开关 前1 中3 后2

extern rc_info_t remoter;
float32_t test[4];

// 函数前向声明
float32_t Double_Encoder_Tool(float32_t motor_angle);

void Chassis_Task(void *argument) 
{
  /* USER CODE Chassis_Task */
  UNUSED(argument);
  osDelay(200);
  int16_t Chassis_3508_Ctrl_Output[4];
  float32_t Chassis_3508_Target_Velocity[4];
  pid_type_def Chassis_Motor_3508_PID[4];
  DJI_motor_t *Chassis_Motor_3508;
  Chassis_Motor_3508 = pvPortMalloc(sizeof(DJI_motor_t));

  int16_t Rising_3508_Ctrl_Output[2];
  float32_t Rising_3508_Target_Velocity[2];
  pid_type_def Rising_Motor_3508_PID[2];
  DJI_motor_t *Rising_Motor_3508;
  Rising_Motor_3508 = pvPortMalloc(sizeof(DJI_motor_t));

  float32_t DM10010l_Target_Angle_L = 0;
  float32_t DM10010l_Target_Angle_R = 0;
  float32_t DM10010l_output_L;
  float32_t DM10010l_output_R;
  DM_motor_t *Rising_Motor_10010l_L;
  DM_motor_t *Rising_Motor_10010l_R;
  pid_type_def *Rising_Motor_DM_PID_L;
  pid_type_def *Rising_Motor_DM_PID_R;
  Rising_Motor_10010l_L = pvPortMalloc(sizeof(DM_motor_t));
  Rising_Motor_10010l_R = pvPortMalloc(sizeof(DM_motor_t));
  Rising_Motor_DM_PID_L = pvPortMalloc(sizeof(pid_type_def));
  Rising_Motor_DM_PID_R = pvPortMalloc(sizeof(pid_type_def));

  Motor_Init_DJI(&Chassis_Motor_3508, &Rising_Motor_3508);
  Motor_Init_DM(&Rising_Motor_10010l_L,&Rising_Motor_10010l_R);
  Chassis_3508_PID_Init(Chassis_Motor_3508_PID);
  Rising_3508_PID_Init(Rising_Motor_3508_PID); 
  Rising_DM_PID_Init(Rising_Motor_DM_PID_L,Rising_Motor_DM_PID_R);
  /* Infinite loop */
  for (;;) 
  {
    switch (Chassis_Mode_Get(&remoter)) 
    {
      case Chassis_PowerOff:
        for(int i = 0; i < 4; i++)
        {
          Chassis_3508_Ctrl_Output[i] = 0;
        }
        Chassis_Motor_SendControl_DJI(Chassis_Motor_3508,
                                    Chassis_3508_Ctrl_Output);
      break;

      case Chassis_Normal:
        Chassis_Motor_TargetVelocity(Chassis_3508_Target_Velocity, remoter);
        Chassis_3508_PID_Calculate(Chassis_Motor_3508_PID,
                                 Chassis_3508_Target_Velocity,
                                 Chassis_Motor_3508, 
                                 Chassis_3508_Ctrl_Output);
        Chassis_Motor_SendControl_DJI(Chassis_Motor_3508,
                                    Chassis_3508_Ctrl_Output);
      break;

      case Chassis_Upstairs:
        remoter.ch3 = 0;
        Chassis_Motor_TargetVelocity(Chassis_3508_Target_Velocity, remoter);
        Rising_Motor_TargetVelocity(Rising_3508_Target_Velocity, remoter);
        Rising_Motor_TargetAngle(&DM10010l_Target_Angle_L, &DM10010l_Target_Angle_R, remoter);
        Chassis_3508_PID_Calculate(Chassis_Motor_3508_PID,
                                 Chassis_3508_Target_Velocity,
                                 Chassis_Motor_3508, 
                                 Chassis_3508_Ctrl_Output);
        Rising_3508_PID_Calculate(Rising_Motor_3508_PID,
                                Rising_3508_Target_Velocity, 
                                Rising_Motor_3508,
                                Rising_3508_Ctrl_Output);
        Rising_DM_PID_Calculate(Rising_Motor_DM_PID_L, 
                                Rising_Motor_DM_PID_R, 
                                DM10010l_Target_Angle_L, 
                                DM10010l_Target_Angle_R, 
                                Rising_Motor_10010l_L, 
                                Rising_Motor_10010l_R, 
                                &DM10010l_output_L, 
                                &DM10010l_output_R);
        Chassis_Motor_SendControl_DJI(Chassis_Motor_3508,
                                    Chassis_3508_Ctrl_Output);
        Rising_Motor_SendControl_DJI(Rising_Motor_3508, 
                                  Rising_3508_Ctrl_Output);
        Rising_Motor_SendControl_DM(Rising_Motor_10010l_L,
                                  Rising_Motor_10010l_R,
                                  DM10010l_output_L,
                                  DM10010l_output_R);
        test[0]=DM10010l_output_L;
        test[1]=DM10010l_Target_Angle_L;
        test[2]=Rising_Motor_10010l_L->motor_msg.motor_angle;
        test[3]=Double_Encoder_Tool(Rising_Motor_10010l_L->motor_msg.motor_angle);
      break;    

      default:

      break;
    }
    osDelay(2);
  }
  /* USER CODE END Chassis_Task */
}

uint8_t Chassis_Mode_Get(rc_info_t *backdata) 
{
  uint8_t Chassis_Mode;
  switch (backdata->sw2) 
  {
    case 1:
      Chassis_Mode = Chassis_Upstairs;
      break;
    case 2:
      Chassis_Mode = Chassis_PowerOff;
      break;
    case 3:
      Chassis_Mode = Chassis_Normal;
      break;
    default:
      Chassis_Mode = Chassis_PowerOff;
  }
  return Chassis_Mode;
}

void Motor_Init_DJI(DJI_motor_t **Chassis_Motor, DJI_motor_t **Rising_Motor) 
{

  if (*Chassis_Motor == NULL) 
  {
    return;
  }
  memset(*Chassis_Motor, 0, sizeof(DJI_motor_t)); // 清零内存
  (*Chassis_Motor)->can_cfg.port = CAN1_PORT;
  (*Chassis_Motor)->can_cfg.id = Chassis_Motor_ALL_id;
  (*Chassis_Motor)->motor_msg[Chassis_Motor_3508_ZQ].can_msg.id =
      Chassis_Motor_3508_ZQ_id;
  (*Chassis_Motor)->motor_msg[Chassis_Motor_3508_ZH].can_msg.id =
      Chassis_Motor_3508_ZH_id;
  (*Chassis_Motor)->motor_msg[Chassis_Motor_3508_YH].can_msg.id =
      Chassis_Motor_3508_YH_id;
  (*Chassis_Motor)->motor_msg[Chassis_Motor_3508_YQ].can_msg.id =
      Chassis_Motor_3508_YQ_id;
  Motor_DJI_Init(*Chassis_Motor);

  if (*Rising_Motor == NULL) 
  {
    return;
  }
  memset(*Rising_Motor, 0, sizeof(DJI_motor_t)); 
  (*Rising_Motor)->can_cfg.port = CAN1_PORT;
  (*Rising_Motor)->can_cfg.id = Rising_Motor_ALL_id;
  (*Rising_Motor)->motor_msg[Rising_Motor_3508_Left].can_msg.id =
      Rising_Motor_3508_Left_id;
  (*Rising_Motor)->motor_msg[Rising_Motor_3508_Right].can_msg.id =
      Rising_Motor_3508_Right_id;
  Motor_DJI_Init(*Rising_Motor);
}

void Motor_Init_DM(DM_motor_t **Rising_Motor_L,DM_motor_t **Rising_Motor_R)
{
    if (*Rising_Motor_L == NULL) 
  {
    return;
  }
  memset(*Rising_Motor_L, 0, sizeof(DM_motor_t)); 
  (*Rising_Motor_L)->can_cfg.id = DM_l0010l_CAN_ID_Left + MIT_MODE;
  (*Rising_Motor_L)->motor_msg.can_msg.id = DM_l0010l_Master_ID_Left;
  (*Rising_Motor_L)->can_cfg.port = CAN3_PORT;
  (*Rising_Motor_L)->tmp.PMAX = 12.5;
  (*Rising_Motor_L)->tmp.VMAX = 3;
  (*Rising_Motor_L)->tmp.TMAX = 100;
  Motor_DM_Init(*Rising_Motor_L);
  Motor_DM_Enable(*Rising_Motor_L);

  if (*Rising_Motor_R == NULL) 
  {
    return;
  }
  memset(*Rising_Motor_R, 0, sizeof(DM_motor_t)); 
  (*Rising_Motor_R)->can_cfg.id = DM_l0010l_CAN_ID_Right + MIT_MODE;
  (*Rising_Motor_R)->motor_msg.can_msg.id = DM_l0010l_Master_ID_Right;
  (*Rising_Motor_R)->can_cfg.port = CAN3_PORT;
  (*Rising_Motor_R)->tmp.PMAX = 12.5;
  (*Rising_Motor_R)->tmp.VMAX = 3;
  (*Rising_Motor_R)->tmp.TMAX = 100;
  Motor_DM_Init(*Rising_Motor_R);  
  Motor_DM_Enable(*Rising_Motor_R);
  osDelay(200);
  Motor_DM_Save_Zero(*Rising_Motor_R);
}

void Chassis_Motor_SendControl_DJI(DJI_motor_t *DJMotor, int16_t output[]) 
{
  Motor_DJI_Refresh(DJMotor);
  set_motor_parameter(
      DJMotor, 
      output[Chassis_Motor_3508_ZQ], 
      output[Chassis_Motor_3508_ZH],
      output[Chassis_Motor_3508_YH], 
      output[Chassis_Motor_3508_YQ]);
}

void Rising_Motor_SendControl_DJI(DJI_motor_t *DJMotor, int16_t output[]) 
{
  Motor_DJI_Refresh(DJMotor);
  set_motor_parameter(
    DJMotor, 
    output[Rising_Motor_3508_Left],
    output[Rising_Motor_3508_Right], 
    0, 0);
}

void Rising_Motor_SendControl_DM(DM_motor_t *DMMotor_L,DM_motor_t *DMMotor_R, int16_t output_L ,int16_t output_R)
{
  Motor_DM_Refresh(DMMotor_L);
  Motor_DM_Refresh(DMMotor_R);
  MIT_CtrlMotorDM(DMMotor_L, 0, 0, 0, 0, output_L);
  MIT_CtrlMotorDM(DMMotor_R, 0, 0, 0, 0, output_R);
}

void Chassis_Motor_TargetVelocity(float32_t Target_Velocity[],rc_info_t remoter) 
{
  basic_vector_t motion;
  motion.x = map(remoter.ch2, 
    -Remoter_CHMAX, 
    Remoter_CHMAX, 
    -Max_Velocity,
    Max_Velocity);
  motion.y = map(remoter.ch1, 
    -Remoter_CHMAX, 
    Remoter_CHMAX, 
    -Max_Velocity,
    Max_Velocity);
  motion.wz = Turning_Forward_Feedback * map(remoter.ch3, 
    -Remoter_CHMAX, 
    Remoter_CHMAX,                    
    -Max_Velocity, 
    Max_Velocity);

  omni_mecanum_kinematics(&motion, Target_Velocity);
}

void Rising_Motor_TargetVelocity(float32_t Target_Velocity[],rc_info_t remoter) 
{
  float32_t Velocity =
      map(remoter.ch2, 
        -Max_Rising_DM_angle, 
        Max_Rising_DM_angle,
        -Max_Rising_Motor_Velocity, 
        Max_Rising_Motor_Velocity);

  Target_Velocity[Rising_Motor_3508_Left] = Velocity;
  Target_Velocity[Rising_Motor_3508_Right] = -Velocity;
}

void Rising_Motor_TargetAngle(float32_t *Target_Angle_L,float32_t *Target_Angle_R,rc_info_t remoter) 
{
  float32_t ch4 = (float32_t)remoter.ch4;
  if (ch4 < 0.0f) {
    ch4 = 0.0f;
  }
  if (ch4 > 660.0f) {
    ch4 = 660.0f;
  }

  float32_t Angle =
      map(ch4,
        0.0f,
        Remoter_CHMAX,
        0.0f,
        Max_Rising_DM_angle);

  *Target_Angle_L = Angle;
  *Target_Angle_R = -Angle;
}

void Chassis_3508_PID_Init(pid_type_def pid[]) 
{
  for (int i = 0; i < 4; i++) 
  {
    PID_Init(pid + i, 
      Chassis_3508_PID_kp, 
      Chassis_3508_PID_ki,
      Chassis_3508_PID_kd, Chassis_3508_PID_Maxout,
      Chassis_3508_PID_Maxiout);
  }
}

void Rising_3508_PID_Init(pid_type_def pid[]) 
{
  for (int i = 0; i < 2; i++) 
  {
    PID_Init(pid + i, Rising_3508_PID_kp, 
      Rising_3508_PID_ki,
      Rising_3508_PID_kd, 
      Rising_3508_PID_Maxout,
      Rising_3508_PID_Maxiout);
  }
}

void Rising_DM_PID_Init(pid_type_def *pid_L,pid_type_def *pid_R)
{
  PID_Init(pid_L, Rising_DM_PID_kp, 
      Rising_DM_PID_ki,
      Rising_DM_PID_kd, 
      Rising_DM_PID_Maxout,
      Rising_DM_PID_Maxiout);
  PID_Init(pid_R, Rising_DM_PID_kp, 
      Rising_DM_PID_ki,
      Rising_DM_PID_kd, 
      Rising_DM_PID_Maxout,
      Rising_DM_PID_Maxiout);
}

void Chassis_3508_PID_Calculate(pid_type_def pid[], float32_t target_speed[],
                                DJI_motor_t *motor, int16_t output[]) {
  float32_t curren_wheel_speed[4];
  for (int i = 0; i < 4; i++) 
  {
    curren_wheel_speed[i] = motor->motor_msg[i].motor_speed * (Motor_Wheel_Trans);
    output[i] = (int16_t)(PID_Calc_Add(pid + i, curren_wheel_speed[i],*(target_speed + i)));
  }
}

void Rising_3508_PID_Calculate(pid_type_def pid[], float32_t target_speed[],DJI_motor_t *motor, int16_t output[]) 
{
  float32_t curren_wheel_speed[2];
  for (int i = 0; i < 2; i++) 
  {
    curren_wheel_speed[i] = motor->motor_msg[i].motor_speed * (Motor_Wheel_Trans);
    output[i] = (int16_t)(PID_Calc_Add(pid + i,curren_wheel_speed[i],*(target_speed + i)));
  }
}

void Rising_DM_PID_Calculate(pid_type_def *pid_L,pid_type_def *pid_R, float32_t target_angle_L,float32_t target_angle_R,DM_motor_t *motor_L,DM_motor_t *motor_R, float32_t *output_L,float32_t *output_R) 
{
  float32_t current_L = Double_Encoder_Tool(motor_L->motor_msg.motor_angle);
  float32_t current_R = Double_Encoder_Tool(motor_R->motor_msg.motor_angle);
  
  *output_L = PID_Calc_Pos(pid_L, current_L, target_angle_L);
  *output_R = PID_Calc_Pos(pid_R, current_R, target_angle_R);
}


float32_t Double_Encoder_Tool(float32_t motor_angle)
{
  // 对25取模，消除套圈导致的 n*25 偏移
  float32_t result = fmodf(motor_angle, 25.0f);
  if (result < 0.0f) {
    result += 25.0f;  // 保证结果为正 [0, 25)
  }
  // 归化到 [-12.5, +12.5)
  if (result >= 12.5f) {
    result -= 25.0f;
  }
  return result;
}
