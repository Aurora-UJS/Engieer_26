#include "PIDtool.h"
#include "chassis_config.h"
#include "stm32h7xx.h" 
#include "motor_DJI.h"
#include "PIDtool.h"
#include "chassismovement.h"  // 假设头文件已经正确包含

// 定义电机编号
#define MOTOR_COUNT 4

// 定义 PID 参数数组
static const float Chassis_Motor_Speed_Kp[MOTOR_COUNT] = {Chassis1_Motor_Speed_Kp, Chassis2_Motor_Speed_Kp, Chassis3_Motor_Speed_Kp, Chassis4_Motor_Speed_Kp};
static const float Chassis_Motor_Speed_Ki[MOTOR_COUNT] = {Chassis1_Motor_Speed_Ki, Chassis2_Motor_Speed_Ki, Chassis3_Motor_Speed_Ki, Chassis4_Motor_Speed_Ki};
static const float Chassis_Motor_Speed_Kd[MOTOR_COUNT] = {Chassis1_Motor_Speed_Kd, Chassis2_Motor_Speed_Kd, Chassis3_Motor_Speed_Kd, Chassis4_Motor_Speed_Kd};
static const float Chassis_Motor_Speed_Maxout[MOTOR_COUNT] = {Chassis1_Motor_Speed_Maxout, Chassis2_Motor_Speed_Maxout, Chassis3_Motor_Speed_Maxout, Chassis4_Motor_Speed_Maxout};
static const float Chassis_Motor_Speed_IMaxout[MOTOR_COUNT] = {Chassis1_Motor_Speed_IMaxout, Chassis2_Motor_Speed_IMaxout, Chassis3_Motor_Speed_IMaxout, Chassis4_Motor_Speed_IMaxout};

pid_type_def pid_chassis[4];
int16_t v1,v2,v3,v4 ;
DJI_motor_t *DJMotor_chassis;


void DJI_motor_init()
{
    // 动态分配内存，以创建一个 DJI_motor_t 类型的实例，并将其地址赋给 DJMotor_chassis
    DJMotor_chassis = pvPortMalloc(sizeof(DJI_motor_t));

    if (DJMotor_chassis == NULL) {
        // 处理内存分配失败的情况
        return;
    }

    // 配置 DJMotor 的 CAN 端口，设置为 CAN1_PORT
    DJMotor_chassis->can_cfg.port = CAN1_PORT;

    // 配置 DJMotor 的 CAN 通信 ID，设置为 0x200
    DJMotor_chassis->can_cfg.id = 0x200;

    // 配置电机消息的 CAN ID，依次设置为 0x201, 0x202, 0x203, 0x204
    for (int i = 0; i < 4; i++) {
         DJMotor_chassis->motor_msg[i].can_msg.id = 0x201 + i;

        }
     Motor_DJI_Init(DJMotor_chassis);
}





void chassis_pid_init(void)
{
    for (int i = 0; i < MOTOR_COUNT; ++i) {
        PID_Init(&pid_chassis[i], 
                 Chassis_Motor_Speed_Kp[i], 
                 Chassis_Motor_Speed_Ki[i], 
                 Chassis_Motor_Speed_Kd[i], 
                 Chassis_Motor_Speed_Maxout[i],
                 Chassis_Motor_Speed_IMaxout[i]);
    }
}




void chassis_move(void)
{
    float motor_voltages[MOTOR_COUNT];

    for (int i = 0; i < MOTOR_COUNT; ++i) {
        motor_voltages[i] = PID_Calc_Pos(
            &pid_chassis[i],
            DJMotor_chassis->motor_msg[i].motor_speed * Multiple,
            MOTOR_speed_out[i] * Multiple
        );
    }

    set_motor_voltage(DJMotor_chassis, motor_voltages[0], motor_voltages[1], motor_voltages[2], motor_voltages[3]);
}


