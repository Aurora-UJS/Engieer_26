#include "main.h"
#include "cmsis_os2.h"
#include "DBusSys.h"
#include "motor_DM.h"
#include "motor_DJI.h"
#include "arm.h"
#include "PIDtool.h"
#include "omni_mecanum_kinematics.h"
#include "chassis_config.h"
#include "chassismovement.h"

pid_type_def PID_DJ_motor_2006_speed_Left, PID_DJ_motor_2006_angle_Left;

basic_vector_t motion;
float motor_speed_out[MOTOR_COUNT];
float motor_voltages[MOTOR_COUNT];

void Remoter_Task(void const *argument)
{
    UNUSED(argument);
    osDelay(1500);

    DJI_motor_init();   // 底盘
    chassis_pid_init(); // 底盘

    DM_motor_init();//arm
    DBus_Init();//arm
    motor_Reset();

    // Motor_DJI_Init(DJ_motor_2006_Left);
    // PID_Init(&PID_DJ_motor_2006_speed_Left,
    //          3,
    //          0,
    //          0,
    //          400,
    //          10000.0);

    while (1)
    {

        DBus_Refresh();
        Motor_DM_Refresh(DM_motor_8009_Down); // 刷新电机状态
        Motor_DM_Refresh(DM_motor_8009_Up);   // 刷新电机状态
        Motor_DJI_Refresh(DJ_motor_2006_Left);
        Motor_DJI_Refresh(DJMotor_chassis);

        if (remoter.sw1 == 1 && remoter.sw2 == 1)
        {
            motion.x = (float)(remoter.ch1) / 660.0f * Chassis_RC_Max_Speed;
            motion.y = (float)(remoter.ch2) / 660.0f * Chassis_RC_Max_Speed;
            motion.wz = (float)(remoter.ch3) / 660.0f * Chassis_Shift_Max_Speed * (-1);
            omni_mecanum_kinematics(&motion, motor_speed_out);
            for (int i = 0; i < MOTOR_COUNT; ++i)
            {
                motor_voltages[i] = PID_Calc_Pos(
                    &pid_chassis[i],
                    DJMotor_chassis->motor_msg[i].motor_speed ,
                    motor_speed_out[i] * Multiple);
            }
           set_motor_voltage(DJMotor_chassis, motor_voltages[0], motor_voltages[1], motor_voltages[2], motor_voltages[3]);

        }
        if (remoter.sw1 == 1 && remoter.sw2 == 2)
        {
             set_motor_voltage(DJMotor_chassis, 0, 0, 0, 0);

        }

        if (remoter.sw1 == 2 && remoter.sw2 == 1)
        {
            motor_8009_down_movement();
            motor_8009_up_movement();
            motor_spin_movement();
        }

        if (remoter.sw1 == 2 && remoter.sw2 == 2)
        {
            motor_Zero();
        }

        osDelay(10);
    }
}
