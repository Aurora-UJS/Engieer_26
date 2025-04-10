#include "IMUtool.h"
#include "main.h"
#include "cmsis_os.h"
#include "BMI088driver.h"
#include "gpio.h"
#include "tim.h"
#include "string.h"
#include "config.h"

static const float DES_TEMP = 40.0f;
static const float KP = 100.0f;
static const float KI = 50.0f;
static const float KD = 10.0f;
static const int MAX_OUT = 500;
static const int correct_Time_define = 1000; // 上电去0飘 1000次取平均
static const float Pi = 3.14159265358979323846;

float out = 0;
float err = 0;

float err_l = 0;
float err_ll = 0;

float gyro[3], accel[3], angles[3], temp;
uint8_t forceStop = 0;
extern osSemaphoreId imuBinarySem01Handle;
bmi088_data_t bmi088_data;
uint32_t temp_Ticks = 0;
uint8_t attitude_flag = 0;
uint32_t correct_times = 0;
float gyro_correct[3] = {0};

/**
 * @brief PID温度控制函数。
 *
 * 该函数实现了一个简单的PID控制器，用于调节IMU模块的温度至目标值（DES_TEMP）。
 * 控制器通过计算当前温度与目标温度之间的误差，调整PWM输出以控制加热功率。
 *
 * 控制公式为：
 *     out = KP * err + KI * (err + err_l + err_ll) + KD * (err - err_l)
 * 其中：
 * - err: 当前温度误差。
 * - err_l: 上一次温度误差。
 * - err_ll: 上上次温度误差。
 * - KP, KI, KD: PID控制器的比例、积分、微分系数。
 *
 * 输出值被限制在 [0, MAX_OUT] 范围内，并通过定时器通道（htim3.Instance->CCR4）设置PWM占空比。
 */
void temple(void)
{
    err_ll = err_l;
    err_l = err;
    err = DES_TEMP - temp;
    out = KP * err + KI * (err + err_l + err_ll) + KD * (err - err_l);
    if (out > MAX_OUT)
        out = MAX_OUT;
    if (out < 0)
        out = 0.f;

    htim3.Instance->CCR4 = (uint16_t)out;
}

void IMUsys(void)
{

    while (osSemaphoreAcquire(imuBinarySem01Handle, osWaitForever) != osOK)
        ;

    BMI088_read(gyro, accel, &temp);

    temple();

    memcpy(&bmi088_data.gyro, gyro, sizeof(gyro));
    memcpy(&bmi088_data.accel, accel, sizeof(accel));
    bmi088_data.temp = temp;

    if ((fabsf(temp - DES_TEMP) < 0.5f) && attitude_flag == 0) // 接近额定温度之差小于0.5° 开始计数
    {
        temp_Ticks++;
        if (temp_Ticks > DES_TEMP) // 计数达到一定次数后 才进入0飘初始化 说明温度已经达到目标
        {
            attitude_flag = 1; // go to correct state
        }
        MahonyAHRSinit(accel[0], accel[1], accel[2], 0, 0, 0);
    }

    // BMI088_read(gyro, accel, &temp);
    if (attitude_flag == 2)
    {
        gyro[0] -= gyro_correct[0]; // 减去陀螺仪0飘
        gyro[1] -= gyro_correct[1];
        gyro[2] -= gyro_correct[2];

#if cheat // 作弊 可以让yaw很稳定 去掉比较小的值
        if (fabsf(gyro[2]) < 0.003f)
            gyro[2] = 0;
#endif

        // //===========================================================================
        // // ekf姿态解算部分
        // // HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13,GPIO_PIN_SET);
        // IMU_QuaternionEKF_Update(gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2]);
        // // HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13,GPIO_PIN_RESET);
        // //===============================================================================
        //=================================================================================
        // mahony姿态解算部分
        // HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13,GPIO_PIN_SET);
        Mahony_update(gyro[0], gyro[1], gyro[2] * (IMU_YAW_OFFSET_CORRECTION), accel[0], accel[1], accel[2], 0, 0, 0);
        Mahony_computeAngles(); // 角度计算
        //=============================================================================
        // 获取姿态角度函数
        // getPitch(); // 获得pitch
        // getRoll();  // 获得roll
        // getYaw();   // 获得yaw
        //==============================================================================
    }
    else if (attitude_flag == 1) // 状态1 开始1000次的陀螺仪0飘初始化
    {
        // gyro correct
        gyro_correct[0] += gyro[0];
        gyro_correct[1] += gyro[1];
        gyro_correct[2] += gyro[2];
        correct_times++;
        if (correct_times >= correct_Time_define)
        {
            gyro_correct[0] /= correct_Time_define;
            gyro_correct[1] /= correct_Time_define;
            gyro_correct[2] /= correct_Time_define;
            attitude_flag = 2; // go to 2 state
        }
    }

    // temperature control
}