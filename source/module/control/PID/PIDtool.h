#ifndef PIDTOOL
#define PIDTOOL

typedef struct
{
    //PID 三参数
    float Kp;
    float Ki;
    float Kd;

    float max_out;  //最大输出
    float max_iout; //最大积分输出

    float set;
    float fdb;

    float out;
    float Pout;
    float Iout;
    float Dout;
    float Dbuf[3];  //微分项 0最新 1上一次 2上上次
    float error[3]; //误差项 0最新 1上一次 2上上次

} pid_type_def;

void PID_Init(pid_type_def *pid, const float kp, const float ki, const float kd, float max_out, float max_iout);
float PID_Calc_Add(pid_type_def *pid, float now, float target);
float PID_Calc_Pos(pid_type_def *pid, float now, float target);



#endif // !PIDTOOL