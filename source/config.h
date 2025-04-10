#ifndef CONFIG_H
#define CONFIG_H

#define VISION_VERSION 1 // 视觉版本号

#define text1 0 // 拨弹设计测试开关

#define conturl 0 // 遥控器紧急断电控制

#define AIM_MOD 3 // 瞄准模式

#define radar_MOD 1 // 雷达模式 0:无雷达 1:雷达

#define IMU_YAW_OFFSET_CORRECTION (360.0f / 292.0f) // IMU yaw轴偏校准

// 车辆弹道解算参数默认配置
#define BALLISTIC_SOLVER 2 // 弹道解算器  1->弹道解算器v1.0  2->弹道解算器v2.0
#define BULLET_TYPE 0   // 子弹类型 0->17mm 1->42mm
#define BULLET_SPEED 17 // 子弹速度
#define BULLET_K 0.092  // 弹道系数
#define BIAS_TIME 10.0   // 弹道补偿时间(偏置时间)
#define S_BIAS 0.085f      // 枪口前推的时间
#define Z_BIAS 0.265f      // yaw电机到枪口水平面的垂直距离
#define G_NUM 9.80665f       // 重力加速度
#define MAX_ITER 20      // 最大迭代次数
#define TOLERANCE 0.005f    // 收敛阈值(5mm)
#define TIME_STEP 0.001f  // 弹道模拟时间步长(v2.0)
#define ESTIMATED_FLIGHT_TIME 0.5f  // 预估的飞行时间 (s)


#endif // !CONFIG_H