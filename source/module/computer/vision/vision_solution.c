/* vision_solution.c - 弹道解算核心算法模块 */

#include "vision_solution.h"
#include "uart_api.h"
#include "vision_receive.h"
#include "config.h"
#include "arm_math.h"

/* 全局变量声明 */
float aim_x_n = 0, aim_y_n = 0, aim_z_n = 0; // 归一化目标坐标（输出到发送模块）

/* 装甲板位置结构体 */
static struct tar_pos {
    float x, y, z;     // 三维坐标（世界坐标系）
    float yaw;         // 偏航角
} tar_position[4];     // 四个装甲板位置缓存

/* 弹道解算参数配置 */
SolveTrajectoryParams_t SolveTrajectoryParams = {
    .k = BULLET_K,          // 空气阻力系数
    .bullet_type = BULLET_TYPE, // 弹丸类型
    .current_v = BULLET_SPEED, // 初速度（m/s）
    .bias_time = BIAS_TIME, // 系统时延补偿（ms）
    .s_bias = S_BIAS,       // 水平偏移补偿
    .z_bias = Z_BIAS,       // 高度偏移补偿
};

/**
  * @brief 弹道轨迹模拟（二维欧拉法）
  * @param v0 初速度 m/s
  * @param theta 发射仰角 rad
  * @param k 阻力系数
  * @param z_target 目标高度 m
  * @param flight_time 返回飞行时间 s
  * @return 水平射程 m
  * @note 使用改进欧拉法+线性插值提升精度
  */
float simulate_trajectory(float v0, float theta, float k, float z_target, float *flight_time)
{
    /* 模拟参数配置 */
    const float dt = 0.001f;     // 时间步长 (s)
    float x = 0.0f, z = 0.0f;    // 初始位置
    float vx = v0 * cosf(theta); // 分解水平速度
    float vz = v0 * sinf(theta); // 分解垂直速度
    float t = 0.0f;              // 时间累计

    /* 弹道模拟循环 */
    while (z >= -z_target) {     // 注意坐标系方向约定
        float v = sqrtf(vx*vx + vz*vz); // 合速度计算
        float ax = -k * v * vx;  // 空气阻力加速度（水平）
        float az = -k * v * vz - G_NUM; // 空气阻力+重力加速度（垂直）

        /* 欧拉法迭代 */
        vx += ax * dt;
        vz += az * dt;
        x += vx * dt;
        z += vz * dt;
        t += dt;

        if (z < -z_target) break; // 精确终止条件
    }

    /* 线性插值修正 */
    float delta_t = (-z_target - (z - vz*dt)) / vz; // 剩余时间估算
    x += vx * delta_t;  // 精确水平位移
    t += delta_t;       // 精确飞行时间

    *flight_time = t;
    return x;
}

/**
  * @brief 俯仰角二分法优化
  * @param target_x 目标水平距离 m
  * @param z_target 目标高度 m
  * @param params 弹道参数
  * @return 最优发射仰角 rad
  * @note 使用二分搜索法寻找最小误差角度
  */
float optimize_pitch(float target_x, float z_target, SolveTrajectoryParams_t params)
{
    float theta_low = 0.0f;      // 搜索下界
    float theta_high = PI / 2;   // 搜索上界
    float best_theta = 0.0f;     // 最优解

    for (int i = 0; i < MAX_ITER; i++) {
        float theta = (theta_low + theta_high) / 2;
        float flight_time;
        float x = simulate_trajectory(params.current_v, theta, params.k, z_target, &flight_time);
        float error = fabsf(x - target_x);

        if (error < TOLERANCE) { // 达到精度要求
            best_theta = theta;
            break;
        }

        /* 二分搜索逻辑 */
        (x < target_x) ? (theta_low = theta) : (theta_high = theta);
        best_theta = theta; // 记录当前最优
    }
    return best_theta;
}

/**
  * @brief 弹道解算主函数
  * @param aim_x 目标x坐标 m
  * @param aim_y 目标y坐标 m
  * @param aim_z 目标z坐标 m
  * @param params 弹道参数
  * @param pitch 输出俯仰角 rad
  * @param yaw 输出偏航角 rad
  */
void solve_trajectory(float aim_x, float aim_y, float aim_z, 
                     SolveTrajectoryParams_t params, float *pitch, float *yaw)
{
    float L = sqrtf(aim_x*aim_x + aim_y*aim_y); // 水平投影距离
    float H = aim_z - params.z_bias;            // 高度差补偿
    
    *pitch = optimize_pitch(L, H, params); // 计算俯仰角
    *yaw = atan2f(aim_y, aim_x);           // 计算偏航角
}

/**
  * @brief 自动目标预测与解算
  * @param pitch 当前云台俯仰角 rad
  * @param yaw 当前云台偏航角 rad
  * @param aim_x 输出预测x坐标
  * @param aim_y 输出预测y坐标
  * @param aim_z 输出预测z坐标
  * @note 包含运动预测和装甲板选择逻辑
  */
void autoSolveTrajectory(float *pitch, float *yaw, float *aim_x, float *aim_y, float *aim_z)
{
    /* 时间延迟补偿 = 系统延迟 + 预估飞行时间 */
    float timeDelay = SolveTrajectoryParams.bias_time/1000.0f + ESTIMATED_FLIGHT_TIME;
    
    /* 更新目标运动状态 */
    SolveTrajectoryParams.tar_yaw += SolveTrajectoryParams.v_yaw * timeDelay;

    /* 计算四个装甲板预测位置 */
    for (int i = 0; i < 4; i++) {
        float tmp_yaw = SolveTrajectoryParams.tar_yaw + i*PI/2.0f; // 装甲板间隔90度
        float r = (i%2 == 0) ? SolveTrajectoryParams.r1 : SolveTrajectoryParams.r2;
        
        tar_position[i].x = SolveTrajectoryParams.xw - r*cosf(tmp_yaw);
        tar_position[i].y = SolveTrajectoryParams.yw - r*sinf(tmp_yaw);
        tar_position[i].z = (i<2) ? SolveTrajectoryParams.zw : SolveTrajectoryParams.zw + SolveTrajectoryParams.dz;
        tar_position[i].yaw = tmp_yaw;
    }

    /* 选择最接近当前瞄准角度的装甲板 */
    int idx = 0;
    float min_diff = fabsf(*yaw - tar_position[0].yaw);
    for (int i=1; i<4; i++) {
        float diff = fabsf(*yaw - tar_position[i].yaw);
        if (diff < min_diff) {
            min_diff = diff;
            idx = i;
        }
    }

    /* 预测目标位置 */
    *aim_z = tar_position[idx].z + SolveTrajectoryParams.vzw * timeDelay;
    *aim_x = tar_position[idx].x + SolveTrajectoryParams.vxw * timeDelay;
    *aim_y = tar_position[idx].y + SolveTrajectoryParams.vyw * timeDelay;

    /* 执行弹道解算 */
    solve_trajectory(*aim_x, *aim_y, *aim_z, SolveTrajectoryParams, pitch, yaw);
}

/**
  * @brief 视觉解算主入口
  * @param pitch 当前俯仰角（输入）
  * @param yaw 当前偏航角（输入）
  * @param target_yaw 输出目标偏航角
  * @param target_pitch 输出目标俯仰角
  */
void vision_solution(float pitch, float yaw, float *target_yaw, float *target_pitch)
{
    if (ReceivedPacketVision.tracking != 1) return; // 目标丢失保护

    /* 更新当前姿态参数 */
    SolveTrajectoryParams.current_pitch = pitch;
    SolveTrajectoryParams.current_yaw = yaw;

    /* 注入视觉数据 */
    SolveTrajectoryParams.armor_id = ReceivedPacketVision.id;
    SolveTrajectoryParams.armor_num = ReceivedPacketVision.armors_num;
    SolveTrajectoryParams.xw = ReceivedPacketVision.x;
    SolveTrajectoryParams.yw = ReceivedPacketVision.y;
    SolveTrajectoryParams.zw = ReceivedPacketVision.z;
    SolveTrajectoryParams.tar_yaw = ReceivedPacketVision.yaw;
    // ...（其他参数注入同理）

    /* 执行自动解算 */
    autoSolveTrajectory(target_pitch, target_yaw, &aim_x_n, &aim_y_n, &aim_z_n);
}