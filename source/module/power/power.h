#ifndef POWER_H
#define POWER_H

typedef struct{

   float initial_total_power;//初始总功率控制
   //uint16_t max_power_limit ;//最大功率限制
   float chassis_max_power; //底盘最大输出功率
   //float other_max_power ; //超级电容的缓冲功率
   float nomal_power;     // 静态功率——此值为车辆在静态时的功率，用于计算车辆在运动时的功率,此处的值要进行更换
   float toque_coefficient; // (20/16384)*(0.3)*(187/3591)/9.55 此值要根据电机参数计算



}power;

#endif // POWER_H

