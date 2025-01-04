#include <stdio.h>
#include <math.h>

void calculate_joint_angles(double x, double y, double L1, double L2, double *angle1, double *angle2) {
    // 计算关节1的角度
    // 使用余弦定理
    double c = sqrt(x*x + y*y);
    *angle1 = acos((L1*L1 + c*c - L2*L2) / (2 * L1 * c));
    
    // 计算关节2的角度
    // 使用正弦定理
    *angle2 = asin(y / c) - asin(L2 * sin(*angle1) / L1);
}


