#ifndef VISION_RECEIVE_H
#define VISION_RECEIVE_H

#include "computer_pack.h"

extern ReceivedPacketVision_t ReceivedPacketVision; // 保存接收到的视觉数据

/**
 * @brief 获取视觉数据包的ID值
 * 
 * 该宏用于快速访问ReceivedPacketVision结构体中的id字段，通常用于标识当前接收到的
 * 视觉数据包编号或类型。宏展开后直接返回结构体的整型id成员值。
 * 
 * @return int 视觉数据包标识符，具体取值范围由协议定义
 */
#define get_vision_id() (ReceivedPacketVision.id)

/**
 * @brief 检测视觉跟踪状态
 * 
 * 该宏用于快速获取目标跟踪状态，通过访问ReceivedPacketVision结构体的tracking字段
 * 判断当前视觉系统是否处于有效跟踪状态。宏展开后返回结构体的布尔型tracking成员值。
 * 
 * @return bool 跟踪状态标识，true表示正在跟踪，false表示丢失目标
 */
#define is_tracking() (ReceivedPacketVision.tracking)

void vision_receive_solve(uint8_t *rx_buf);

#endif // !VISION_RECEIVE_H