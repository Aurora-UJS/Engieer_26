#ifndef VISION_RECEIVE_H
#define VISION_RECEIVE_H

#include "computer_pack.h"

extern ReceivedPacketVision_t ReceivedPacketVision; // 保存接收到的视觉数据

void vision_receive_solve(uint8_t *rx_buf);

#endif // !VISION_RECEIVE_H