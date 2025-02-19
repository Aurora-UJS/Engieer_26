#ifndef _RADARTOOL_H_
#define _RADARTOOL_H_
#include "main.h"
#include "stdint.h"
#include "stdbool.h"
#include "radarTool.h"
#include "computer_pack.h"


#define RADAR_BUFFER_SIZE 100


#define CRC_Check_False   0
#define CRC_Check_True    1
typedef unsigned char u8;

extern uint8_t rader_rx_buffer[RADAR_BUFFER_SIZE];
extern SendPacketTwist radarTwist; 
extern ReceivedPacketVision_t radarVisionData;
void receiveRadarData(uint8_t *rx_buffer);
#endif // !_RADARTOOL_H_



