#ifndef _RADARTOOL_H_
#define _RADARTOOL_H_
#include "main.h"
#include "stdint.h"
#include "stdbool.h"
#include "radarTool.h"



#define RADAR_BUFFER_SIZE 100

#pragma pack(1)
typedef struct 
{
  uint8_t header;
  uint8_t detect_color : 1;  // 0-red 1-blue
  bool reset_tracker : 1;
  uint8_t reserved : 6;
  float roll;
  float pitch;
  float yaw;
  float aim_x;
  float aim_y;
  float aim_z;
  uint16_t checksum;
}ReceivePacketVision;


typedef struct 
{
  uint8_t header;
  float linear_x;
  float linear_y;
  float linear_z;
  float angular_x;
  float angular_y;
  float angular_z;
  uint16_t checksum;
}SendPacketTwist;


typedef struct
{
  uint8_t header;
  bool tracking : 1;
  uint8_t id : 3;          // 0-outpost 6-guard 7-base
  uint8_t armors_num : 3;  // 2-balance 3-outpost 4-normal
  uint8_t reserved : 1;
  float x;
  float y;
  float z;
  float yaw;
  float vx;
  float vy;
  float vz;
  float v_yaw;
  float r1;
  float r2;
  float dz;
  uint16_t checksum;
}SendPacketVision;
#pragma pack()

#define CRC_Check_False   0
#define CRC_Check_True    1
typedef unsigned char u8;

extern uint8_t rader_rx_buffer[RADAR_BUFFER_SIZE];
extern SendPacketTwist radarTwist; 
extern SendPacketVision radarVisionData;
extern ReceivePacketVision radarVision;
void sendRadarData(void);
void receiveRadarData(uint8_t *rx_buffer);
#endif // !_RADARTOOL_H_



