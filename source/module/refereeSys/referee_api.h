#ifndef _REFEREE_API_H
#define _REFEREE_API_H

#include "main.h"
#include "referee_protocol.h"
#include "crc_api.h"
#include "string.h"
#include "uart_api.h"
#include "usart.h"

#define CtrllerData_Length 27
#define RE_RX_BUFFER_SIZE 255u // 裁判系统接收缓冲区大小

#define SCREEN_WIDTH 1080
#define SCREEN_LENGTH 1920

#define MAX_SIZE 128
#define frameheader_len 5 // 帧头长度
#define cmd_len 2		  // 命令码长度
#define crc_len 2		  // CRC16校验码长度

typedef struct
{
	// referee_id_t referee_id;
	xFrameHeader FrameHeader; // 接收到的帧头信息
	uint16_t CmdID;
	ext_game_state_t GameState;							   // 0x0001
	ext_game_result_t GameResult;						   // 0x0002
	ext_game_robot_HP_t GameRobotHP;					   // 0x0003
	ext_event_data_t EventData;							   // 0x0101
	ext_supply_projectile_action_t SupplyProjectileAction; // 0x0102
	ext_game_robot_state_t GameRobotState;				   // 0x0201
	ext_power_heat_data_t PowerHeatData;				   // 0x0202
	ext_game_robot_pos_t GameRobotPos;					   // 0x0203
	ext_buff_musk_t BuffMusk;							   // 0x0204
	aerial_robot_energy_t AerialRobotEnergy;			   // 0x0205
	ext_robot_hurt_t RobotHurt;							   // 0x0206
	ext_shoot_data_t ShootData;							   // 0x0207
	// 自定义交互数据的接收
	Communicate_ReceiveData_t ReceiveData;
	uint8_t init_flag;
} referee_info_t;

typedef struct
{
	xFrameHeader FrameHeader; // 接收到的帧头信息
	uint16_t CmdID;
	// 自定义控制器数据  0x0302
	custom_controller_data_t CustomController; // 0x0302
	// 键鼠数据  0x0304
	keyboard_t keyboard;					   // 0x0304
} custom_controller_info_t;

typedef struct
{
	uint8_t uart_init_ok;
	uint32_t rx_event_count;
	uint32_t rx_total_bytes;
	uint16_t rx_last_size;
	uint8_t rx_last_head[8];
	uint32_t frame_parse_count;
	uint32_t crc8_fail_count;
	uint32_t crc16_fail_count;
	uint16_t last_cmd_id;
	uint16_t last_frame_length;
	uint8_t last_robot_id;
} referee_link_debug_t;

void JudgeReadData(uint8_t *buff);
void CtrllerReadData(uint8_t *buff);
referee_info_t *get_referee_msg(void);
custom_controller_info_t *get_custom_controller_msg(void);
void referee_data_pack_handle(uint8_t sof, uint16_t cmd_id, uint8_t *p_data, uint16_t len);
void referee_init(UART_HandleTypeDef *huart);
void ctrller_init(UART_HandleTypeDef *huart);
HAL_StatusTypeDef referee_send_raw_data(const uint8_t *p_data, uint16_t len, uint32_t timeout);
uint8_t referee_is_inited(void);

/* 常规链路调试结构体：
 * 1. 先看 UART1 是否真的有数据进来；
 * 2. 再看 CRC 和帧解析有没有通过；
 * 3. 最后确认 0x0201 是否被解析到，从而拿到 robot_id。
 */
extern volatile referee_link_debug_t g_referee_link_debug;

#endif 
