#include "referee_api.h"
#include "uart_api.h"

/*===========================================================================*/
/*                         环形缓冲区通用实现                                  */
/*===========================================================================*/

#define RING_BUF_SIZE 1024  // 环形缓冲区大小（足够容纳多帧数据）

typedef struct {
	uint8_t buf[RING_BUF_SIZE];
	volatile uint32_t head;  // 写入位置
	volatile uint32_t tail;  // 读取位置
} ring_buffer_t;

static void ring_init(ring_buffer_t *rb)
{
	rb->head = 0;
	rb->tail = 0;
	memset(rb->buf, 0, sizeof(rb->buf));
}

static void ring_write(ring_buffer_t *rb, const uint8_t *data, uint32_t len)
{
	for (uint32_t i = 0; i < len; i++) {
		uint32_t next_head = (rb->head + 1) % RING_BUF_SIZE;
		if (next_head == rb->tail) {
			// 缓冲区已满，丢弃最旧数据保证新数据写入
			rb->tail = (rb->tail + 1) % RING_BUF_SIZE;
		}
		rb->buf[rb->head] = data[i];
		rb->head = next_head;
	}
}

static uint32_t ring_available(ring_buffer_t *rb)
{
	if (rb->head >= rb->tail) {
		return rb->head - rb->tail;
	} else {
		return RING_BUF_SIZE - rb->tail + rb->head;
	}
}

static uint8_t ring_peek(ring_buffer_t *rb, uint32_t offset)
{
	return rb->buf[(rb->tail + offset) % RING_BUF_SIZE];
}

static void ring_discard(ring_buffer_t *rb, uint32_t len)
{
	rb->tail = (rb->tail + len) % RING_BUF_SIZE;
}

static void ring_read(ring_buffer_t *rb, uint8_t *dest, uint32_t len)
{
	for (uint32_t i = 0; i < len; i++) {
		dest[i] = ring_peek(rb, i);
	}
	ring_discard(rb, len);
}

/*===========================================================================*/
/*                         裁判系统 - 环形缓冲区方案                           */
/*===========================================================================*/

static uart_rx_t server_recieve_data;
static uart_msg_t server_rx_msg;
static uint8_t server_rx_data[RE_RX_BUFFER_SIZE]; // DMA接收缓冲区
static ring_buffer_t server_ring;                  // 环形缓冲区
static uint8_t server_frame_buf[RE_RX_BUFFER_SIZE]; // 帧解析缓冲区

static void server_rx_callback(uint8_t *pData, uint32_t size)
{
	if (size > 0) {
		ring_write(&server_ring, pData, size);
	}
}

/** 
 * @brief 初始化裁判模块的UART接收配置
 * @param huart 指向UART句柄的指针
 */
void referee_init(UART_HandleTypeDef *huart)
{
	ring_init(&server_ring);
	
	server_recieve_data.rx_msg = &server_rx_msg;
	server_recieve_data.rx_msg->huart = huart;
	server_recieve_data.rx_msg->pBuffer = server_rx_data;
	server_recieve_data.rx_msg->Len = sizeof(server_rx_data);

	uart_rx_init(&server_recieve_data);
	uart_rx_hook_reg(&server_recieve_data, server_rx_callback);
}

/*===========================================================================*/
/*                    自定义控制器 - 环形缓冲区方案                            */
/*===========================================================================*/

static uart_rx_t ctrller_recieve_data;
static uart_msg_t ctrller_rx_msg;
static uint8_t ctrller_rx_data[RE_RX_BUFFER_SIZE]; // DMA接收缓冲区
static ring_buffer_t ctrller_ring;                  // 环形缓冲区
static uint8_t ctrller_frame_buf[RE_RX_BUFFER_SIZE]; // 帧解析缓冲区

uint8_t Ctrller_Receive_Buffer[256];

static void ctrller_rx_callback(uint8_t *pData, uint32_t size)
{
		if (size > 0) {
			ring_write(&ctrller_ring, pData, size);
			}
}

/** 
 * @brief 初始化控制器模块的UART接收配置
 * @param huart 指向UART句柄的指针
 */
void ctrller_init(UART_HandleTypeDef *huart)
{
	ring_init(&ctrller_ring);
	
	ctrller_recieve_data.rx_msg = &ctrller_rx_msg;
	ctrller_recieve_data.rx_msg->huart = huart;
	ctrller_recieve_data.rx_msg->pBuffer = ctrller_rx_data;
	ctrller_recieve_data.rx_msg->Len = sizeof(ctrller_rx_data);
  
	uart_rx_init(&ctrller_recieve_data);
	uart_rx_hook_reg(&ctrller_recieve_data, ctrller_rx_callback);
}

static referee_info_t referee_info;

/**
 * @brief 解析单帧裁判数据（内部使用）
 */
static void referee_parse_frame(uint8_t *buff, uint16_t len)
{
	(void)len;
	memcpy(&referee_info.FrameHeader, buff, LEN_HEADER);
	referee_info.CmdID = (uint16_t)(buff[6] << 8) | buff[5];
	
	switch (referee_info.CmdID)
	{
	case ID_game_state:
		memcpy(&referee_info.GameState, (buff + DATA_Offset), LEN_game_state);
		break;
	case ID_game_result:
		memcpy(&referee_info.GameResult, (buff + DATA_Offset), LEN_game_result);
		break;
	case ID_game_robot_survivors:
		memcpy(&referee_info.GameRobotHP, (buff + DATA_Offset), LEN_game_robot_HP);
		break;
	case ID_event_data:
		memcpy(&referee_info.EventData, (buff + DATA_Offset), LEN_event_data);
		break;
	case ID_supply_projectile_action:
		memcpy(&referee_info.SupplyProjectileAction, (buff + DATA_Offset), LEN_supply_projectile_action);
		break;
	case ID_game_robot_state:
		memcpy(&referee_info.GameRobotState, (buff + DATA_Offset), LEN_game_robot_state);
		break;
	case ID_power_heat_data:
		memcpy(&referee_info.PowerHeatData, (buff + DATA_Offset), LEN_power_heat_data);
		break;
	case ID_game_robot_pos:
		memcpy(&referee_info.GameRobotPos, (buff + DATA_Offset), LEN_game_robot_pos);
		break;
	case ID_buff_musk:
		memcpy(&referee_info.BuffMusk, (buff + DATA_Offset), LEN_buff_musk);
		break;
	case ID_aerial_robot_energy:
		memcpy(&referee_info.AerialRobotEnergy, (buff + DATA_Offset), LEN_aerial_robot_energy);
		break;
	case ID_robot_hurt:
		memcpy(&referee_info.RobotHurt, (buff + DATA_Offset), LEN_robot_hurt);
		break;
	case ID_shoot_data:
		memcpy(&referee_info.ShootData, (buff + DATA_Offset), LEN_shoot_data);
		break;
	case ID_student_interactive:
		memcpy(&referee_info.ReceiveData, (buff + DATA_Offset), LEN_receive_data);
		break;
	default:
		break;
	}
}

/**
 * @brief 从裁判系统环形缓冲区中搜索并解析完整帧
 */
static int referee_process_ring_buffer(void)
{
	int frames_parsed = 0;
	
	while (ring_available(&server_ring) >= LEN_HEADER) {
		uint32_t available = ring_available(&server_ring);
		uint32_t sof_offset = 0;
		uint8_t found_sof = 0;
		
		for (sof_offset = 0; sof_offset < available; sof_offset++) {
			if (ring_peek(&server_ring, sof_offset) == REFEREE_SOF) {
				found_sof = 1;
				break;
			}
		}
		
		if (sof_offset > 0) {
			ring_discard(&server_ring, sof_offset);
		}
		
		if (!found_sof) break;
		if (ring_available(&server_ring) < LEN_HEADER) break;
		
		uint8_t header[LEN_HEADER];
		for (int i = 0; i < LEN_HEADER; i++) {
			header[i] = ring_peek(&server_ring, i);
		}
		
		if (Verify_CRC8_Check_Sum(header, LEN_HEADER) != CRC_Check_True) {
			ring_discard(&server_ring, 1);
			continue;
		}
		
		uint16_t data_length = (uint16_t)(header[2] << 8) | header[1];
		uint16_t frame_length = LEN_HEADER + LEN_CMDID + data_length + LEN_TAIL;
		
		if (frame_length > RE_RX_BUFFER_SIZE) {
			ring_discard(&server_ring, 1);
			continue;
		}
		
		if (ring_available(&server_ring) < frame_length) break;
		
		ring_read(&server_ring, server_frame_buf, frame_length);
		
		if (Verify_CRC16_Check_Sum(server_frame_buf, frame_length) == CRC_Check_True) {
			referee_parse_frame(server_frame_buf, frame_length);
			frames_parsed++;
		}
	}
	
	return frames_parsed;
}

/** 
 * @brief 解析裁判数据包（兼容旧接口）
 */
void JudgeReadData(uint8_t *buff)
{
	(void)buff;
	referee_process_ring_buffer();
}

static custom_controller_info_t custom_controller_info;

uint8_t CtrllerData[CtrllerData_Length] = {
    '3','1','4','1',
    '3','1','4','1',
    '3','1','4','1',
    '3','1','4','1',
    '3','1','4','1',
    '3','1','4','1',
	'0','0'
};
keyboard_t kb_info;

/**
 * @brief 解析单帧控制器数据（内部使用）
 * @param buff 完整的帧数据
 * @param len 帧长度
 */
static void ctrller_parse_frame(uint8_t *buff, uint16_t len)
{
	// 调试用：复制帧数据
	memcpy(Ctrller_Receive_Buffer, buff, len < 256 ? len : 256);
	memcpy(CtrllerData, Ctrller_Receive_Buffer + 7, 26);
	// 写入帧头数据
	memcpy(&custom_controller_info.FrameHeader, buff, LEN_HEADER);
	uint16_t data_length = (uint16_t)(buff[2] << 8) | buff[1];
	
	// 提取 CmdID (小端序)
	custom_controller_info.CmdID = (uint16_t)(buff[6] << 8) | buff[5];
	
	// 根据 CmdID 解析数据
	switch (custom_controller_info.CmdID)
	{
	case 0x0302:  // 自定义控制器数据
		if (data_length >= (uint16_t)sizeof(custom_controller_info.CustomController)) {
			memcpy(&custom_controller_info.CustomController, (buff + DATA_Offset), sizeof(custom_controller_info.CustomController));
		}
		break;
	case 0x0304:  // 键鼠数据
		if (data_length >= LEN_keyboard) {
			memcpy(&custom_controller_info.keyboard, (buff + DATA_Offset), LEN_keyboard);
		}
		kb_info = custom_controller_info.keyboard;
		break;
	default:
		break;
	}
}

/**
 * @brief 从环形缓冲区中搜索并解析完整帧
 * @return 成功解析的帧数
 */
static int ctrller_process_ring_buffer(void)
{
	int frames_parsed = 0;
	
	while (ring_available(&ctrller_ring) >= LEN_HEADER) {
		uint32_t available = ring_available(&ctrller_ring);
		uint32_t sof_offset = 0;
		uint8_t found_sof = 0;
		
		for (sof_offset = 0; sof_offset < available; sof_offset++) {
			if (ring_peek(&ctrller_ring, sof_offset) == REFEREE_SOF) {
				found_sof = 1;
				break;
			}
		}
		
		if (sof_offset > 0) {
			ring_discard(&ctrller_ring, sof_offset);
		}
		
		if (!found_sof) break;
		if (ring_available(&ctrller_ring) < LEN_HEADER) break;
		
		uint8_t header[LEN_HEADER];
		for (int i = 0; i < LEN_HEADER; i++) {
			header[i] = ring_peek(&ctrller_ring, i);
		}
		
		if (Verify_CRC8_Check_Sum(header, LEN_HEADER) != CRC_Check_True) {
			ring_discard(&ctrller_ring, 1);
			continue;
		}
		
		uint16_t data_length = (uint16_t)(header[2] << 8) | header[1];
		uint16_t frame_length = LEN_HEADER + LEN_CMDID + data_length + LEN_TAIL;
		
		if (frame_length > RE_RX_BUFFER_SIZE) {
			ring_discard(&ctrller_ring, 1);
			continue;
		}
		
		if (ring_available(&ctrller_ring) < frame_length) break;
		
		ring_read(&ctrller_ring, ctrller_frame_buf, frame_length);
		
		if (Verify_CRC16_Check_Sum(ctrller_frame_buf, frame_length) == CRC_Check_True) {
			ctrller_parse_frame(ctrller_frame_buf, frame_length);
			frames_parsed++;
		}
	}
	
	return frames_parsed;
}

/** 
 * @brief 解析控制器数据包（兼容旧接口）
 * @param buff 输入的数据缓冲区指针（不再使用，保留接口兼容）
 */
void CtrllerReadData(uint8_t *buff)
{
	(void)buff;  // 不再使用此参数
	ctrller_process_ring_buffer();
}

/** 
 * @brief 获取解析后的裁判信息数据
 * @return 当前裁判数据结构体
 */
referee_info_t *get_referee_msg(void)
{
	referee_process_ring_buffer();
	return &referee_info;
}

/** 
 * @brief 获取解析后的自定义控制器信息数据
 * @return 当前自定义控制器数据结构体
 */
custom_controller_info_t *get_custom_controller_msg(void)
{
	// 从环形缓冲区解析所有可用帧
	ctrller_process_ring_buffer();
	return &custom_controller_info;
}

/** 
 * @brief 将数据打包并通过UART发送到底层设备
 * @param sof 帧头标识符
 * @param cmd_id 数据命令ID
 * @param p_data 待发送数据的指针
 * @param len 数据长度
 * @return uint8_t 无意义返回值（函数未使用返回值）
 */
uint8_t seq = 0;/*sequence初始化*/
void referee_data_pack_handle(uint8_t sof, uint16_t cmd_id, uint8_t *p_data, uint16_t len)//英步usart1 工程uart10 常规链路 ui发送
{
	unsigned char i = 0;
	uint8_t tx_buff[MAX_SIZE];
	uint16_t frame_length = frameheader_len/*5*/ + cmd_len/*2*/ + len/*函参*/ + crc_len/*2*/;
	memset(tx_buff, 0, frame_length);  //将数组tx_buff中长度为“frame_length”的空间赋值为0
	tx_buff[0] = sof/*函参*/;
	memcpy(&tx_buff[1], (uint8_t *)&len, sizeof(len));
	tx_buff[3] = seq;
	Append_CRC8_Check_Sum(tx_buff, frameheader_len);
	memcpy(&tx_buff[frameheader_len], (uint8_t *)&cmd_id, cmd_len);
	memcpy(&tx_buff[frameheader_len + cmd_len], p_data, len);
	Append_CRC16_Check_Sum(tx_buff, frame_length);
	if (seq == 0xff) seq = 0;
	else seq++;/*sequence循环*/
	for (i = 0;i < frame_length;i++)
	{
		while (HAL_UART_Transmit_IT(server_recieve_data.rx_msg->huart,&tx_buff[i],sizeof(tx_buff[i])) == HAL_BUSY);		
	}
}
