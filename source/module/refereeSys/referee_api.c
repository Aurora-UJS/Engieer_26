#include "referee_api.h"
#include "uart_api.h"

#include <limits.h>

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

volatile referee_link_debug_t g_referee_link_debug = {0};

static void server_rx_callback(uint8_t *pData, uint32_t size)
{
	if (size > 0) {
		uint32_t copy_size = size;

		g_referee_link_debug.rx_event_count++;
		g_referee_link_debug.rx_total_bytes += size;
		g_referee_link_debug.rx_last_size = (uint16_t)size;

		if (copy_size > sizeof(g_referee_link_debug.rx_last_head)) {
			copy_size = sizeof(g_referee_link_debug.rx_last_head);
		}
		memset((void *)g_referee_link_debug.rx_last_head, 0, sizeof(g_referee_link_debug.rx_last_head));
		memcpy((void *)g_referee_link_debug.rx_last_head, pData, copy_size);

		ring_write(&server_ring, pData, size);
	}
}

uint8_t referee_is_inited(void)
{
	return (((server_recieve_data.rx_msg != NULL) &&
	         (server_recieve_data.rx_msg->huart != NULL) &&
	         (g_referee_link_debug.uart_init_ok != 0U)) ? 1U : 0U);
}

/** 
 * @brief 初始化裁判模块的UART接收配置
 * @param huart 指向UART句柄的指针
 */
void referee_init(UART_HandleTypeDef *huart)
{
	uart_status_t rx_init_status;
	uart_status_t rx_hook_status;

	ring_init(&server_ring);
	
	server_recieve_data.rx_msg = &server_rx_msg;
	server_recieve_data.rx_msg->huart = huart;
	server_recieve_data.rx_msg->pBuffer = server_rx_data;
	server_recieve_data.rx_msg->Len = sizeof(server_rx_data);

	rx_init_status = uart_rx_init(&server_recieve_data);
	rx_hook_status = uart_rx_hook_reg(&server_recieve_data, server_rx_callback);
	g_referee_link_debug.uart_init_ok = ((rx_init_status == UART_OK) && (rx_hook_status == UART_OK)) ? 1U : 0U;
}

/*===========================================================================*/
/*                    自定义控制器 - 环形缓冲区方案                            */
/*===========================================================================*/

static uart_rx_t ctrller_recieve_data;
static uart_msg_t ctrller_rx_msg;
static uint8_t ctrller_rx_data[RE_RX_BUFFER_SIZE]; // DMA接收缓冲区
static ring_buffer_t ctrller_ring;                  // 环形缓冲区
static uint8_t ctrller_frame_buf[RE_RX_BUFFER_SIZE]; // 帧解析缓冲区
static custom_controller_info_t custom_controller_info;
keyboard_t kb_info;

static int16_t ctrller_saturate_i32_to_i16(int32_t value)
{
	if (value > INT16_MAX) {
		return INT16_MAX;
	}
	if (value < INT16_MIN) {
		return INT16_MIN;
	}
	return (int16_t)value;
}

static uint8_t ctrller_read_varint(const uint8_t *data, uint16_t len, uint16_t *offset, uint64_t *value)
{
	uint64_t result = 0U;
	uint8_t shift = 0U;

	if ((data == NULL) || (offset == NULL) || (value == NULL)) {
		return 0U;
	}

	while ((*offset < len) && (shift < 64U)) {
		const uint8_t byte = data[*offset];
		(*offset)++;

		result |= ((uint64_t)(byte & 0x7FU)) << shift;
		if ((byte & 0x80U) == 0U) {
			*value = result;
			return 1U;
		}

		shift = (uint8_t)(shift + 7U);
	}

	return 0U;
}

/* 2026 协议中，旧 0x0304 被替换为 KeyboardMouseControl。
 * 当前工程仍然沿用 keyboard_t 作为统一输入结构，因此这里做一层 protobuf -> keyboard_t 映射。
 * 结构体字段对应关系：
 * 1 mouse_x  -> keyboard_t.mouse_x
 * 2 mouse_y  -> keyboard_t.mouse_y
 * 3 mouse_z  -> keyboard_t.mouse_z
 * 4 left     -> keyboard_t.left_button_down
 * 5 right    -> keyboard_t.right_button_down
 * 6 key mask -> keyboard_t.key_code(低16位)
 * 7 mid      -> keyboard_t.reserved bit0
 */
static uint8_t ctrller_decode_keyboardmousecontrol_to_kbinfo(const uint8_t *data, uint16_t len)
{
	keyboard_t decoded = {0};
	uint16_t offset = 0U;
	uint8_t parsed_field = 0U;

	if ((data == NULL) || (len == 0U)) {
		return 0U;
	}

	while (offset < len) {
		uint64_t tag = 0U;
		uint64_t value = 0U;
		uint32_t field_number;
		uint32_t wire_type;

		if (ctrller_read_varint(data, len, &offset, &tag) == 0U) {
			return 0U;
		}

		field_number = (uint32_t)(tag >> 3);
		wire_type = (uint32_t)(tag & 0x07U);
		if ((field_number == 0U) || (wire_type != 0U)) {
			return 0U;
		}

		if (ctrller_read_varint(data, len, &offset, &value) == 0U) {
			return 0U;
		}

		switch (field_number) {
			case 1U:
				decoded.mouse_x = ctrller_saturate_i32_to_i16((int32_t)value);
				parsed_field = 1U;
				break;

			case 2U:
				decoded.mouse_y = ctrller_saturate_i32_to_i16((int32_t)value);
				parsed_field = 1U;
				break;

			case 3U:
				decoded.mouse_z = ctrller_saturate_i32_to_i16((int32_t)value);
				parsed_field = 1U;
				break;

			case 4U:
				decoded.left_button_down = (value != 0U) ? 1 : 0;
				parsed_field = 1U;
				break;

			case 5U:
				decoded.right_button_down = (value != 0U) ? 1 : 0;
				parsed_field = 1U;
				break;

			case 6U:
				decoded.key_code.key_code = (uint16_t)(value & 0xFFFFU);
				decoded.reserved = (uint16_t)((value >> 16U) & 0xFFFFU);
				parsed_field = 1U;
				break;

			case 7U:
				if (value != 0U) {
					decoded.reserved |= 0x0001U;
				} else {
					decoded.reserved &= (uint16_t)(~0x0001U);
				}
				parsed_field = 1U;
				break;

			default:
				return 0U;
		}
	}

	if (parsed_field == 0U) {
		return 0U;
	}

	custom_controller_info.keyboard = decoded;
	kb_info = decoded;
	return 1U;
}

static uint8_t ctrller_try_decode_keyboardmousecontrol_packet(const uint8_t *data, uint16_t len, uint16_t *decoded_offset)
{
	uint16_t offset = 0U;

	if ((data == NULL) || (len == 0U)) {
		return 0U;
	}

	if (ctrller_decode_keyboardmousecontrol_to_kbinfo(data, len) != 0U) {
		if (decoded_offset != NULL) {
			*decoded_offset = 0U;
		}
		return 1U;
	}

	for (offset = 1U; offset < len; offset++) {
		if (ctrller_decode_keyboardmousecontrol_to_kbinfo(data + offset, (uint16_t)(len - offset)) != 0U) {
			if (decoded_offset != NULL) {
				*decoded_offset = offset;
			}
			return 1U;
		}
	}

	return 0U;
}

static uint8_t ctrller_decode_legacy_keyboard_window_to_kbinfo(const uint8_t *data, uint16_t len, uint16_t offset)
{
	keyboard_t decoded = {0};
	const uint16_t legacy_len = 12U;
	const uint8_t *payload = NULL;
	int32_t abs_mouse_x;
	int32_t abs_mouse_y;
	int32_t abs_mouse_z;

	if ((data == NULL) || (len < (uint16_t)(offset + legacy_len))) {
		return 0U;
	}

	payload = data + offset;
	decoded.mouse_x = (int16_t)((uint16_t)payload[0] | ((uint16_t)payload[1] << 8));
	decoded.mouse_y = (int16_t)((uint16_t)payload[2] | ((uint16_t)payload[3] << 8));
	decoded.mouse_z = (int16_t)((uint16_t)payload[4] | ((uint16_t)payload[5] << 8));
	decoded.left_button_down = (payload[6] != 0U) ? 1 : 0;
	decoded.right_button_down = (payload[7] != 0U) ? 1 : 0;
	decoded.key_code.key_code = (uint16_t)((uint16_t)payload[8] | ((uint16_t)payload[9] << 8));
	decoded.reserved = (uint16_t)((uint16_t)payload[10] | ((uint16_t)payload[11] << 8));

	abs_mouse_x = (decoded.mouse_x >= 0) ? decoded.mouse_x : -decoded.mouse_x;
	abs_mouse_y = (decoded.mouse_y >= 0) ? decoded.mouse_y : -decoded.mouse_y;
	abs_mouse_z = (decoded.mouse_z >= 0) ? decoded.mouse_z : -decoded.mouse_z;

	if ((abs_mouse_x > 5000) || (abs_mouse_y > 5000) || (abs_mouse_z > 5000)) {
		return 0U;
	}

	custom_controller_info.keyboard = decoded;
	kb_info = decoded;
	return 1U;
}

static uint8_t ctrller_try_decode_fixed21_legacy_packet(const uint8_t *data, uint16_t len, uint16_t *decoded_offset)
{
	const uint16_t fixed_offset = 9U;

	if (ctrller_decode_legacy_keyboard_window_to_kbinfo(data, len, fixed_offset) == 0U) {
		return 0U;
	}

	if (decoded_offset != NULL) {
		*decoded_offset = fixed_offset;
	}

	return 1U;
}

uint8_t Ctrller_Receive_Buffer[256];

static void ctrller_rx_callback(uint8_t *pData, uint32_t size)
{
	if ((size == 0U) || (pData == NULL)) {
		return;
	}

	if ((size == 21U) && (pData[0] == 0xA9U) && (pData[1] == 0x53U)) {
		/* 当前现场链路已经确认会稳定下发 0xA9 0x53 开头的 21 字节固定包。
		 * 这类包必须只走固定窗口解析，不能再落回 protobuf 兜底，否则会把正确的 mouse_x
		 * 和按键值又用错误路径覆盖掉，表现成“偶尔对一下，随后又变掉”。
		 */
		(void)ctrller_try_decode_fixed21_legacy_packet(pData, (uint16_t)size, NULL);
		return;
	}

	/* 新版 KeyboardMouseControl 可能不再走旧的 A5+cmd_id 帧格式，而是直接给出 protobuf 数据段。
	 * 遇到非 A5 起始的数据时，优先按 KeyboardMouseControl 尝试解码；成功后直接更新 kb_info。
	 */
	if (pData[0] != REFEREE_SOF) {
		if (ctrller_try_decode_keyboardmousecontrol_packet(pData, (uint16_t)size, NULL) != 0U) {
			return;
		}
	}

	ring_write(&ctrller_ring, pData, size);
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

static void referee_copy_payload(void *dst, size_t dst_size, const uint8_t *src, uint16_t payload_len)
{
	size_t copy_len = payload_len;

	if ((dst == NULL) || (src == NULL) || (dst_size == 0U)) {
		return;
	}

	if (copy_len > dst_size) {
		copy_len = dst_size;
	}

	memset(dst, 0, dst_size);
	memcpy(dst, src, copy_len);
}

/**
 * @brief 解析单帧裁判数据（内部使用）
 */
static void referee_parse_frame(uint8_t *buff, uint16_t len)
{
	const uint16_t payload_len = (len >= (DATA_Offset + LEN_TAIL)) ? (uint16_t)(len - DATA_Offset - LEN_TAIL) : 0U;

	(void)len;
	memcpy(&referee_info.FrameHeader, buff, LEN_HEADER);
	referee_info.CmdID = (uint16_t)(buff[6] << 8) | buff[5];
	g_referee_link_debug.last_cmd_id = referee_info.CmdID;
	g_referee_link_debug.last_frame_length = len;
	
	switch (referee_info.CmdID)
	{
	case ID_game_state:
		referee_copy_payload(&referee_info.GameState, sizeof(referee_info.GameState), buff + DATA_Offset, payload_len);
		break;
	case ID_game_result:
		referee_copy_payload(&referee_info.GameResult, sizeof(referee_info.GameResult), buff + DATA_Offset, payload_len);
		break;
	case ID_game_robot_survivors:
		referee_copy_payload(&referee_info.GameRobotHP, sizeof(referee_info.GameRobotHP), buff + DATA_Offset, payload_len);
		break;
	case ID_event_data:
		referee_copy_payload(&referee_info.EventData, sizeof(referee_info.EventData), buff + DATA_Offset, payload_len);
		break;
	case ID_supply_projectile_action:
		referee_copy_payload(&referee_info.SupplyProjectileAction, sizeof(referee_info.SupplyProjectileAction), buff + DATA_Offset, payload_len);
		break;
	case ID_game_robot_state:
		referee_copy_payload(&referee_info.GameRobotState, sizeof(referee_info.GameRobotState), buff + DATA_Offset, payload_len);
		g_referee_link_debug.last_robot_id = referee_info.GameRobotState.robot_id;
		break;
	case ID_power_heat_data:
		referee_copy_payload(&referee_info.PowerHeatData, sizeof(referee_info.PowerHeatData), buff + DATA_Offset, payload_len);
		break;
	case ID_game_robot_pos:
		referee_copy_payload(&referee_info.GameRobotPos, sizeof(referee_info.GameRobotPos), buff + DATA_Offset, payload_len);
		break;
	case ID_buff_musk:
		referee_copy_payload(&referee_info.BuffMusk, sizeof(referee_info.BuffMusk), buff + DATA_Offset, payload_len);
		break;
	case ID_aerial_robot_energy:
		referee_copy_payload(&referee_info.AerialRobotEnergy, sizeof(referee_info.AerialRobotEnergy), buff + DATA_Offset, payload_len);
		break;
	case ID_robot_hurt:
		referee_copy_payload(&referee_info.RobotHurt, sizeof(referee_info.RobotHurt), buff + DATA_Offset, payload_len);
		break;
	case ID_shoot_data:
		referee_copy_payload(&referee_info.ShootData, sizeof(referee_info.ShootData), buff + DATA_Offset, payload_len);
		break;
	case ID_student_interactive:
		referee_copy_payload(&referee_info.ReceiveData, sizeof(referee_info.ReceiveData), buff + DATA_Offset, payload_len);
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
			g_referee_link_debug.crc8_fail_count++;
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
			g_referee_link_debug.frame_parse_count++;
			frames_parsed++;
		} else {
			g_referee_link_debug.crc16_fail_count++;
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

uint8_t CtrllerData[CtrllerData_Length] = {
    '3','1','4','1',
    '3','1','4','1',
    '3','1','4','1',
    '3','1','4','1',
    '3','1','4','1',
    '0','6','2','8',
	'3','0',  '0'
};

/**
 * @brief 解析单帧控制器数据（内部使用）
 * @param buff 完整的帧数据
 * @param len 帧长度
 */
static void ctrller_parse_frame(uint8_t *buff, uint16_t len)
{
	const uint16_t payload_len = (len >= (DATA_Offset + LEN_TAIL)) ? (uint16_t)(len - DATA_Offset - LEN_TAIL) : 0U;

	// 调试用：复制帧数据
	memcpy(Ctrller_Receive_Buffer, buff, len < 256 ? len : 256);
	// 写入帧头数据
	memcpy(&custom_controller_info.FrameHeader, buff, LEN_HEADER);
	
	// 提取 CmdID (小端序)
	custom_controller_info.CmdID = (uint16_t)(buff[6] << 8) | buff[5];
	
	// 根据 CmdID 解析数据
	switch (custom_controller_info.CmdID)
	{
	case 0x0302:  // 自定义控制器数据
		// memcpy(&custom_controller_info.CustomController, (buff + DATA_Offset), LEN_custom_controller);
		// memcpy(CtrllerData, &custom_controller_info.CustomController, LEN_custom_controller);
			memcpy(CtrllerData, Ctrller_Receive_Buffer + 7, 27);
				break;
	case 0x0304:  // 键鼠数据
				memcpy(&custom_controller_info.keyboard, (buff + DATA_Offset), LEN_keyboard);
				kb_info = custom_controller_info.keyboard;
				break;
	case 0x0311:  // 若底层把 KeyboardMouseControl 通过自定义数据链路透传，则这里尝试按 protobuf 兼容解析
				(void)ctrller_decode_keyboardmousecontrol_to_kbinfo(buff + DATA_Offset, payload_len);
				break;
	default:
				/* 兼容一些中间层改造：即使 cmd_id 改了，只要 payload 仍然是 KeyboardMouseControl 的 protobuf，
				 * 这里也可以直接更新 kb_info。
				 */
				(void)ctrller_decode_keyboardmousecontrol_to_kbinfo(buff + DATA_Offset, payload_len);
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
	uint8_t tx_buff[MAX_SIZE];
	uint16_t frame_length = frameheader_len/*5*/ + cmd_len/*2*/ + len/*函参*/ + crc_len/*2*/;

	if ((p_data == NULL) || (len == 0U) || (frame_length > MAX_SIZE) || (referee_is_inited() == 0U))
	{
		return;
	}

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

	HAL_UART_Transmit(server_recieve_data.rx_msg->huart, tx_buff, frame_length, HAL_MAX_DELAY);
}

HAL_StatusTypeDef referee_send_raw_data(const uint8_t *p_data, uint16_t len, uint32_t timeout)
{
	if ((p_data == NULL) || (len == 0U) || (referee_is_inited() == 0U)) {
		return HAL_ERROR;
	}

	return HAL_UART_Transmit(server_recieve_data.rx_msg->huart, (uint8_t *)p_data, len, timeout);
}
