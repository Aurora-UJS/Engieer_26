#include "referee_api.h"

static uart_rx_t server_recieve_data;
static uart_msg_t server_rx_msg;
static uint8_t server_rx_data[RE_RX_BUFFER_SIZE]; // 接收数据缓冲区
static uint32_t server_rx_cont = 0;

/** 
 * @brief 初始化裁判模块的UART接收配置
 * @param huart 指向UART句柄的指针
 */
void referee_init(UART_HandleTypeDef *huart)
{
	server_recieve_data.rx_msg = &server_rx_msg;
	server_recieve_data.rx_msg->huart = huart;
	server_recieve_data.rx_msg->pBuffer = server_rx_data;
	server_recieve_data.rx_msg->Len = sizeof(server_rx_data);

	uart_rx_init(&server_recieve_data);
}

static uart_rx_t ctrller_recieve_data;
static uart_msg_t ctrller_rx_msg;
static uint8_t ctrller_rx_data[RE_RX_BUFFER_SIZE]; // 接收数据缓冲区
static uint32_t ctrller_rx_cont = 0;

/** 
 * @brief 初始化控制器模块的UART接收配置
 * @param huart 指向UART句柄的指针
 */
void ctrller_init(UART_HandleTypeDef *huart)
{
	ctrller_recieve_data.rx_msg = &ctrller_rx_msg;
	ctrller_recieve_data.rx_msg->huart = huart;
	ctrller_recieve_data.rx_msg->pBuffer = ctrller_rx_data;
	ctrller_recieve_data.rx_msg->Len = sizeof(ctrller_rx_data);
  
	uart_rx_init(&ctrller_recieve_data);
}

static referee_info_t referee_info;

/** 
 * @brief 解析裁判数据包，处理帧头校验和数据分发
 * @param buff 输入的数据缓冲区指针
 */
void JudgeReadData(uint8_t *buff)
{
	uint16_t judge_length; // 统计一帧数据长度
	if (buff == NULL)	   // 空数据包，则不作任何处理
		return;

	// 写入帧头数据(5-byte),用于判断是否开始存储裁判数据
	memcpy(&referee_info.FrameHeader, buff, LEN_HEADER);
  
	// 判断帧头数据(0)是否为0xA5
	if (buff[SOF] == REFEREE_SOF)
	{
		// 帧头CRC8校验
		if (Verify_CRC8_Check_Sum(buff, LEN_HEADER) == CRC_Check_True)
		{
			// 统计一帧数据长度(byte),用于CR16校验
			judge_length = buff[DATA_LENGTH] + LEN_HEADER + LEN_CMDID + LEN_TAIL;
			// 帧尾CRC16校验
			if (Verify_CRC16_Check_Sum(buff, judge_length) == CRC_Check_True)
			{
				// 2个8位拼成16位int
				referee_info.CmdID = (buff[6] << 8 | buff[5]);
				// 解析数据命令码,将数据拷贝到相应结构体中(注意拷贝数据的长度)
				// 第8个字节开始才是数据 data=7
				switch (referee_info.CmdID)
				{
				case ID_game_state: // 0x0001
					memcpy(&referee_info.GameState, (buff + DATA_Offset), LEN_game_state);
					break;
				case ID_game_result: // 0x0002
					memcpy(&referee_info.GameResult, (buff + DATA_Offset), LEN_game_result);
					break;
				case ID_game_robot_survivors: // 0x0003
					memcpy(&referee_info.GameRobotHP, (buff + DATA_Offset), LEN_game_robot_HP);
					break;
				case ID_event_data: // 0x0101
					memcpy(&referee_info.EventData, (buff + DATA_Offset), LEN_event_data);
					break;
				case ID_supply_projectile_action: // 0x0102
					memcpy(&referee_info.SupplyProjectileAction, (buff + DATA_Offset), LEN_supply_projectile_action);
					break;
				case ID_game_robot_state: // 0x0201
					memcpy(&referee_info.GameRobotState, (buff + DATA_Offset), LEN_game_robot_state);
					break;
				case ID_power_heat_data: // 0x0202
					memcpy(&referee_info.PowerHeatData, (buff + DATA_Offset), LEN_power_heat_data);
					break;
				case ID_game_robot_pos: // 0x0203
					memcpy(&referee_info.GameRobotPos, (buff + DATA_Offset), LEN_game_robot_pos);
					break;
				case ID_buff_musk: // 0x0204
					memcpy(&referee_info.BuffMusk, (buff + DATA_Offset), LEN_buff_musk);
					break;
				case ID_aerial_robot_energy: // 0x0205
					memcpy(&referee_info.AerialRobotEnergy, (buff + DATA_Offset), LEN_aerial_robot_energy);
					break;
				case ID_robot_hurt: // 0x0206
					memcpy(&referee_info.RobotHurt, (buff + DATA_Offset), LEN_robot_hurt);
					break;
				case ID_shoot_data: // 0x0207
					memcpy(&referee_info.ShootData, (buff + DATA_Offset), LEN_shoot_data);
					break;
				case ID_student_interactive: // 0x0301   syhtodo接收代码未测试
					memcpy(&referee_info.ReceiveData, (buff + DATA_Offset), LEN_receive_data);
					break;
				}
			}
		}
		// 首地址加帧长度,指向CRC16下一字节,用来判断是否为0xA5,从而判断一个数据包是否有多帧数据
		if (*(buff + sizeof(xFrameHeader) + LEN_CMDID + referee_info.FrameHeader.DataLength + LEN_TAIL) == 0xA5)
		{ // 如果一个数据包出现了多帧数据,则再次调用解析函数,直到所有数据包解析完毕
			JudgeReadData(buff + sizeof(xFrameHeader) + LEN_CMDID + referee_info.FrameHeader.DataLength + LEN_TAIL);
		}
	}
}

static custom_controller_info_t custom_controller_info;

/** 
 * @brief 解析控制器数据包，处理帧头校验和数据分发
 * @param buff 输入的数据缓冲区指针
 */
void CtrllerReadData(uint8_t *buff)
{
	uint16_t judge_length; // 统计一帧数据长度
	if (buff == NULL)	   // 空数据包，则不作任何处理
		return;

	// 写入帧头数据(5-byte),用于判断是否开始存储裁判数据
	memcpy(&custom_controller_info.FrameHeader, buff, LEN_HEADER);
  
	// 判断帧头数据(0)是否为0xA5
	if (buff[SOF] == REFEREE_SOF)
	{
		// 帧头CRC8校验
		if (Verify_CRC8_Check_Sum(buff, LEN_HEADER) == CRC_Check_True)
		{
			// 统计一帧数据长度(byte),用于CR16校验
			judge_length = buff[DATA_LENGTH] + LEN_HEADER + LEN_CMDID + LEN_TAIL;
			// 帧尾CRC16校验
			if (Verify_CRC16_Check_Sum(buff, judge_length) == CRC_Check_True)
			{
				// 2个8位拼成16位int
				custom_controller_info.CmdID = (buff[6] << 8 | buff[5]);
				// 解析数据命令码,将数据拷贝到相应结构体中(注意拷贝数据的长度)
				// 第8个字节开始才是数据 data=7
				switch (custom_controller_info.CmdID)
				{
                case 0x0302:  // 0x0302
					memcpy(&custom_controller_info.CustomController, (buff + DATA_Offset), LEN_custom_controller);
					break;

					case 0x0304:
					memcpy(&custom_controller_info.keyboard, (buff + DATA_Offset), LEN_keyboard);
				}
			}
		}
		// 首地址加帧长度,指向CRC16下一字节,用来判断是否为0xA5,从而判断一个数据包是否有多帧数据
		if (*(buff + sizeof(xFrameHeader) + LEN_CMDID + custom_controller_info.FrameHeader.DataLength + LEN_TAIL) == 0xA5)
		{ // 如果一个数据包出现了多帧数据,则再次调用解析函数,直到所有数据包解析完毕
			CtrllerReadData(buff + sizeof(xFrameHeader) + LEN_CMDID + custom_controller_info.FrameHeader.DataLength + LEN_TAIL);
		}
	}
}

/** 
 * @brief 获取解析后的裁判信息数据
 * @return 当前裁判数据结构体
 */
referee_info_t *get_referee_msg(void)
{
	if (server_rx_cont == server_recieve_data.count)
	{
		return &referee_info;
	}
	server_rx_cont = server_recieve_data.count;
	JudgeReadData(server_rx_data);
	return &referee_info;
}

/** 
 * @brief 获取解析后的自定义控制器信息数据
 * @return 当前自定义控制器数据结构体
 */
custom_controller_info_t *get_custom_controller_msg(void)
{
	if (ctrller_rx_cont == ctrller_recieve_data.count)
	{
		return &custom_controller_info;
	}
	ctrller_rx_cont = ctrller_recieve_data.count;
	CtrllerReadData(ctrller_rx_data);
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
