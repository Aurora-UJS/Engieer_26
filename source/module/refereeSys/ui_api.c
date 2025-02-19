#include "string.h"
#include "main.h"
#include "ui_api.h"
#include "crc_api.h"
#include "usart.h"

unsigned char frame_header[3];//4-Byte
unsigned int frame_crc_dwLength = 3;//stream length
char cacre[9];//暂时没用到

ext_student_interactive_header_data_t graph_one;//最终结构体
ext_student_interactive_header_data_t graph_two;//最终结构体
ext_student_interactive_header_data_t graph_three;//最终结构体

ext_client_custom_graphic_t custom_graphic;//暂时未用到
ext_client_custom_graphic_t struct_t;

/**
  * @brief     pack data to bottom device
  * @param[in] sof : start of framehearder
  * @param[in] cmd_id:  command id of data
  * @param[in] *p_data: pointer to the data to be sent
  * @param[in] len:     the data length
  */
uint8_t seq = 0;/*sequence初始化*/
void referee_data_pack_handle(uint8_t sof, uint16_t cmd_id, uint8_t *p_data, uint16_t len)
{
	unsigned char i = 0;
	uint8_t tx_buff[MAX_SIZE];
	//uint8_t tx_out[MAX_SIZE];
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
		while (HAL_UART_Transmit_IT(&huart1,&tx_buff[i],sizeof(tx_buff[i])) == HAL_BUSY);		
	}
}

void graph_1(void)
{
	graph_one.data_cmd_id = 0x0101;
	graph_one.sender_ID = 1;
	graph_one.receiver_ID = 0x0101;

	graph_one.graphic_custom.grapic_data_struct[0].figure_name[0] = '1';
	graph_one.graphic_custom.grapic_data_struct[0].figure_name[1] = '0';
	graph_one.graphic_custom.grapic_data_struct[0].figure_name[2] = '0';
	
	graph_one.graphic_custom.grapic_data_struct[0].operate_tpye = 1;//图形操作：增加一个图形
	graph_one.graphic_custom.grapic_data_struct[0].figure_type = 2;//图形类型：正圆
	graph_one.graphic_custom.grapic_data_struct[0].layer = 1;//图层数：1
	graph_one.graphic_custom.grapic_data_struct[0].color = 3;//颜色：己方颜色
	graph_one.graphic_custom.grapic_data_struct[0].details_a = 0;//起始角度：0
	graph_one.graphic_custom.grapic_data_struct[0].details_b = 0;//终止角度：0
	graph_one.graphic_custom.grapic_data_struct[0].width = 10;//线宽：10
	graph_one.graphic_custom.grapic_data_struct[0].start_x = 960;//起始x：960
	graph_one.graphic_custom.grapic_data_struct[0].start_y = 540;//起始y：540
	graph_one.graphic_custom.grapic_data_struct[0].details_c = 181;
	graph_one.graphic_custom.grapic_data_struct[0].details_d = 0;
	graph_one.graphic_custom.grapic_data_struct[0].details_e = 0;
	
	referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&graph_one, sizeof(graph_one));
}

void graph_2(void)
{
	graph_two.data_cmd_id = 0x0101;
	graph_two.sender_ID = 1;
	graph_two.receiver_ID = 0x0101;

	graph_two.graphic_custom.grapic_data_struct[0].figure_name[0] = '1';
	graph_two.graphic_custom.grapic_data_struct[0].figure_name[1] = '1';
	graph_two.graphic_custom.grapic_data_struct[0].figure_name[2] = '0';
	
	graph_two.graphic_custom.grapic_data_struct[0].operate_tpye = 1;//图形操作：增加一个图形
	graph_two.graphic_custom.grapic_data_struct[0].figure_type = 2;//图形类型：正圆
	graph_two.graphic_custom.grapic_data_struct[0].layer = 1;//图层数：1
	graph_two.graphic_custom.grapic_data_struct[0].color = 3;//颜色：己方颜色
	graph_two.graphic_custom.grapic_data_struct[0].details_a = 0;//起始角度：0
	graph_two.graphic_custom.grapic_data_struct[0].details_b = 0;//终止角度：0
	graph_two.graphic_custom.grapic_data_struct[0].width = 10;//线宽：10
	graph_two.graphic_custom.grapic_data_struct[0].start_x = 960;//起始x：960
	graph_two.graphic_custom.grapic_data_struct[0].start_y = 540;//起始y：540
	graph_two.graphic_custom.grapic_data_struct[0].details_c = 90;
	graph_two.graphic_custom.grapic_data_struct[0].details_d = 0;
	graph_two.graphic_custom.grapic_data_struct[0].details_e = 0;
	
	referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&graph_two, sizeof(graph_two));
}

void graph_3(uint32_t value)
{
	graph_three.data_cmd_id = 0x0101;
	graph_three.sender_ID = 1;
	graph_three.receiver_ID = 0x0101;

	graph_three.graphic_custom.grapic_data_struct[0].figure_name[0] = '1';
	graph_three.graphic_custom.grapic_data_struct[0].figure_name[1] = '0';
	graph_three.graphic_custom.grapic_data_struct[0].figure_name[2] = '1';

	graph_three.graphic_custom.grapic_data_struct[0].operate_tpye = 1;//图形操作：增加一个图形
	graph_three.graphic_custom.grapic_data_struct[0].figure_type = 6;//图形类型：整形数
	graph_three.graphic_custom.grapic_data_struct[0].layer = 3;//图层数：1
	graph_three.graphic_custom.grapic_data_struct[0].color = 3;//颜色：己方颜色
	graph_three.graphic_custom.grapic_data_struct[0].details_a = 40;//字体大小
	graph_three.graphic_custom.grapic_data_struct[0].details_b = 0;//终止角度：0
	graph_three.graphic_custom.grapic_data_struct[0].width = 0;//线宽：10
	graph_three.graphic_custom.grapic_data_struct[0].start_x = 960;//起始x：960
	graph_three.graphic_custom.grapic_data_struct[0].start_y = 540;//起始y：540
	graph_three.graphic_custom.grapic_data_struct[0].details_c = value&((1<<10)-1);
	value>>=10;
	graph_three.graphic_custom.grapic_data_struct[0].details_d = value&((1<<11)-1);
	value>>=11;
	graph_three.graphic_custom.grapic_data_struct[0].details_e = value&((1<<11)-1);
	referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&graph_three, sizeof(graph_three));
}
