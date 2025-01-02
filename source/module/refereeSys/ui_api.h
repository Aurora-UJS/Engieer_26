#ifndef _UI_H
#define _UI_H

//屏幕分辨率1920x1080
#define SCREEN_WIDTH 1080
#define SCREEN_LENGTH 1920

#define MAX_SIZE          128    
#define frameheader_len  5       //帧头长度
#define cmd_len          2       //命令码长度
#define crc_len          2       //CRC16校验码长度

#pragma pack(1)

typedef struct
{
  uint8_t figure_name[3];  
	uint32_t operate_tpye : 3;
	uint32_t figure_type : 3;
	uint32_t layer : 4;      
	uint32_t color : 4;       
	uint32_t details_a : 9; 
	uint32_t details_b : 9;   
	uint32_t width : 10;     
	uint32_t start_x : 11;   
	uint32_t start_y : 11;    
	uint32_t details_c : 10;
	uint32_t details_d : 11;      
	uint32_t details_e : 11;
} graphic_data_struct_t;

typedef struct
{
	graphic_data_struct_t grapic_data_struct[1];
} ext_client_custom_graphic_t/**/;

typedef struct
{
	uint16_t data_cmd_id;	                     //子命令ID
	uint16_t sender_ID;	                       //发送者ID
	uint16_t receiver_ID;	                     //接受者ID
	ext_client_custom_graphic_t/**/ graphic_custom;//总结构体
}ext_student_interactive_header_data_t;

#pragma pack()

extern ext_student_interactive_header_data_t graph_one;//最终结构体
extern ext_student_interactive_header_data_t graph_two;//最终结构体
extern ext_student_interactive_header_data_t graph_three;//最终结构体

extern ext_client_custom_graphic_t/**/ custom_graphic;	

/***********************函数部分******************************/

void referee_data_pack_handle(uint8_t sof, uint16_t cmd_id, uint8_t *p_data, uint16_t len);
/***********************函数部分******************************/

void graph_1(void);
void graph_2(void);
void graph_3(uint32_t value);

#endif
