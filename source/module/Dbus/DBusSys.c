#include "DBusSys.h"
#include "usart.h"
#include "FreeRTOS.h"
// #include "DbusSys.h"
#include "string.h"
#include "stdlib.h"

rc_info_t remoter;
uart_rx_t DBus_msg;
uart_msg_t Dbus_rx_msg;
uint8_t Dbus_rx_buff[DBUS_BUFF_SIZE];
uint32_t Lsat_Conut;

/**
 * @brief  DBusSys初始化函数
 * 
 * */
void DBus_Init(void)
{
  // 分配内存给DBus_msg中的rx_msg结构体
  DBus_msg.rx_msg = &Dbus_rx_msg;
  // 分配内存给DBus_msg中的rx_msg的pBuffer数组，用于存储接收到的数据
  DBus_msg.rx_msg->pBuffer = Dbus_rx_buff;
  // 设置DBus_msg中的rx_msg的huart为huart5，即使用USART5
  DBus_msg.rx_msg->huart = &huart5;
  // 设置DBus_msg中的rx_msg的Len为18，表示接收数据的长度为18字节
  DBus_msg.rx_msg->Len = DBUS_BUFF_SIZE;
  // 调用uart_rx_init函数初始化DBus_msg
  uart_rx_init(&DBus_msg);
}

/**
 * @brief  DBusSys刷新函数
 * 
 * */
void DBus_Refresh(void)
{
  // 如果uart5_msg的count与上一次的Lsat_Conut不相等，说明有新数据接收
  if (DBus_msg.count != Lsat_Conut)
  {
    // 调用get_dr16_data函数解析DBus_msg的接收缓冲区数据，更新遥控器数据
    get_dr16_data(&remoter, Dbus_rx_buff);
    // 更新Lsat_Conut为当前的uart5_msg->count，用于下一次比较
    Lsat_Conut = DBus_msg.count;
  }
}

/**
 * @brief  解析DBus_msg的接收缓冲区数据，更新遥控器数据
 * 
 * */
void get_dr16_data(rc_info_t *rc, uint8_t buff[])
{
  // satori：这里完成的是数据的分离和拼接，减去1024是为了让数据的中间值变为0
  rc->ch1 = (buff[0] | buff[1] << 8) & 0x07FF;
  rc->ch1 -= 1024;
  rc->ch2 = (buff[1] >> 3 | buff[2] << 5) & 0x07FF;
  rc->ch2 -= 1024;
  rc->ch3 = (buff[2] >> 6 | buff[3] << 2 | buff[4] << 10) & 0x07FF;
  rc->ch3 -= 1024;
  rc->ch4 = (buff[4] >> 1 | buff[5] << 7) & 0x07FF;
  rc->ch4 -= 1024;

  // satori:防止数据零漂，设置正负5的死区
  /* prevent remote control zero deviation */
  if (rc->ch1 <= 5 && rc->ch1 >= -5)
    rc->ch1 = 0;
  if (rc->ch2 <= 5 && rc->ch2 >= -5)
    rc->ch2 = 0;
  if (rc->ch3 <= 5 && rc->ch3 >= -5)
    rc->ch3 = 0;
  if (rc->ch4 <= 5 && rc->ch4 >= -5)
    rc->ch4 = 0;

  rc->sw1 = ((buff[5] >> 4) & 0x000C) >> 2;
  rc->sw2 = (buff[5] >> 4) & 0x0003;

  // satori:防止数据溢出
  if ((abs(rc->ch1) > 660) ||
      (abs(rc->ch2) > 660) ||
      (abs(rc->ch3) > 660) ||
      (abs(rc->ch4) > 660))
  {
    memset(rc, 0, sizeof(rc_info_t));
    return;
  }

  rc->keyboard.mouse_x = buff[6] | (buff[7] << 8); // x axis
  rc->keyboard.mouse_y = buff[8] | (buff[9] << 8);
  rc->keyboard.mouse_z = buff[10] | (buff[11] << 8);

  rc->keyboard.left_button_down = buff[12];
  rc->keyboard.right_button_down = buff[13];

  rc->keyboard.key_code.key_code = buff[14] | buff[15] << 8; // key borad code
  rc->wheel = (buff[16] | buff[17] << 8) - 1024;
}
