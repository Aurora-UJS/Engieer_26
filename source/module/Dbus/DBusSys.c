#include "DBusSys.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "DbusSys.h"
#include "String.h"
#include "stdlib.h"

rc_info_t remoter;
uint8_t rx_buff[BUFF_SIZE];
uart_rx_t *DBus_msg;

// 函数功能：初始化DBus相关数据结构和资源
// 参数：无
// 返回值：无
void DBus_Init(void)
{
    // 分配内存给DBus_msg结构体
    DBus_msg = (uart_rx_t *)pvPortMalloc(sizeof(uart_rx_t));
    // 分配内存给DBus_msg中的rx_msg结构体
    DBus_msg->rx_msg = (uart_msg_t *)pvPortMalloc(sizeof(uart_msg_t));
    // 分配内存给DBus_msg中的rx_msg的pBuffer数组，用于存储接收到的数据
    DBus_msg->rx_msg->pBuffer = (uint8_t *)pvPortMalloc(18);
    // 设置DBus_msg中的rx_msg的huart为huart5，即使用USART5
    DBus_msg->rx_msg->huart = &huart5;
    // 设置DBus_msg中的rx_msg的Len为18，表示接收数据的长度为18字节
    DBus_msg->rx_msg->Len = 18;
    // 调用uart_rx_init函数初始化DBus_msg
    uart_rx_init(DBus_msg);
}

// 函数功能：刷新DBus数据
// 参数：无
// 返回值：无
void DBus_Refresh(void)
{
    // 判断uart5_msg的count是否为奇数
    if (uart5_msg->count % 2 == 1)
    {
        // 将uart5_msg的接收缓冲区数据复制到DBus_msg的接收缓冲区
        memcpy(DBus_msg->rx_msg->pBuffer, uart5_msg->rx_msg->pBuffer, 18);
        // 解析DBus_msg的接收缓冲区数据，更新遥控器数据
        get_dr16_data(&remoter, DBus_msg->rx_msg->pBuffer);
    }
}


// 定义DBusSys的接口
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

  rc->mouse.x = buff[6] | (buff[7] << 8); // x axis
  rc->mouse.y = buff[8] | (buff[9] << 8);
  rc->mouse.z = buff[10] | (buff[11] << 8);

  rc->mouse.l = buff[12];
  rc->mouse.r = buff[13];

  rc->kb.key_code = buff[14] | buff[15] << 8; // key borad code
  rc->wheel = (buff[16] | buff[17] << 8) - 1024;
}