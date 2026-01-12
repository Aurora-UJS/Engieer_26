#ifndef DBUSSYS_H
#define DBUSSYS_H


#include "main.h"
#include "uart_api.h"
#include "keyBoard.h"

#define DBUS_BUFF_SIZE	18
#define Remoter_CHMAX 660

typedef struct 
{
  /* rocker channel information */
  int16_t ch1;
  int16_t ch2;
  int16_t ch3;
  int16_t ch4;
  /* left and right lever information */
  uint8_t sw1;
  uint8_t sw2;
  /* mouse movement and button information */
  keyboard_t keyboard;
  int16_t wheel;
}rc_info_t;

void get_dr16_data(rc_info_t *rc, uint8_t buff[]);
void DBus_Init(void);
void DBus_Refresh(void );

extern rc_info_t remoter;


#endif // !DBUSSYS_H
