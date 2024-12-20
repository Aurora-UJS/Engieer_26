#ifndef __BUZZER_BSP_H__
#define __BUZZER_BSP_H__
#include "main.h"

#define   SOUND_L1     262-1//低调 do 的频率
#define   SOUND_L2     294-1//低调 re 的频率
#define   SOUND_L3     330-1//低调 mi 的频率
#define   SOUND_L4     350-1//低调 fa 的频率
#define   SOUND_L5     392-1//低调 sol 的频率
#define   SOUND_L6     440-1//低调 la 的频率
#define   SOUND_L7     494-1//低调 si 的频率
                                     
#define   SOUND_M1     524-1//中调 do 的频率
#define   SOUND_M2     588-1//中调 re 的频率
#define   SOUND_M3     660-1//中调 mi 的频率
#define   SOUND_M4     700-1//中调 fa 的频率
#define   SOUND_M5     784-1//中调 sol 的频率
#define   SOUND_M6     880-1//中调 la 的频率
#define   SOUND_M7     988-1//中调 si 的频率

#define   SOUND_H1     1048-1//高调 do 的频率
#define   SOUND_H2     1176-1//高调 re 的频率
#define   SOUND_H3     1320-1//高调 mi 的频率
#define   SOUND_H4     1480-1//高调 fa 的频率
#define   SOUND_H5     1640-1//高调 sol 的频率
#define   SOUND_H6     1760-1//高调 la 的频率
#define   SOUND_H7     1976-1//高调 si 的频率

#define   SOUND_S      0//不发音

void buzzer_init(void);
void buzzer_on(uint32_t frequency, float duty_cycle);
void buzzer_off(void);

#endif /* __BUZZER_BSP_H__ */