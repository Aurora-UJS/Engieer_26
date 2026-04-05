#ifndef REFEREE_UI_H
#define REFEREE_UI_H

#include <stdint.h>
#include "stm32h7xx_hal.h"

typedef struct
{
    uint32_t service_count;
    uint32_t send_count;
    uint32_t send_fail_count;
    uint32_t send_bytes;
    uint16_t last_robot_id;
    uint16_t last_send_len;
    HAL_StatusTypeDef last_send_status;
} referee_ui_debug_t;

void Referee_UI_Init(void);
void Referee_UI_Service(void);
void Referee_UI_RequestRefresh(void);

extern volatile referee_ui_debug_t g_referee_ui_debug;

#endif
