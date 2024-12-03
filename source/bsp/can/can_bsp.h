#ifndef CANTOOL_H
#define CANTOOL_H

#include "main.h"
#include "fdcan.h"

void can_bsp_init(void);
void can_filter_init(void);
uint8_t fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len);
uint8_t fdcanx_send_data_ext(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len);
uint8_t fdcanx_send_data_fd(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len);
FDCAN_RxHeaderTypeDef* fdcanx_receive(FDCAN_HandleTypeDef *hfdcan, uint8_t *buf);
void fdcan1_rx_callback(void);
void fdcan2_rx_callback(void);
void fdcan3_rx_callback(void);

#endif // !CANTOOL_H
