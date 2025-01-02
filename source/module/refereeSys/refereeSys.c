#include "refereeSys.h"
#include "uart_api.h"


uart_msg_t refereeSys_tx_msg;

void refereeSys_init(uart_rx_t *rx_callback)
{
    if (uart_rx_cheak(rx_callback)!= UART_OK)
    {
        while (1)
        {
            
        }
        
    }
    refereeSys_tx_msg.huart = rx_callback->rx_msg->huart;
}