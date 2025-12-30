#include "cmsis_os2.h"
#include "uart_api.h"
#include "usart.h"
#include <string.h>

uart_rx_t testUart_msg;
uart_msg_t testUart_rx_msg;
uart_msg_t testUart_tx_msg;
uint8_t testUartBuffer[18];
uint32_t lastCount;

extern void (*uart7_rx_hook)(uint8_t *pData, uint32_t size);

void uart7_test_hook(uint8_t *buf, uint32_t len)
{
    HAL_UART_Transmit(&huart7, buf, len, 100);
}

void testUart_rx_init(void){
    testUart_msg.rx_msg =  &testUart_rx_msg;
    testUart_msg.rx_msg-> pBuffer  = testUartBuffer;
    testUart_msg.rx_msg->huart = &huart7; 
    testUart_msg.rx_msg->Len = 18;
    uart7_rx_hook = uart7_test_hook;
    uart_rx_init(&testUart_msg);
}
void testUart_tx_init(void){
    testUart_tx_msg.huart= &huart7;
    testUart_tx_msg.pBuffer = (uint8_t *)"hello\r\n";
    testUart_tx_msg.Len = strlen((char *) testUart_tx_msg.pBuffer);
}


void uart_Refresh(void)
{
    if (testUart_msg.count != lastCount)
    {

        lastCount = testUart_msg.count;
    }
}
void uart_test(void *arguments)
{
    UNUSED(arguments);
    osDelay(10);
    testUart_rx_init();
    testUart_tx_init();
    while (1) {
        uart_tx_send_IT(&testUart_tx_msg);
        osDelay(500);
    }
}
