以下是格式化后的文档内容，使用了清晰的标题、列表和代码块标记，方便阅读和理解：


# UART 通讯代码文档

## 概述
本文档介绍了基于 STM32 的 UART 通讯代码，包括初始化、数据接收和发送等功能。代码主要分为两个文件：`uart_bsp.c` 和 `uart_api.c`。

## 文件结构

### `uart_bsp.c`
包含 UART 的底层驱动回调函数：
- `HAL_UARTEx_RxEventCallback`
- `HAL_UART_ErrorCallback`

### `uart_api.c`
提供 UART 的高级 API 接口，包括：
- 初始化
- 数据接收
- 数据发送

## 接口函数说明

### 1. `uart_rx_cheak`
**功能**：检查 `uart_rx_t` 结构体的有效性。  
**参数**：  
- `uart_rx_t *uart_rx_msg`：指向 `uart_rx_t` 结构体的指针。  
**返回值**：  
- `UART_OK`：检查通过。  
- `UART_ERROR`：检查失败。  

**示例**：
```c
if (uart_rx_cheak(uart_rx_msg) == UART_ERROR) {
    // 处理错误
}
```

### 2. `uart_msg_cheak`
**功能**：检查 `uart_msg_t` 结构体的有效性。  
**参数**：  
- `uart_msg_t *rx_msg`：指向 `uart_msg_t` 结构体的指针。  
**返回值**：  
- `UART_OK`：检查通过。  
- `UART_ERROR`：检查失败。  

**示例**：
```c
if (uart_msg_cheak(rx_msg) == UART_ERROR) {
    // 处理错误
}
```

### 3. `uart_reg_cheak`
**功能**：检查 UART 实例是否已被注册。  
**参数**：  
- `uart_rx_t *uart_rx_msg`：指向 `uart_rx_t` 结构体的指针。  
**返回值**：  
- `UART_OK`：未注册。  
- `UART_ERROR`：已注册。  

**示例**：
```c
if (uart_reg_cheak(uart_rx_msg) == UART_ERROR) {
    // 处理错误
}
```

### 4. `uart_rx_init`
**功能**：初始化 UART 接收。  
**参数**：  
- `uart_rx_t *uart_rx_msg`：指向 `uart_rx_t` 结构体的指针。  
**返回值**：  
- `UART_OK`：初始化成功。  
- `UART_ERROR`：初始化失败。  

**示例**：
```c
uart_rx_t uart5_rx_msg;
// 初始化 uart5_rx_msg 结构体
if (uart_rx_init(&uart5_rx_msg) == UART_ERROR) {
    // 处理错误
}
```

### 5. `uart_tx_send_IT`
**功能**：通过中断发送数据。  
**参数**：  
- `uart_msg_t *uart_msg`：指向 `uart_msg_t` 结构体的指针。  
**返回值**：  
- `UART_OK`：发送成功。  
- `UART_ERROR`：发送失败。  

**示例**：
```c
uart_msg_t uart_msg;
// 初始化 uart_msg 结构体
if (uart_tx_send_IT(&uart_msg) == UART_ERROR) {
    // 处理错误
}
```

### 6. `uart_rx_hook_reg`
**功能**：注册 UART 接收钩子函数。  
**参数**：  
- `uart_rx_t *uart_rx_msg`：指向 `uart_rx_t` 结构体的指针。  
- `void (*hook)(uint8_t *pData, uint32_t size)`：钩子函数指针。  
**返回值**：  
- `UART_OK`：注册成功。  
- `UART_ERROR`：注册失败。  

**示例**：
```c
void my_uart5_rx_hook(uint8_t *pData, uint32_t size) {
    // 处理接收到的数据
}

if (uart_rx_hook_reg(&uart5_rx_msg, my_uart5_rx_hook) == UART_ERROR) {
    // 处理错误
}
```

## 使用示例

以下是一个完整的使用示例，展示了如何初始化 UART、注册钩子函数以及发送数据：

```c
#include "uart_api.h"
#include "usart.h"

// 定义 UART 接收消息结构体
uart_rx_t uart5_rx_msg;
uart_msg_t uart5_msg;

// 定义 UART 接收消息数组
uint8_t uart5_rx_buffer[100];
// 定义 UART 发送消息结构体
uart_msg_t uart5_tx_msg;

// 定义钩子函数
void my_uart5_rx_hook(uint8_t *pData, uint32_t size) {
    // 处理接收到的数据
    printf("Received data: %s\n", pData);
}

int main(void) {
    // 初始化 HAL 库
    HAL_Init();

    // 初始化 UART5
    uart5_msg.huart = &huart5;
    uart5_msg.pBuffer = uart5_rx_buffer; // 分配缓冲区
    uart5_msg.Len = 100;

    uart5_rx_msg.rx_msg = &uart5_msg;
    uart5_rx_msg.count = 0;
    uart5_rx_msg.time = 0;

    // 初始化 UART 接收
    if (uart_rx_init(&uart5_rx_msg) == UART_ERROR) {
        // 处理错误
    }

    // 注册钩子函数
    if (uart_rx_hook_reg(&uart5_rx_msg, my_uart5_rx_hook) == UART_ERROR) {
        // 处理错误
    }

    // 初始化 UART5 发送消息
    uart5_tx_msg.huart = &huart5;
    uart5_tx_msg.pBuffer = (uint8_t *)"Hello UART5";
    uart5_tx_msg.Len = strlen((char *)uart5_tx_msg.pBuffer);

    // 发送数据
    if (uart_tx_send_IT(&uart5_tx_msg) == UART_ERROR) {
        // 处理错误
    }

    // 主循环
    while (1) {
        // 主程序逻辑
    }
}
```

## 注意事项

1. **缓冲区管理**：  
   - 确保分配的缓冲区足够大。  
   - 在使用完毕后释放缓冲区。

2. **错误处理**：  
   - 在每个函数调用后检查返回值，确保操作成功。

3. **钩子函数**：  
   - 钩子函数应在初始化 UART 接收后注册，以确保接收到数据时能够正确处理。
