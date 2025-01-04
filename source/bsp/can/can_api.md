# CAN API说明

## 1. 简介

CAN API提供了一系列的函数，用于操作CAN总线。

## 2. 基础配置

### 2.1 can数据包介绍

CAN数据包是CAN总线上发送和接收的数据单元。在发送和接收中都会用到这个数据包。

### 2.2 can数据包结构体介绍

```c
// 定义CAN消息的结构体
typedef struct
{
    uint8_t port; // CAN1_ID, CAN2_ID, CAN3_ID
    uint32_t id;
    uint8_t data[8];
    uint8_t len;
    uint8_t cnt;
}can_msg_t;
```

- port: CAN端口号，CAN1_ID, CAN2_ID, CAN3_ID
- id: CAN消息ID
- data: CAN消息数据，最大8字节
- len: CAN消息数据长度，最大8字节
- cnt: 发送计数器，每发送一次计数器加1，最大255。

在发送和接收中，都会用到这个数据包结构体。

### 3.1 can初始化

```c
void can_sys_init();
```

该函数用于初始化CAN总线。
已经加入头文件“bsp.h”中，无需再调用。

### 3.2 can数据发送

在发送数据时，需要先定义一个“can_msg_t”结构体即CAN数据包，然后调用相应的发送函数。
这个can数据包需要预先配置好CAN端口号、ID、数据、数据长度等信息。
以下是一个配置的例子：

```c
can_msg_t msg;
msg.port = CAN1_ID;
msg.id = 0x123;
msg.data[0] = 0x01;
msg.data[1] = 0x02;
msg.data[2] = 0x03;
msg.data[3] = 0x04;
msg.data[4] = 0x05;
msg.data[5] = 0x06;
msg.data[6] = 0x07;
msg.data[7] = 0x08;
msg.len = FDCAN_DLC_BYTES_8;

```

```c
// 发送经典CAN消息
can_status_t can_msg_send_classical(can_msg_t *msg);

// 发送扩展CAN消息
can_status_t can_msg_send_extended(can_msg_t *msg);

// 发送fdCAN消息
can_status_t can_msg_send_fd(can_msg_t *msg);
```

### 3.3 can数据接收

要配置can接收消息，需要先配置一个“can_msg_t”结构体，然后调用can_msg_add_item函数，将需要收的ID添加到消息列表中。

```c

// 向消息列表中添加消息项
can_status_t can_msg_add_item(can_msg_t *msg);
```
