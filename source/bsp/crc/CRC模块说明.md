# CRC校验模块说明文档

## 1. 功能概述
本模块提供CRC8和CRC16校验功能，包括：
- 计算数据的CRC校验值
- 验证数据的CRC校验值
- 为数据追加CRC校验值

## 2. API说明

### CRC8相关函数
```c
// 计算CRC8校验和
unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength, unsigned char ucCRC8);

// 验证CRC8校验和 
unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);

// 追加CRC8校验和到数据末尾
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
```

### CRC16相关函数  
```c
// 计算CRC16校验和
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC);

// 验证CRC16校验和
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);

// 追加CRC16校验和到数据末尾
void Append_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
```

## 3. 使用示例

### CRC8使用示例
```c
unsigned char data[10] = {0x01, 0x02, 0x03, 0x04};
unsigned int length = 4;

// 计算校验和
unsigned char crc = Get_CRC8_Check_Sum(data, length, CRC8_INIT);

// 验证校验和
if(Verify_CRC8_Check_Sum(data, length + 1)) {
    // 校验通过
}

// 追加校验和
Append_CRC8_Check_Sum(data, length + 1);
```

### CRC16使用示例
```c
uint8_t data[10] = {0x01, 0x02, 0x03, 0x04};
uint32_t length = 4;

// 计算校验和
uint16_t crc = Get_CRC16_Check_Sum(data, length, CRC_INIT);

// 验证校验和  
if(Verify_CRC16_Check_Sum(data, length + 2)) {
    // 校验通过
}

// 追加校验和
Append_CRC16_Check_Sum(data, length + 2);
```

## 4. 注意事项
1. CRC8校验和占用1字节，CRC16校验和占用2字节
2. 验证和追加函数需要确保数据缓冲区有足够空间存放校验和
3. CRC8初始值为0xff，CRC16初始值为0xffff
4. 校验表已预计算并存储在模块中
5. 数据长度参数不包含校验和本身的长度
