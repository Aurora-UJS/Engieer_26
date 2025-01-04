# 大疆遥控器模块使用说明

## 1. 示例代码

```C
{
    void DBus_Init(void)

    // 开始无限循环，持续控制电机
    while (1)
    {
         DBus_Refresh();
    }
}

```

## 2.调用

```C
##include "Dbus.h"

    remoter.ch1;
    remoter.mouse.x;
    remoter.kb.bit.w;

```
