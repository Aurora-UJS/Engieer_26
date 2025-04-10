# 河南科技大学鼎行双创战队电控组基础开发库

本项目为河南科技大学鼎行双创战队电控组基础开发库，用于开发达妙动力MC02开发板。
更多代码内容参看[代码文档(html)](doc/html/index.html)

## 系统配置
主要系统配置参数参见[[config.h](source/config.h)]

## 模块文档

### BSP层 (硬件抽象层)

- [蜂鸣器部分](source/bsp/buzzer/buzzer_bsp.md)
- [CAN通讯部分](source/bsp/can/can_api.md)
- [DWT部分](source/bsp/dwt/dwt_bsp.md)
- [UART通讯部分](source/bsp/uart/uart_api.md)
- [BMI088 IMU部分](source/bsp/BMI088/BMI088.md)
- [CRC模块](source/bsp/crc/CRC模块说明.md)
- [键盘模块](source/bsp/keyboard/键盘模块说明.md)

### 模块层

#### 计算机模块
- [计算机系统](source/module/computer/computer_sys.md)

#### 控制模块
- [控制系统](source/module/control/control_module.md)
- [电源控制](source/module/control/PowerControl/PowerControl.md)

#### 其他模块
- [IMU模块](source/module/IMU/IMU_module.md)
- [大疆电机控制](source/module/motor/motor_Dji/motor_DJI.md)
- [达妙电机控制](source/module/motor/motor_DM/motor_DM.md)
- [滤波器](source/module/filter/filter_module.md)
- [裁判系统](source/module/refereeSys/refereeSys_module.md)
- [超级电容](source/module/superCap/XidiNewise/super_str.md)

## 有关代码规范

见[代码规范](code_style.md)