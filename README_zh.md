# Tuya IoTOS Embedded Mcu Demo Wifi Ble STM32_LORA_LCD_RECEIVE

[English](./README.md) | [中文](./README_zh.md)

## 简介 

本Demo通过涂鸦智能云平台、涂鸦智能APP、迪文串口屏、LORA和IoTOS Embeded MCU SDK实现远程控制设备和本地控制设备，以及实时显示传感器数据。

已实现功能包括：

+ 实时显示传感器数据
+ 远程控制设备
+ 本地控制设备


## 快速上手 

### 编译与烧录
+ 下载Tuya IoTOS嵌入式代码

+ 执行Project.uvprojx文件

+ 点击软件中的编译，并完成下载


### 文件介绍 

```
├── Src
│   ├── main.c
│   ├── connect_wifi.c
│   ├── delay.c
│   ├── lcd.c
│   ├── myOS.c
│   ├── stm32l4xx_hal_msp.c
│   ├── stm32l4xx_it.c
│   ├── sx126x_v01.c
│   ├── system_stm32l4xx.c
│   ├── time.c
│   ├──usart.c
├── Inc
│   ├── main.h
│   ├── connect_wifi.h
│   ├── delay.h
│   ├── lcd.h
│   ├── myOS.h
│   ├── stm32l4xx_hal_conf.h
│   ├── stm32l4xx_it.h
│   ├── sx126x_v01.h
│   ├── time.h
│   ├── type.h
│   ├──usart.h
├── Drivers
│   ├── CMSIS
        ├── Device
           │   ├──STM32L4xx
        ├── DSP_Lib
           │   ├──Source
        ├── Include
        ├── Lib
           │   ├──ARM
           │   ├──GCC
        ├── RTOS
           │   ├──Template
│   ├── STM32L4xx_HAL_Driver
        ├── Inc
        ├── Src
└── MCU_SDK
    ├── mcu_api.c
    ├── mcu_api.h
    ├── protocol.c
    ├── protocol.h
    ├── system.c
    ├── system.h
    └── wifi.h     
```

### Demo入口

入口文件：main.c

重要函数：main()

+ 对mcu的USART，TIME，SPI，LORA，LCD等进行初始化配置，所有事件在while(1)中轮询判断。




### DP点相关

+ 上报dp点处理: mcu_dp_value_update()

| 函数名 | unsigned char mcu_dp_value_update(unsigned char dpid,unsigned long value) |
| ------ | ------------------------------------------------------------ |
| dpid   | DP的ID号                                                     |
| value  | DP数据                                                       |
| Return | SUCCESS: 成功  ERROR: 失败                                   |

+ mcu获取bool型下发dp值: mcu_get_dp_download_bool()

| 函数名  | unsigned char mcu_get_dp_download_bool(const unsigned char value[],unsigned short len) |
| ------- | ------------------------------------------------------------ |
| value[] | DP数据缓冲区地址                                             |
| len     | DP数据长度                                                   |
| Return  | 当前DP值                                                     |

### I/O 列表 

|   LCD    |          LORA          |           TIME           | UASRT2  |  UASRT3   |     GPIO     |
| :------: | :--------------------: | :----------------------: | :-----: | :-------: | :----------: |
| PA9 TXD  |        SCK/PA5         |       定时器3中断        | PA2 TXD | PC4  TXD  |     PC3      |
| PA10 RXD | MISO/PA6      MOSI/PA7 | 实现不同时刻处理不同任务 | PA3 RXD | PC5   RXD | 联网配置按键 |

## 相关文档

涂鸦Demo中心：https://developer.tuya.com/demo



## 技术支持

您可以通过以下方法获得涂鸦的支持:

- 开发者中心：https://developer.tuya.com
- 帮助中心: https://support.tuya.com/help
- 技术支持工单中心: [https://service.console.tuya.com](https://service.console.tuya.com/) 