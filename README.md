# Tuya IoTOS Embedded Mcu Demo Wifi Ble STM32_LORA_LCD_RECEIVE

[English](./README.md) | [中文](./README_zh.md)

## Introduction  

This Demo uses the Tuya smart cloud platform, Tuya smart APP,  Divin serial screen, LORA and IoTOS Embeded MCU SDK to achieve remote control and local control devices, as well as real-time display of sensor data.

The implemented features include:

+ Display Sensor Data In Real Time
+ Remote Control Equipment
+ Local Control Equipment


## Quick start  

### Compile & Burn
+ Download Tuya IoTOS Embeded Code
+ Execute the Project.uvprojx file
+ Click Compile in the software and complete the download


### File introduction 

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



### Demo entry

Entry file：main.c

Important functions：main()

+ Initialize and configure MCU USART，TIME，SPI，LORA，LCD, etc. All events are polled and judged in while(1)。




### DataPoint related

+ DP point processing: mcu_dp_value_update()

| function name | unsigned char mcu_dp_value_update(unsigned char dpid,unsigned long value) |
| ------------- | ------------------------------------------------------------ |
| dpid          | DP ID number                                                 |
| value         | DP data                                                      |
| Return        | SUCCESS: Success ERROR: Failure                              |

+ MCU gets the dp value of the bool type:mcu_get_dp_download_bool()

| function name | unsigned char mcu_get_dp_download_bool(const unsigned char value[],unsigned short len) |
| ------------- | ------------------------------------------------------------ |
| value[]       | DP data buffer address                                       |
| len           | DP data length                                               |
| Return        | The current values of dp                                     |

### I/O List  

|   LCD    |          LORA          |                    TIME                     | UASRT2  |  UASRT3   |             GPIO             |
| :------: | :--------------------: | :-----------------------------------------: | :-----: | :-------: | :--------------------------: |
| PA9 TXD  |        SCK/PA5         |              Timer 3 interrupt              | PA2 TXD | PC4  TXD  |             PC3              |
| PA10 RXD | MISO/PA6      MOSI/PA7 | Implement Different Tasks At DifferentTimes | PA3 RXD | PC5   RXD | Network Configuration Button |

## Related Documents

 Tuya Demo Center: https://developer.tuya.com/demo



## Technical Support

  You can get support for Tuya by using the following methods:

- Developer Center: https://developer.tuya.com
- Help Center: https://support.tuya.com/help
- Technical Support Work Order Center: [https://service.console.tuya.com](https://service.console.tuya.com/) 

