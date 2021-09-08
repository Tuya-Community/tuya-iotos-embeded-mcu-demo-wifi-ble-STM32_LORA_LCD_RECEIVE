使用ST的NUCLEO-L476RG开发板，WPG的SX1268模块，迪文串口屏，涂鸦云模组WB3S，进行远程控制设备和本地控制设备，以及实时显示数据。
MCU通过SPI和LORA模块通信，处理LORA模块接收到的数据。
控制屏和APP上实时显示温湿度和光照度数据，同时可以使用APP和控制屏进行控制GPIO口的电平，从而控制相应的负载设备。

MCU通过串口1和迪文屏进行通信。
MCU通过串口3和涂鸦云模组WB3S进行通信。
MCU通过串口2打印日志，例如通过LORA模块接收到的数据。

通过控制屏上的节点控制或者APP上的开关控制相应的GPIO口(PC2,PC6,PC7,PC8)。

Using ST nucleo-L476RG development board, WPG SX1268 module, Devin serial port screen, Doodle cloud module WB3S, 
remote control device and local control device, and real-time display data.  
MCU communicates with LORA module through SPI and processes the data received by LORA module.  
Temperature, humidity and illumination data can be displayed on the control panel and APP in real time. 
At the same time, APP and control panel can be used to control the level of GPIO port, so as to control the corresponding load equipment.  

The MCU communicates with diwen Screen through serial port 1.  
MCU communicates through serial port 3 and Doodle cloud module WB3S.  
The MCU prints logs through serial port 2, such as the data received through the LORA module. 

The corresponding GPIO port (PC2,PC6,PC7,PC8) is controlled by the node control on the control panel or the switch on the APP.  