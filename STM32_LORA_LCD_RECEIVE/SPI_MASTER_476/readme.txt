ʹ��ST��NUCLEO-L476RG�����壬WPG��SX1268ģ�飬���Ĵ�������Ϳѻ��ģ��WB3S������Զ�̿����豸�ͱ��ؿ����豸���Լ�ʵʱ��ʾ���ݡ�
MCUͨ��SPI��LORAģ��ͨ�ţ�����LORAģ����յ������ݡ�
��������APP��ʵʱ��ʾ��ʪ�Ⱥ͹��ն����ݣ�ͬʱ����ʹ��APP�Ϳ��������п���GPIO�ڵĵ�ƽ���Ӷ�������Ӧ�ĸ����豸��

MCUͨ������1�͵���������ͨ�š�
MCUͨ������3��Ϳѻ��ģ��WB3S����ͨ�š�
MCUͨ������2��ӡ��־������ͨ��LORAģ����յ������ݡ�

ͨ���������ϵĽڵ���ƻ���APP�ϵĿ��ؿ�����Ӧ��GPIO��(PC2,PC6,PC7,PC8)��

Using ST nucleo-L476RG development board, WPG SX1268 module, Devin serial port screen, Doodle cloud module WB3S, 
remote control device and local control device, and real-time display data.  
MCU communicates with LORA module through SPI and processes the data received by LORA module.  
Temperature, humidity and illumination data can be displayed on the control panel and APP in real time. 
At the same time, APP and control panel can be used to control the level of GPIO port, so as to control the corresponding load equipment.  

The MCU communicates with diwen Screen through serial port 1.  
MCU communicates through serial port 3 and Doodle cloud module WB3S.  
The MCU prints logs through serial port 2, such as the data received through the LORA module. 

The corresponding GPIO port (PC2,PC6,PC7,PC8) is controlled by the node control on the control panel or the switch on the APP.  