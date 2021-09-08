/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "sx126x_v01.h"
#include <time.h>
#include <stdlib.h>


/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
//��������

#define LORA_MODE	0
#define FSK_MODE    1


#define TRANSMITTER     0
#define RECEIVER        1

#define RX_CONTINOUS    1   //��������

#if (TRANSMITTER == RECEIVER)
    #error "Please define only Transmitter or receiver."
#endif

#define TEST_MODE	3  	//0-infinite preamble TX mode����ʱֻ������lora��
						//1-continous CW TX 
						//����ֵ���ܽ��뵽�շ�����ģʽ
						


#if (LORA_MODE == FSK_MODE)
    #error "Please define only LoRa or FSK."
#endif






#define TX_OUTPUT_POWER                             10        // dBm  //�������18.536
#define RF_FREQUENCY                                490000000//480000000//915000000//470000000 // Hz


#if (FSK_MODE==1)

#define FSK_FDEV                                    25e3      // Hz 
#define FSK_DATARATE                                50e3      // bps
#define FSK_BANDWIDTH                               100e3     // Hz >> DSB in sx126x
#define FSK_AFC_BANDWIDTH                           100e3     // Hz
#define FSK_PREAMBLE_LENGTH                         5         // Same for Tx and Rx
#define FSK_FIX_LENGTH_PAYLOAD_ON                   false

#elif (LORA_MODE==1)

#define LORA_BANDWIDTH                              0        // [0: 125 kHz,
															//	1: 250 kHz, 														 
															//	2: 500k
															//	3 :20.83kHz
															//	 4:31.25kHz
															//	5:62.5kHz4
															//6:41.67
#define LORA_SPREADING_FACTOR                       12         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx  SF5&6 will automatilly change to 12
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false

#endif


#define HAL_MAX_DELAY      0xFFFFFFFFU

#define RX_TIMEOUT_VALUE                            1000
#define TX_TIMEOUT                                  65535 
#define BUFFER_SIZE                                 250//10//250 // Define the payload size here

#define CADTIMEOUT_MS								2000   //CAD timeout ʱ��  ��ms��ʾ

uint8_t dat;
uint8_t cnt=0x55;
uint8_t recdat=0;
uint8_t version=0;

uint16_t BufferSize = BUFFER_SIZE;
uint8_t Buffer[BUFFER_SIZE]={0};

int8_t RssiValue = 0;
int8_t SnrValue = 0;

PacketStatus_t RadioPktStatus;
uint8_t RadioRxPayload[255];
uint8_t RadioRxPacketSize;



uint8_t SendCnt=0;


volatile bool TXDone=false;
volatile bool RXDoneFlag=false;
volatile bool TimeOutFlag=false;
volatile bool CRCFail=false;




volatile int Cnt1=0;

const RadioLoRaBandwidths_t Bandwidths_copy[] = { LORA_BW_125, LORA_BW_250, LORA_BW_500,LORA_BW_020,LORA_BW_031,LORA_BW_062,LORA_BW_041 };


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
  uint8_t i=0;
  bool DetectTruetable[100]={0};//CAD�ɹ��ķֲ�
  bool RXTruetable[100]={0};//CAD���ܽ�����ȷ�ķֲ�
  uint8_t CadDetectedTime=0;//��⵽��cad�Ĵ���
  uint8_t RxCorrectTime=0;//RX ������ȷ����
  uint8_t TxTime=0;		//TX ����
  int random_number=0;
  RadioStatus_t RadioStatus;
  uint8_t L1_1st_gainstep=0;

	//�������͵�ʱ����
	uint8_t ModulationParam[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    uint8_t PacketParam[9] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */

  for(i=0;i<1;i++)
  {
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
	HAL_Delay(1000);
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
	HAL_Delay(1000);
  }
  //HAL_Delay(2000);

  SX126xReset();
  //HAL_Delay(1000);
  i=SX126xReadRegister(REG_LR_CRCSEEDBASEADDR);
  if(i==0x1D)
  {   
	printf("SPI SUCCESS!\n\r");
  }
  else
  {
	printf("SPI Fail! REG_LR_CRCSEEDBASEADDR=%x\n\r",i);
  }
  //while(1);
  RadioInit();
  SX126xWriteRegister(0x889, SX126xReadRegister(0x889) & 0xfB);//SdCfg0 (0x889) sd_res (bit 2) = 0 
  printf("RadioInit Done!\n\r");


#if (TEST_MODE==0)   //infinite preamble TX mode
	//��������
	SX126xSetStandby( STDBY_RC );
	SX126xSetPacketType(PACKET_TYPE_LORA);//todo: ���ӷ���FSKģʽ�µĸ�ָ��
	
	printf("set lora params\n");
	ModulationParam[0]=LORA_SPREADING_FACTOR;
	ModulationParam[1]=Bandwidths_copy[LORA_BANDWIDTH];
	ModulationParam[2]=LORA_CODINGRATE;
	ModulationParam[3]=0;//1:SF11 and SF12 0:���� �������Ż�  
	SX126xWriteCommand( RADIO_SET_MODULATIONPARAMS, ModulationParam, 4 );//lora�����������


	//����lora������
	PacketParam[0]=(LORA_PREAMBLE_LENGTH>>8)& 0xFF;
	PacketParam[1]=LORA_PREAMBLE_LENGTH;
	PacketParam[2]=LORA_FIX_LENGTH_PAYLOAD_ON;//head type
	PacketParam[3]=0xFF;//0Xff is MaxPayloadLength
	PacketParam[4]=true;//CRC on
	PacketParam[5]=LORA_IQ_INVERSION_ON;
	SX126xWriteCommand( RADIO_SET_PACKETPARAMS, PacketParam, 6 );

	//SX126xWriteBuffer( 0x00, SendData, 10 );

	//��������lora
	SX126xSetRfFrequency( RF_FREQUENCY );
    SX126xSetRfTxPower( TX_OUTPUT_POWER );
	SX126xSetTxInfinitePreamble();

	printf("TxContinuousWave Now--infinite preamble!\n\r");
	while(1);
#elif (TEST_MODE==1) //TX CW

	RadioSetTxContinuousWave( RF_FREQUENCY, TX_OUTPUT_POWER, TX_TIMEOUT );
	printf("TxContinuousWave Now---CW!\n\r");
	while(1);

#endif


#if (FSK_MODE==1)

	SX126xSetRfFrequency(RF_FREQUENCY);
	RadioSetTxConfig( MODEM_FSK, TX_OUTPUT_POWER, FSK_FDEV, 0,
						FSK_DATARATE, 0,
						FSK_PREAMBLE_LENGTH, FSK_FIX_LENGTH_PAYLOAD_ON,
						true, 0, 0, 0, 3000 );
	
	RadioSetRxConfig( MODEM_FSK, FSK_BANDWIDTH, FSK_DATARATE,
						0, FSK_AFC_BANDWIDTH, FSK_PREAMBLE_LENGTH,
						0, FSK_FIX_LENGTH_PAYLOAD_ON, 0, true,0, 0,false, RX_CONTINOUS );
	
	//printf("FSK:%d,Fdev=%ld,BitRate=%ld,BW=%ld,PWR=%d,PreLen=%d,PYLOAD=%d\n\r",RF_FREQUENCY,FSK_FDEV,FSK_DATARATE,FSK_BANDWIDTH,TX_OUTPUT_POWER,FSK_PREAMBLE_LENGTH,BUFFER_SIZE);
	printf("configure FSK parameters done\n!");

#elif (LORA_MODE==1)

			SX126xSetRfFrequency(RF_FREQUENCY);
			RadioSetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
									   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
									   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
									   true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );


		  RadioSetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
										 LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
										 LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
										 0, true, 0, 0, LORA_IQ_INVERSION_ON, RX_CONTINOUS );//���һ�����������Ƿ�����������


		
		 printf("LORA:%d,SF=%d,codeRate=%d,BW=%d,PWR=%d,PreLen=%d,PYLOAD=%d\n\r",RF_FREQUENCY,LORA_SPREADING_FACTOR,LORA_CODINGRATE,LORA_BANDWIDTH,TX_OUTPUT_POWER,LORA_PREAMBLE_LENGTH,BUFFER_SIZE);
		// { true,false }--public,{ false,false }--private
		 if (RadioPublicNetwork.Previous==true && RadioPublicNetwork.Current==false)
			printf("public\n\r");
		 else if (RadioPublicNetwork.Previous==false && RadioPublicNetwork.Current==false)
			printf("private\n\r");
		printf("configure LORA parameters done\n!");
		 
		 		 
		 ///tx filter CPL=7
		//printf("before modification:reg0x745=0x%x\n",SX126xReadRegister(0x745));
		//SX126xWriteRegister(0x745, SX126xReadRegister(0x745) | 0x6);
		//printf("after modification:reg0x745=0x%x\n",SX126xReadRegister(0x745));

		 
		 
		 //timeout=0�����ν��գ����ǻ����ж��Ƿ����������գ�������ȼ����
		  //RadioRx(3000);//�������� ֻҪ����������һ�ξͿ���  �м䲻��sleep ����Ҫ����ִ��RadioRx();
		  //printf("begin to RX--lora packets mode\n!");
#endif


	//SX126xSetRxTxFallbackMode(0x30);//�˻ص�STANDBY_XOSC
	//printf("SX126xSetRxTxFallbackMode to STANDBY_XOSC\n!");
	
	//��������ʱʹ�����
	 //while(1);



	/*	
  //ana_gain_mode=2
  SX126xWriteRegister(0xA, SX126xReadRegister(0xA) & 0xEF);//(bit 4) = 0 
  SX126xWriteRegister(0xA, SX126xReadRegister(0xA) | 0x20);//(bit 5) = 1 
  //gain_step_mc
  SX126xWriteRegister(0xA, SX126xReadRegister(0xA) & 0xFE);//(bit(3..0)) = 1111B 
  SX126xWriteRegister(0xA, SX126xReadRegister(0xA) | 0x01);//(bit(3..0)) = 1111B 
	*/
  

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

#if (TRANSMITTER==1)
	//printf("before modification:reg0x745=0x%x\n",SX126xReadRegister(0x745));
	//SX126xWriteRegister(0x745, SX126xReadRegister(0x745) | 0x7);
	//printf("after modification:reg0x745=0x%x\n",SX126xReadRegister(0x745));
		
	while(1)
	{
		Buffer[0] = TxTime++;
	    Buffer[1] = 0;
	    Buffer[2] = 0;
	    Buffer[3] = 0;
	    Buffer[4] = 0;
	    Buffer[5] = 0;
		RadioSend(Buffer,6);
		while(TXDone==false && TimeOutFlag==false);//һֱ�ȴ�tx done
		TXDone=false;
		TimeOutFlag=false;
		printf("TxTime=%d\n",TxTime);
		HAL_Delay(1000); ///1s

		//��ȡ״̬
		RadioStatus=SX126xGetStatus();
		printf("RadioStatus is(after TX_DONE) %d\n",(((RadioStatus.Value)>>4)&0x07));
		
		
	}
#elif (RECEIVER==1) 
	/*
		///RSSI ά����һ�̶�ֵ
		printf("before modification:AgcHystCtrl=0x%x,AgcRssiCtrl0=0x%x,AgcMode=0x%x\n",SX126xReadRegister(0x8B4),SX126xReadRegister(0x89B),SX126xReadRegister(0x8A5));
		//ana_fe_lv.AgcHystCtrl.dynamic_hyst = 0
		SX126xWriteRegister(0x8B4, SX126xReadRegister(0x8B4) & 0xFE);
		//ana_fe_lv.AgcRssiCtrl0.rssi_length = 7
		SX126xWriteRegister(0x89B, SX126xReadRegister(0x89B) | 0x1C);
		//ana_fe_lv.AgcMode.agc_ignore_pkt_det = 1
		SX126xWriteRegister(0x8A5, SX126xReadRegister(0x8A5) | 0x10);
		printf("after modification:AgcHystCtrl=0x%x,AgcRssiCtrl0=0x%x,AgcMode=0x%x\n",SX126xReadRegister(0x8B4),SX126xReadRegister(0x89B),SX126xReadRegister(0x8A5));
		////
	*/
	/*

		///RX ��blokc��ָ�
		//set AgcL1Ctrl.disable_L1 to 1. This will force AGC to leave ADC saturation control autonomous, instead of forcing a smallest gain G1. A way may be closer to 1278.
		//SX126xWriteRegister(0x8B5, SX126xReadRegister(0x8B5) | 0x01);

		//AgcL1Ctrl.L1_1st_gainstep (addr 0x8B5, bit 5 downto 2)
		//L1_1st_gainstep=4;
		//printf("before modification:0x8B5=0x%x\n",SX126xReadRegister(0x8B5));
		//SX126xWriteRegister(0x8B5, (SX126xReadRegister(0x8B5) & 0xC3)|(L1_1st_gainstep<<2));
		//printf("after modification:0x8B5=0x%x\n",SX126xReadRegister(0x8B5));
	*/


		//�����Ƶ�޸Ĵ���������

		
	

	while(1)
	{

#if (RX_CONTINOUS==1)
		//��ʼ����
		RadioRx(0xFFFFFF);//50MS(0XC80)��ʱ  0-���ν��� �޳�ʱ
		printf("continous RX...\n");
		while(1);//��������
#endif
		RadioRx(2000);//50MS(0XC80)��ʱ  0-���ν��� �޳�ʱ
		while(RXDoneFlag==false && TimeOutFlag==false && CRCFail==false);
		if(RXDoneFlag==true || TimeOutFlag==true || CRCFail==true)
		{
			if(CRCFail==false)	//CRC�޴���
			{
				if(RXDoneFlag==true)
				{
					printf("\n%d:RxCorrect-PING\n",RxCorrectTime);
					
					//HAL_GPIO_WritePin(RX_OK_PLUS_GPIO_Port,RX_OK_PLUS_Pin,GPIO_PIN_SET);
					//HAL_GPIO_WritePin(RX_OK_PLUS_GPIO_Port,RX_OK_PLUS_Pin,GPIO_PIN_RESET);

					RxCorrectTime++;
				}
			}
			

			CRCFail=false;
			RXDoneFlag=false;
			TimeOutFlag=false;
		}
		
	}
#endif		

 }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the main internal regulator output voltage 
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, SW_CTL1_Pin|SW_CTL2_Pin|TrigIO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, NRESET_Pin|DeviceSel_Pin|SPI_CS_Pin|ANT_SWITCH_POWER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SW_CTL1_Pin SW_CTL2_Pin TrigIO_Pin */
  GPIO_InitStruct.Pin = SW_CTL1_Pin|SW_CTL2_Pin|TrigIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : NRESET_Pin DeviceSel_Pin SPI_CS_Pin ANT_SWITCH_POWER_Pin */
  GPIO_InitStruct.Pin = NRESET_Pin|DeviceSel_Pin|SPI_CS_Pin|ANT_SWITCH_POWER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BUSY_Pin */
  GPIO_InitStruct.Pin = BUSY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUSY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DIO1_Pin */
  GPIO_InitStruct.Pin = DIO1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DIO1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

	//printf("ISR mode=%d!\n",(uint8_t)SX126xGetOperatingMode());
	SX126xOnDio1Irq();

}
//DIO1���жϺ���
void SX126xOnDio1Irq(void)
{
	 uint16_t irqRegs = SX126xGetIrqStatus( );
     SX126xClearIrqStatus( IRQ_RADIO_ALL );//��������жϱ�־
     //���ͽ���
	 if( ( irqRegs & IRQ_TX_DONE ) == IRQ_TX_DONE )
        {
// 			TimerStop( &TxTimeoutTimer );
//          if( ( RadioEvents != NULL ) && ( RadioEvents->TxDone != NULL ) )
//          {
//               RadioEvents->TxDone( );
//          }
		   TXDone=true;
		   HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
		   OnTxDone();
			
        }

		//��SX126xSetTx()������һ����ʱʱ�� ���Լ��Ĺ��� --ok
 		if( ( irqRegs & IRQ_RX_TX_TIMEOUT ) == IRQ_RX_TX_TIMEOUT )
        {
// 			TimerStop( &TxTimeoutTimer );
//          if( ( RadioEvents != NULL ) && ( RadioEvents->TxDone != NULL ) )
//          {
//               RadioEvents->TxDone( );
//          }
		   TimeOutFlag=true;
		   printf(" RX/TX timeout\n");
			
        }


        

        if( ( irqRegs & IRQ_RX_DONE ) == IRQ_RX_DONE )
        {
            

            //TimerStop( &RxTimeoutTimer );
            SX126xGetPayload( RadioRxPayload, &RadioRxPacketSize , 255 );
            SX126xGetPacketStatus( &RadioPktStatus );
//            if( ( RadioEvents != NULL ) && ( RadioEvents->RxDone != NULL ) )
//            {

				HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
                OnRxDone();
                RXDoneFlag=true;
//            }
      }

        if( ( irqRegs & IRQ_CRC_ERROR ) == IRQ_CRC_ERROR )
        {
            //if( ( RadioEvents != NULL ) && ( RadioEvents->RxError ) )
            //{
                //RadioEvents->RxError( );
                printf("CRC fail\n");
                CRCFail=true;
            //}
        }

        if( ( irqRegs & IRQ_CAD_DONE ) == IRQ_CAD_DONE )
        {
//            if( ( RadioEvents != NULL ) && ( RadioEvents->CadDone != NULL ) )
//            {
//               RadioEvents->CadDone( ( ( irqRegs & IRQ_CAD_ACTIVITY_DETECTED ) == IRQ_CAD_ACTIVITY_DETECTED ) );
//            }
//			printf("IRQ_CAD_DONE\n");
//			CadDone=true;
			if ( ( irqRegs & IRQ_CAD_ACTIVITY_DETECTED ) == IRQ_CAD_ACTIVITY_DETECTED ) 
			{
//				printf("IRQ_CAD_ACTIVITY_DETECTED\n");	
//				CadDetect=true;
			}
//            
        }

//        if( ( irqRegs & IRQ_RX_TX_TIMEOUT ) == IRQ_RX_TX_TIMEOUT )
//        {
//            if( SX126xGetOperatingMode( ) == MODE_TX )
//            {
//                TimerStop( &TxTimeoutTimer );
//                if( ( RadioEvents != NULL ) && ( RadioEvents->TxTimeout != NULL ) )
//                {
//                    RadioEvents->TxTimeout( );
//                }
//            }
//            else if( SX126xGetOperatingMode( ) == MODE_RX )
//            {
//                TimerStop( &RxTimeoutTimer );
//                if( ( RadioEvents != NULL ) && ( RadioEvents->RxTimeout != NULL ) )
//                {
//                    RadioEvents->RxTimeout( );
//                }
//            }
//        }

        if( ( irqRegs & IRQ_PREAMBLE_DETECTED ) == IRQ_PREAMBLE_DETECTED )
        {
            __NOP( );
        }

        if( ( irqRegs & IRQ_SYNCWORD_VALID ) == IRQ_SYNCWORD_VALID )
        {
            __NOP( );
        }

        if( ( irqRegs & IRQ_HEADER_VALID ) == IRQ_HEADER_VALID )
        {
            __NOP( );
        }

//        if( ( irqRegs & IRQ_HEADER_ERROR ) == IRQ_HEADER_ERROR )
//        {
//            TimerStop( &RxTimeoutTimer );
//            if( ( RadioEvents != NULL ) && ( RadioEvents->RxTimeout != NULL ) )
//            {
//                RadioEvents->RxTimeout( );
//            }
//        }
    
}
//������������
void GenTrig(void)
{
	HAL_GPIO_WritePin(TrigIO_GPIO_Port,TrigIO_Pin,GPIO_PIN_SET);
	
	HAL_GPIO_WritePin(TrigIO_GPIO_Port,TrigIO_Pin,GPIO_PIN_RESET);

	
}

void OnTxDone(void)
{
	SleepParams_t params = { 0 };

    params.Fields.WarmStart = 1;//������
    //printf("params.value=%d\n",params.Value);
    //SX126xSetSleep( params );//���������Ա������˯��ǰ��״̬����ؼĴ���  sleep������������жϱ�־λ
	printf("OnTxDone\n");

}
void OnRxDone()
{

	int i;
	
	printf("onRXDone\n");
    SleepParams_t params = { 0 };

    params.Fields.WarmStart = 1;//������

//	printf("before sleep,operate mode=%d\n",SX126xGetOperatingMode());

  //  SX126xSetSleep( params );//

//	printf("after sleep,operate mode=%d\n",SX126xGetOperatingMode());
    //BufferSize = size;

	//TODO:�����������
	
	

	
	printf("PacketSIZE=%d,RSSI=%d,SNR=%d\n",
			RadioRxPacketSize,
					RadioPktStatus.Params.LoRa.RssiPkt, 
							RadioPktStatus.Params.LoRa.SnrPkt);

	printf("Payload: ");
	for(i=0;i<RadioRxPacketSize;i++)
	{
		printf("0x%x ",RadioRxPayload[i]);
	}

	//for(k=0;k<RadioRxPacketSize;k++)
    //	printf("%d ",RadioRxPayload[k]);
    //printf("RX OK=%d\n",++Cnt1);
    //RssiValue = rssi;
    //SnrValue = snr;
   // State = RX;
}


/*******************************************************************************
�������ƣ�fputc(int ch, FILE *f)
��    �ܣ�����ʵ��Printf()��������
��    ������
����ֵ  ��
********************************************************************************/

int fputc(int ch, FILE *f)
{
      //USART_SendData(USART1, (unsigned char) ch);
      HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 20);
	  while(HAL_UART_GetState(&huart2)==HAL_UART_STATE_RESET);
      return (ch);
}




/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
