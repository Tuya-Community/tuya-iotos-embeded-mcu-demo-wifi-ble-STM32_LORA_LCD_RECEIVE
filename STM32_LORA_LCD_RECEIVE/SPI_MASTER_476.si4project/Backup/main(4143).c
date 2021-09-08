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
//连续发送


#define TEST_MODE	3  //0-continous CW
						//1-continous preamble
						//2-lora packets TX
						//其他值是lora packets RX

#if (TEST_MODE==0)					//连续CW							
	#define CONTINUE_MODE									
	#define CONTINUE_TX_TYPE									0x00//01-LORA 00-GFSK
#elif (TEST_MODE==1)			//无限前导
	#define CONTINUE_MODE									
	#define CONTINUE_TX_TYPE									0x01//01-LORA 00-GFSK
#elif (TEST_MODE==2)		//包模式发送
	#define TX_LORA
#endif									//其他就是接收

//FSK or LORA (包模式)
//#define TXFSK
#define FSK_BANDWIDTH 234300
//在ping的基础上只发
 

#define HAL_MAX_DELAY      0xFFFFFFFFU

#define TX_OUTPUT_POWER                             22        // dBm  //测出来是18.536


#define LORA_BANDWIDTH                              0        // [0: 125 kHz,
															//	1: 250 kHz, 														 
															//	2: 500k
															//	3 :20.83kHz
															//	 4:31.25kHz
															//	5:62.5kHz4
															//6:41.67



#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx  SF5&6 will automatilly change to 12
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false

#define RX_TIMEOUT_VALUE                            1000

#define RF_FREQUENCY                                501000000//480000000//915000000//470000000 // Hz


#define TX_TIMEOUT                                  65535 
#define BUFFER_SIZE                                 20//10//250 // Define the payload size here

#define CADTIMEOUT_MS								2000   //CAD timeout 时间  用ms表示

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
  bool DetectTruetable[100]={0};//CAD成功的分布
  bool RXTruetable[100]={0};//CAD后能接收正确的分布
  uint8_t CadDetectedTime=0;//检测到的cad的次数
  uint8_t RxCorrectTime=0;//RX 接收正确次数
  uint8_t TxTime=0;		//TX 次数
  int random_number=0;
  RadioStatus_t RadioStatus;

	//连续发送的时候用
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

#ifdef CONTINUE_MODE
	//连续发送
	SX126xSetStandby( STDBY_RC );
	SX126xSetPacketType(CONTINUE_TX_TYPE);
	printf("type=%d\n",SX126xGetPacketType());

	if(SX126xGetPacketType()==0x01) //lora连续发送
	{
		//设置lora调制参数,FSK连续发送时不设置
		printf("set lora params\n");
		ModulationParam[0]=LORA_SPREADING_FACTOR;
		ModulationParam[1]=LORA_BANDWIDTH;
		ModulationParam[2]=LORA_CODINGRATE;
		ModulationParam[3]=0;//1:SF11 and SF12 0:其他 低速率优化  
		SX126xWriteCommand( RADIO_SET_MODULATIONPARAMS, ModulationParam, 4 );//lora发射参数配置


		//设置lora包参数
		PacketParam[0]=(LORA_PREAMBLE_LENGTH>>8)& 0xFF;
		PacketParam[1]=LORA_PREAMBLE_LENGTH;
		PacketParam[2]=LORA_FIX_LENGTH_PAYLOAD_ON;//head type
		PacketParam[3]=0xFF;//0Xff is MaxPayloadLength
		PacketParam[4]=true;//CRC on
		PacketParam[5]=LORA_IQ_INVERSION_ON;
		SX126xWriteCommand( RADIO_SET_PACKETPARAMS, PacketParam, 6 );

		//SX126xWriteBuffer( 0x00, SendData, 10 );

		//连续发送lora
		SX126xSetRfFrequency( RF_FREQUENCY );
	    SX126xSetRfTxPower( TX_OUTPUT_POWER );
		SX126xSetTxInfinitePreamble();

		printf("TxContinuousWave Now--infinite preamble!\n\r");
		while(1);
	}
	else//CW连续发送,不用设置包参数
	{
		RadioSetTxContinuousWave( RF_FREQUENCY, TX_OUTPUT_POWER, TX_TIMEOUT );
		printf("TxContinuousWave Now---CW!\n\r");
		while(1);
	}

#else

	#ifdef TXFSK
		SX126xSetRfFrequency(RF_FREQUENCY);
		RadioSetTxConfig( MODEM_FSK, TX_OUTPUT_POWER, 0, FSK_BANDWIDTH,
								   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
								   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
								   true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );
		printf("begin to FSK\n!");

	#else	

		#ifdef TX_LORA
			SX126xSetRfFrequency(RF_FREQUENCY);
			RadioSetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
									   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
									   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
									   true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );
			 printf("begin to TX-lora packets mode\n!");
			 printf("%d,SF=%d,codeRate=%d,BW=%d,PWR=%d,PYLOAD=%d\n\r",RF_FREQUENCY,LORA_SPREADING_FACTOR,LORA_CODINGRATE,LORA_BANDWIDTH,TX_OUTPUT_POWER,BUFFER_SIZE);
			// { true,false }--public,{ false,false }--private
			 if (RadioPublicNetwork.Previous==true && RadioPublicNetwork.Current==false)
			 	printf("public\n\r");
			 else if (RadioPublicNetwork.Previous==false && RadioPublicNetwork.Current==false)
			 	printf("private\n\r");



		#else

		  SX126xSetRfFrequency(RF_FREQUENCY);
		  RadioSetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
										 LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
										 LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
										 0, true, 0, 0, LORA_IQ_INVERSION_ON, false );//最后一个参数设置是否是连续接收


		printf("%d,SF=%d,codeRate=%d,BW=%d!\n\r",RF_FREQUENCY,LORA_SPREADING_FACTOR,LORA_CODINGRATE,LORA_BANDWIDTH);
		 // { true,false }--public,{ false,false }--private
		 if (RadioPublicNetwork.Previous==true && RadioPublicNetwork.Current==false)
		 	printf("public\n\r");
		 else if (RadioPublicNetwork.Previous==false && RadioPublicNetwork.Current==false)
		 	printf("private\n\r");

		 //timeout=0代表单次接收，但是会先判断是否是连续接收，这个优先级最高
		  //RadioRx(3000);//连续接收 只要在这里设置一次就可以  中间不能sleep 否则要重新执行RadioRx();
		  printf("begin to RX--lora packets mode\n!");
		#endif
	#endif
#endif


	//SX126xSetRxTxFallbackMode(0x30);//退回到STANDBY_XOSC
	//printf("SX126xSetRxTxFallbackMode to STANDBY_XOSC\n!");
	
	//连续接收时使能这个
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
  

#ifdef TX_LORA
	while(1)
	{
		Buffer[0] = TxTime++;
	    Buffer[1] = 0;
	    Buffer[2] = 0;
	    Buffer[3] = 0;
	    Buffer[4] = 0;
	    Buffer[5] = 0;
		RadioSend(Buffer,BUFFER_SIZE);
		while(TXDone==false && TimeOutFlag==false);//一直等待tx done
		TXDone=false;
		TimeOutFlag=false;
		printf("TxTime=%d\n",TxTime);
		HAL_Delay(1); ///1s

		//读取状态
		RadioStatus=SX126xGetStatus();
		printf("RadioStatus is(after TX_DONE) %d\n",(((RadioStatus.Value)>>4)&0x07));
		
		
	}
#else
	/*
		///
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
	while(1)
	{
		
		//开始接收
		RadioRx(0);
		//while(1);//连续接收
		while(RXDoneFlag==false && TimeOutFlag==false && CRCFail==false);
		if(RXDoneFlag==true || TimeOutFlag==true || CRCFail==true)
		{
			if(CRCFail==false)	//CRC无错误
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
//DIO1的中断函数
void SX126xOnDio1Irq(void)
{
	 uint16_t irqRegs = SX126xGetIrqStatus( );
     SX126xClearIrqStatus( IRQ_RADIO_ALL );//这里清掉中断标志
     //发送结束
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

		//在SX126xSetTx()设置了一个超时时间 可以检测改功能 --ok
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
//产生触发脉冲
void GenTrig(void)
{
	HAL_GPIO_WritePin(TrigIO_GPIO_Port,TrigIO_Pin,GPIO_PIN_SET);
	
	HAL_GPIO_WritePin(TrigIO_GPIO_Port,TrigIO_Pin,GPIO_PIN_RESET);

	
}

void OnTxDone(void)
{
	SleepParams_t params = { 0 };

    params.Fields.WarmStart = 1;//热启动
    //printf("params.value=%d\n",params.Value);
    //SX126xSetSleep( params );//热启动可以保存进入睡眠前的状态的相关寄存器  sleep可以清掉所有中断标志位
	printf("OnTxDone\n");

}
void OnRxDone()
{

	int i;
	
	printf("onRXDone\n");
    SleepParams_t params = { 0 };

    params.Fields.WarmStart = 1;//热启动

//	printf("before sleep,operate mode=%d\n",SX126xGetOperatingMode());

  //  SX126xSetSleep( params );//

//	printf("after sleep,operate mode=%d\n",SX126xGetOperatingMode());
    //BufferSize = size;

	//TODO:处理包的数据
	
	

	
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
函数名称：fputc(int ch, FILE *f)
功    能：串口实现Printf()函数功能
参    数：无
返回值  ：
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
