/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include "ssd1306.h"
#include "i2c-lcd.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan1;

DAC_HandleTypeDef hdac;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t mode,MeanCounter=0;
uint16_t ADC1_BUF[128][4];
uint32_t MeanUbin,MeanIbin,MeanPot1B,MeanPot2B;
uint32_t BPot1,BPot2,BmeraneU,BmeraneI;
float meraneU,meraneI,Pot1,Pot2;

#define Ukonst 0.155844
#define	Ikonst 0.006012053
#define UkonstNastav 0.155844
#define	IkonstNastav 0.006012053
/* 	mode 0 - start
	mode 1 - manual mode
	mode 2 - automatic linear voltage
	mode 3 - automatic PWM current
	mode 4 - fast charge
*/
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN1_Init(void);
static void MX_DAC_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/*
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc->Instance==ADC1)
		{
			BmeraneU=ADC1_BUF[0];
			BmeraneI=ADC1_BUF[1];
			BPot1=ADC1_BUF[2];
			BPot2=ADC1_BUF[3];
		}
	meraneU=((float)BmeraneU*Ukonst);
		meraneI=((float)BmeraneI*Ikonst);
		Pot1=((float)BPot1*Ukonst);
		Pot2=((float)BPot2*Ikonst);

		MeanU[MeanCounter]=ADC1_BUF[0];
		MeanI[MeanCounter]=ADC1_BUF[1];
		MeanPot1[MeanCounter]=ADC1_BUF[2];
		MeanPot2[MeanCounter]=ADC1_BUF[3];

		MeanCounter++;
		MeanUbin=0;
		MeanIbin=0;
		MeanPot1B=0;
		MeanPot2B=0;

		if (MeanCounter>=25)	MeanCounter=0;

		for(int counterF=0;counterF<=25;counterF++)
		  {
		  		MeanUbin+=(uint32_t)MeanU[counterF];
		  		MeanIbin+=(uint32_t)MeanI[counterF];
		  		MeanPot1B+=(uint32_t)MeanPot1[counterF];
				MeanPot2B+=(uint32_t)MeanPot2[counterF];
		  }
		MeanUbin=(uint32_t)(MeanUbin/25);
		MeanIbin=(uint32_t)(MeanIbin/25);
		MeanPot1B=(uint32_t)(MeanPot2B/25);
		MeanPot2B=(uint32_t)(MeanPot2B/25);
		if (MeanUbin>=4095)	MeanUbin=0;
		if (MeanIbin>=4095)	MeanIbin=0;
		if (MeanPot1B>=4095)	MeanPot1B=0;
		if (MeanPot2B>=4095)	MeanPot2B=0;
}*/
void nastavU(float napetie)
{
	uint32_t Ubinar;
	Ubinar = (uint32_t)(napetie/UkonstNastav);
	HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R, Ubinar);
}
void nastavI(float prud)
{
	uint32_t Ibinar;
	Ibinar = (uint32_t)(prud/IkonstNastav);
	HAL_DAC_SetValue(&hdac,DAC_CHANNEL_2,DAC_ALIGN_12B_R, Ibinar);
}
void nastavUbin(uint32_t Ubinar)
{
	HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R, Ubinar);
}
void nastavIbin(uint32_t Ibinar)
{
	HAL_DAC_SetValue(&hdac,DAC_CHANNEL_2,DAC_ALIGN_12B_R, Ibinar);
}
void vypisOLED()
{
	char text [20];
	//SSD1306_Init ();
	SSD1306_Fill (0);
	SSD1306_GotoXY (2, 0);
	//sprintf(text,"Uz:%.2d V",Pot1);
	sprintf(text,"Uz:%d V",BPot1);
	SSD1306_Puts (text, &Font_7x10, 1);
	//SSD1306_UpdateScreen();
	SSD1306_GotoXY (2, 12);
	//sprintf(text,"Um:%.2d V",(float)meraneU);
	sprintf(text,"Um:%d V",BmeraneU);
	SSD1306_Puts (text, &Font_7x10, 1);
	//SSD1306_UpdateScreen();
	SSD1306_GotoXY (2, 24);
	//sprintf(text,"Iz:%.2d A",(float)Pot2);
	sprintf(text,"Iz:%d A",BPot2);
	SSD1306_Puts (text, &Font_7x10, 1);
	//SSD1306_UpdateScreen();
	SSD1306_GotoXY (2, 36);
	//sprintf(text,"Im:%.2d A",(float)meraneI);
	sprintf(text,"Im:%d A",BmeraneI);
	SSD1306_Puts (text, &Font_7x10, 1);
	SSD1306_GotoXY (2, 48);
	sprintf(text,"IMD:%s BMS:%s","ok","ok");
	SSD1306_Puts (text, &Font_7x10, 1);
	SSD1306_UpdateScreen();

}
void DACtest()
{
	uint32_t	valByte2,valByte;
	valByte2 = (uint32_t)((1.0/1.0)*4095);
	float valVolt = 1.9 ;
	valByte = (uint32_t)((valVolt/3.3)*4095);
	for(int d=0;d<3300;d++)
	{
		float num=(float)d;
		valByte = (uint32_t)((num/3300.0)*4095);
		valByte2 = (uint32_t)((1-(num/3300.0))*4095);
		HAL_DAC_SetValue(&hdac,DAC_CHANNEL_2,DAC_ALIGN_12B_R, valByte);
		HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R, valByte2);
		//HAL_Delay(1);
	}
}
void DMAMeranie()
{
	MeanUbin=0;
	MeanIbin=0;
	MeanPot1B=0;
	MeanPot2B=0;
	MeanPot2B=0;
	for(int counterF=0;counterF<126;counterF++)
		{
			  		MeanUbin+=ADC1_BUF[counterF][0];
			  		MeanIbin+=ADC1_BUF[counterF][1];
			  		MeanPot1B+=ADC1_BUF[counterF][2];
					MeanPot2B+=ADC1_BUF[counterF][3];
		}
	BmeraneU=MeanUbin/126;
	BmeraneI=MeanIbin/126;
	BPot1=MeanPot1B/126;
	BPot2=MeanPot2B/126;
	//HAL_ADC_Start_IT(&hadc1);
	//HAL_ADC_Start_DMA(&hadc1,(uint32_t*)ADC1_BUF,4);

}
void InitVypis()
{
	SSD1306_Init ();
	SSD1306_Fill (0);
	SSD1306_GotoXY (40, 0);
	SSD1306_Puts ("NES", &Font_16x26, 1);
	SSD1306_UpdateScreen();
	SSD1306_GotoXY (30, 26);
	SSD1306_Puts ("CHARGER", &Font_11x18, 1);
	SSD1306_UpdateScreen();
	SSD1306_GotoXY (10, 50);
	SSD1306_Puts ("STUBA GREEN TEAM", &Font_7x10, 1);
	SSD1306_UpdateScreen();
}
void BlockVypis()
{
	char text [20];
		//SSD1306_Init ();
		SSD1306_Fill (0);

		SSD1306_GotoXY (2, 0);
		SSD1306_Puts ("Vynuluj", &Font_11x18, 1);
		SSD1306_GotoXY (60, 15);
				SSD1306_Puts ("zdroj", &Font_11x18, 1);

		SSD1306_GotoXY (2, 30);
		sprintf(text,"Uz:%d V",BPot1);
		SSD1306_Puts (text, &Font_11x18, 1);

		SSD1306_GotoXY (2, 45);
		sprintf(text,"Iz:%d A",BPot2);
		SSD1306_Puts (text, &Font_11x18, 1);
		//SSD1306_UpdateScreen();


		SSD1306_UpdateScreen();
}
void PrimitivneOvladanie()
{
//PriameMeranie();
DMAMeranie();
nastavUbin(BPot1);
nastavIbin(BPot2);
//DACtest();
vypisOLED();
HAL_Delay(10);
//testvypis();
}

void Mode0()
{
	while(1)
	{
		nastavIbin(0);
		nastavUbin(0);
		InitVypis();
		//set blokaciu
		//break na interupt bud tla4idlo alebo y 4D displeya
	}
}
void Mode1()
{
	//while(1)
		//{
			//reset blokacie
			PrimitivneOvladanie();

		//}
}
void Mode2()
{
	while(1)
		{

		}
}
void Mode3()
{
	while(1)
		{

		}
}
void Mode4()
{
	while(1)
		{

		}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_CAN1_Init();
  MX_DAC_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  mode=1;
  HAL_ADC_Start_DMA(&hadc1,(uint32_t*)ADC1_BUF,512);
  //HAL_ADC_Start_IT(&hadc1);
  //HAL_ADC_Start(&hadc2);
  uint32_t nula=0;
  HAL_DAC_Start(&hdac,DAC_CHANNEL_1);
  HAL_DAC_Start(&hdac,DAC_CHANNEL_2);
  HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R, nula);
  HAL_DAC_SetValue(&hdac,DAC_CHANNEL_2,DAC_ALIGN_12B_R, nula);
  SSD1306_Init ();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  InitVypis();
  HAL_Delay(1000);

  while (1)
  {
	  HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R, nula);
	  HAL_DAC_SetValue(&hdac,DAC_CHANNEL_2,DAC_ALIGN_12B_R, nula);
	  //PriameMeranie();
	  DMAMeranie();
	  BlockVypis();
	  if(BPot1==0) break;
  }
  while (1)
  {

	  switch(mode)
	  	  {
	  	  	  case 0://vyber a zaistenie
	  	  		  Mode0();
	  	  		  break;

	  	  	  case 1://primitivne ovladanie
	  	  		  Mode1();
	  	  		  break;

	  	  	  case 2://automatica napätova
	  	  		  Mode2();
	  	  		  break;

	  	  	  case 3://automatika prudova
	  	  		  Mode3();
	  	  		  break;

	  	  	  case 4://fast charger
	  	  		  Mode4();
	  	  		  break;
	  	  }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
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
  RCC_OscInitStruct.Prediv1Source = RCC_PREDIV1_SOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL7;
  RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL_NONE;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /**Configure the Systick interrupt time 
    */
  __HAL_RCC_PLLI2S_ENABLE();

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* CAN1 init function */
static void MX_CAN1_Init(void)
{

  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SJW = CAN_SJW_1TQ;
  hcan1.Init.BS1 = CAN_BS1_1TQ;
  hcan1.Init.BS2 = CAN_BS2_1TQ;
  hcan1.Init.TTCM = DISABLE;
  hcan1.Init.ABOM = DISABLE;
  hcan1.Init.AWUM = DISABLE;
  hcan1.Init.NART = DISABLE;
  hcan1.Init.RFLM = DISABLE;
  hcan1.Init.TXFP = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* DAC init function */
static void MX_DAC_Init(void)
{

  DAC_ChannelConfTypeDef sConfig;

    /**DAC Initialization 
    */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**DAC channel OUT1 config 
    */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**DAC channel OUT2 config 
    */
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(blokacia_GPIO_Port, blokacia_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, led1_Pin|led2_Pin|led3_Pin|led4_Pin 
                          |led5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : blokacia_Pin */
  GPIO_InitStruct.Pin = blokacia_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(blokacia_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : prehriatie_Pin */
  GPIO_InitStruct.Pin = prehriatie_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(prehriatie_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : led1_Pin led2_Pin led3_Pin led4_Pin 
                           led5_Pin */
  GPIO_InitStruct.Pin = led1_Pin|led2_Pin|led3_Pin|led4_Pin 
                          |led5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PIN1_Pin PIN2_Pin PIN3_Pin PIN4_Pin */
  GPIO_InitStruct.Pin = PIN1_Pin|PIN2_Pin|PIN3_Pin|PIN4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
