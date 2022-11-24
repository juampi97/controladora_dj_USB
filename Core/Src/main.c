/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define READY			0
#define BUSY			1

#define wait_bouncing	800

#define DEBOUNCING_TIME		100

#define SW1_PORT	GPIOE
#define SW1_PIN		GPIO_PIN_2

#define SW2_PORT	GPIOE
#define SW2_PIN		GPIO_PIN_3

#define SW3_PORT	GPIOE
#define SW3_PIN		GPIO_PIN_4

#define SW4_PORT	GPIOE
#define SW4_PIN		GPIO_PIN_5

#define SW5_PORT	GPIOE
#define SW5_PIN		GPIO_PIN_6

#define SW_BOUNCING_TX_USB 5000

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */

uint32_t led_counter;

uint8_t   	ADC_buffer[14];
uint32_t 	ADC_counter;
uint8_t 	MIDI_pot1[3] = {0xB0,01,00};
uint16_t	pot1;
uint8_t 	MIDI_pot2[3] = {0xB0,02,00};
uint16_t	pot2;
uint8_t 	MIDI_pot3[3] = {0xB0,03,00};
uint16_t	pot3;
uint8_t 	MIDI_pot4[3] = {0xB0,04,00};
uint16_t	pot4;
uint8_t 	MIDI_pot5[3] = {0xB0,05,00};
uint16_t	pot5;

uint32_t	counter_encoder = 0;
uint32_t	tim2_count = 0;
uint32_t	direc;
uint32_t	encoder_bouncing;
int			data_encoder;
uint32_t	tx_encoder;

uint8_t		MIDI_scroll[3] = {0XB0, 0X1A, 0X7F};

uint8_t		MIDI_sw1[3] = {0x90, 0x34, 0x47};
uint32_t	counter_sw1;

uint8_t		MIDI_sw2[3] = {0x90, 0x35, 0x47};
uint32_t	counter_sw2;

uint8_t		MIDI_sw3[3] = {0x90, 0x36, 0x47};
uint32_t	counter_sw3;

uint8_t		MIDI_sw4[3] = {0x90, 0x37, 0x47};
uint32_t	counter_sw4;

uint8_t		MIDI_sw5[3] = {0x90, 0x38, 0x47};
uint32_t	counter_sw5;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim){
	ADC_counter++;

	led_counter++;

	tx_encoder++;
	encoder_bouncing++;

	counter_sw1++;
	counter_sw2++;
	counter_sw3++;
	counter_sw4++;
	counter_sw5++;
}

void led_task(void){
	if(led_counter > 1000){
		led_counter = 0;
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	tim2_count = __HAL_TIM_GET_COUNTER(htim);
	direc = !(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2));

	if( (tim2_count != counter_encoder) && (direc == 1) ){
		MIDI_scroll[1] = 0X1A;
		MIDI_scroll[2] = 127;
		data_encoder = 1;
		encoder_bouncing = 0;
		counter_encoder = tim2_count;
	}

	if( (tim2_count != counter_encoder) && (direc == 0) ){
		MIDI_scroll[1] = 0X1B;
		MIDI_scroll[2] = 1;
		data_encoder = 1;
		encoder_bouncing = 0;
		counter_encoder = tim2_count;
	}
}

void encoder_task(void){
	if( (encoder_bouncing == wait_bouncing) & (tx_encoder > 2000) ){
			data_encoder = 0;
			VCP_Transmit(MIDI_scroll,3);
			tx_encoder = 0;
	}
}

void ADC_task(void){

	if(ADC_counter > 500){

		ADC_counter = 0;

		if( ( (ADC_buffer[0]) < (pot1-5) ) || ( (ADC_buffer[0]) > (pot1+5) ) ){
			pot1 = ADC_buffer[0];
			MIDI_pot1[2] = ADC_buffer[0] * 127 / 256;
			VCP_Transmit(MIDI_pot1,3);
		}


		if( ( (ADC_buffer[1]) < (pot2-5) ) || ( (ADC_buffer[1]) > (pot2+5) ) ){
			pot2 = ADC_buffer[1];
			MIDI_pot2[2] = ADC_buffer[1] * 127 / 256;
			VCP_Transmit(MIDI_pot2,3);
		}


		if( ( (ADC_buffer[2]) < (pot3-5) ) || ( (ADC_buffer[2]) > (pot3+5) ) ){
			pot3 = ADC_buffer[2];
			MIDI_pot3[2] = ADC_buffer[2] * 127 / 256;
			VCP_Transmit(MIDI_pot3,3);
		}


		if( ( (ADC_buffer[3]) < (pot4-5) ) || ( (ADC_buffer[3]) > (pot4+5) ) ){
			pot4 = ADC_buffer[3];
			MIDI_pot4[2] = ADC_buffer[3] * 127 / 256;
			VCP_Transmit(MIDI_pot4,3);
		}
		if( ( (ADC_buffer[4]) < (pot5-5) ) || ( (ADC_buffer[4]) > (pot5+5) ) ){
			pot5 = ADC_buffer[4];
			MIDI_pot5[2] = ADC_buffer[4] * 127 / 256;
			VCP_Transmit(MIDI_pot5,3);
		}
	}
}

void sw1_task(void){
	static uint8_t state1 = 1;

	switch(state1)
	{
		case 1:
			if((HAL_GPIO_ReadPin(SW1_PORT, SW1_PIN) == 0))
			{
				state1 = 2;
				counter_sw1= 0;
			}
		break;
		case 2:
			if(counter_sw1 >= DEBOUNCING_TIME)
			{
				if(HAL_GPIO_ReadPin(SW1_PORT, SW1_PIN) == 0)
				{
					state1 = 3;
				}
				else
					state1 = 1;
			}
		break;
		case 3:
			if(HAL_GPIO_ReadPin(SW1_PORT, SW1_PIN) == 1)
			{
				VCP_Transmit(MIDI_sw1,3);
				state1 = 1;
			}
		break;
	}
}

void sw2_task(void){
	static uint8_t state2 = 1;

	switch(state2)
	{
		case 1:
			if((HAL_GPIO_ReadPin(SW2_PORT, SW2_PIN) == 0))
			{
				state2 = 2;
				counter_sw2= 0;
			}
		break;
		case 2:
			if(counter_sw2 >= DEBOUNCING_TIME)
			{
				if(HAL_GPIO_ReadPin(SW2_PORT, SW2_PIN) == 0)
				{
					state2 = 3;
				}
				else
					state2 = 1;
			}
		break;
		case 3:
			if(HAL_GPIO_ReadPin(SW2_PORT, SW2_PIN) == 1)
			{
				VCP_Transmit(MIDI_sw2,3);
				state2 = 1;
			}
		break;
	}
}

void sw3_task(void){
	static uint8_t state3 = 1;

	switch(state3)
	{
		case 1:
			if((HAL_GPIO_ReadPin(SW3_PORT, SW3_PIN) == 0))
			{
				state3 = 2;
				counter_sw3= 0;
			}
		break;
		case 2:
			if(counter_sw3 >= DEBOUNCING_TIME)
			{
				if(HAL_GPIO_ReadPin(SW3_PORT, SW3_PIN) == 0)
				{
					state3 = 3;
				}
				else
					state3 = 1;
			}
		break;
		case 3:
			if(HAL_GPIO_ReadPin(SW3_PORT, SW3_PIN) == 1)
			{
				VCP_Transmit(MIDI_sw3,3);
				state3 = 1;
			}
		break;
	}
}

void sw4_task(void){
	static uint8_t state4 = 1;

	switch(state4)
	{
		case 1:
			if((HAL_GPIO_ReadPin(SW4_PORT, SW4_PIN) == 0))
			{
				state4 = 2;
				counter_sw4= 0;
			}
		break;
		case 2:
			if(counter_sw4 >= DEBOUNCING_TIME)
			{
				if(HAL_GPIO_ReadPin(SW4_PORT, SW4_PIN) == 0)
				{
					state4 = 3;
				}
				else
					state4 = 1;
			}
		break;
		case 3:
			if(HAL_GPIO_ReadPin(SW4_PORT, SW4_PIN) == 1)
			{
				VCP_Transmit(MIDI_sw4,3);
				state4 = 1;
			}
		break;
	}
}

void sw5_task(void){
	static uint8_t state5 = 1;

	switch(state5)
	{
		case 1:
			if((HAL_GPIO_ReadPin(SW5_PORT, SW5_PIN) == 0))
			{
				state5 = 2;
				counter_sw5= 0;
			}
		break;
		case 2:
			if(counter_sw5 >= DEBOUNCING_TIME)
			{
				if(HAL_GPIO_ReadPin(SW5_PORT, SW5_PIN) == 0)
				{
					state5 = 3;
				}
				else
					state5 = 1;
			}
		break;
		case 3:
			if(HAL_GPIO_ReadPin(SW5_PORT, SW5_PIN) == 1)
			{
				VCP_Transmit(MIDI_sw5,3);
				state5 = 1;
			}
		break;
	}
}

//	Funciones Transmit- Reception USB

void VCP_TransmitCpltCallback(void){
	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
}

void VCP_ReceiveCpltCallback(uint8_t *buffer, uint32_t size){

}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_USB_DEVICE_Init();
  MX_DMA_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Encoder_Start_IT(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *) &ADC_buffer, 15);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  led_task();

	  encoder_task();

	  ADC_task();

	  sw1_task();
	  sw2_task();
	  sw3_task();
	  sw4_task();
	  sw5_task();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 13;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 9;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 10;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 11;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 12;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 13;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 15599;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_OC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE3 PE4 PE5
                           PE0 PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PE12 PE13 PE14 PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB11 PB12 PB13
                           PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PB6 PB7 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
