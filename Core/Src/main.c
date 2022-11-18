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
#define wait_bouncing	200

#define encoder_port	GPIOE
#define encoder_key 	GPIO_PIN_1
#define ON			1
#define OFF			0
#define IDLE		0
#define CHECK		1
#define PUSH		2
#define NPUSH		3
#define DEBOUNCING_TIME		20

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
uint8_t		flagSend_MIDI_pot[5] = {0,0,0,0,0};

uint8_t		tx_buffer[15];
uint8_t		tx_state = READY;
uint16_t	tx_counter;
int			tx_data;

uint32_t	counter_encoder = 0;
uint32_t	tim2_count = 0;
uint32_t	direc;
uint32_t	encoder_bouncing;
int			data_encoder;
uint8_t		MIDI_scroll[3] = {0XB0, 0X1A, 0X7F};

uint8_t		MIDI_swEncoder[3] = {0x90, 0x34, 0x47};
uint8_t		flag_swEncoder_push;
uint8_t		flagSend_MIDI_swEncoder;
uint32_t	counter_swEncoder;
uint8_t 	value_swEncoder;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim){
	if(data_encoder == 1){
		encoder_bouncing++;
	}
	counter_swEncoder++;
	ADC_counter++;
	tx_counter++;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){

	tim2_count = __HAL_TIM_GET_COUNTER(htim);
	direc = !(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2));

	if( (tim2_count != counter_encoder) && (direc == 1) ){
		MIDI_scroll[2] = 127;
		tx_data = 1;
		data_encoder = 1;
		encoder_bouncing = 0;
		counter_encoder = tim2_count;
	}

	if( (tim2_count != counter_encoder) && (direc == 0) ){
		MIDI_scroll[2] = 1;
		tx_data = 1;
		data_encoder = 1;
		encoder_bouncing = 0;
		counter_encoder = tim2_count;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(HAL_GPIO_ReadPin(encoder_port, encoder_key) == 1){
		flag_swEncoder_push = ON;
		value_swEncoder = HAL_GPIO_ReadPin(encoder_port, encoder_key);
	}
}

void swEncoder_Task(void){

	static uint8_t state = IDLE;

	switch(state)
	{
		case IDLE:
			if(flag_swEncoder_push == ON)
			{
				state = CHECK;
				counter_swEncoder = 0;
				flag_swEncoder_push = OFF;
			}
		break;
		case CHECK:
			if(counter_swEncoder == DEBOUNCING_TIME)
			{
				if(value_swEncoder == HAL_GPIO_ReadPin(encoder_port, encoder_key))
				{
					if(value_swEncoder == GPIO_PIN_SET)
						state = PUSH;
					else
						state = NPUSH;
				}
				else
					state = IDLE;
			}
		break;
		case PUSH:
			//-- Aca poner instrucciones de que hacer al presionar sw0 --//
				flagSend_MIDI_swEncoder = 1;
			//---------------------------- Fin --------------------------//
			state=IDLE;
		break;
		case NPUSH:
			//-- Aca poner instrucciones de que hacer al soltar sw0 --//
			//---------------------------- Fin --------------------------//
			state=IDLE;
		break;
	}
}

void ADC_task(void){

	if(ADC_counter > 500){

		ADC_counter = 0;

		if( ( (ADC_buffer[0]) < (pot1-5) ) || ( (ADC_buffer[0]) > (pot1+5) ) ){
			pot1 = ADC_buffer[0];
			MIDI_pot1[2] = ADC_buffer[0] * 127 / 256;
			flagSend_MIDI_pot[1] = 1;



		}


		if( ( (ADC_buffer[1]) < (pot2-5) ) || ( (ADC_buffer[1]) > (pot2+5) ) ){
			pot2 = ADC_buffer[1];
			MIDI_pot2[2] = ADC_buffer[1] * 127 / 256;
			flagSend_MIDI_pot[2] = 1;
		}


		if( ( (ADC_buffer[2]) < (pot3-5) ) || ( (ADC_buffer[2]) > (pot3+5) ) ){
			pot3 = ADC_buffer[2];
			MIDI_pot3[2] = ADC_buffer[2] * 127 / 256;
			flagSend_MIDI_pot[3] = 1;
		}


		if( ( (ADC_buffer[3]) < (pot4-5) ) || ( (ADC_buffer[3]) > (pot4+5) ) ){
			pot4 = ADC_buffer[3];
			MIDI_pot4[2] = ADC_buffer[3] * 127 / 256;
			flagSend_MIDI_pot[4] = 1;
		}
	}


}

//	Funciones Transmit- Reception USB

void VCP_TransmitCpltCallback(void){

	tx_state = READY;
	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);

}

void tx_task(void){

/*	if((tx_state == READY) && (encoder_bouncing == wait_bouncing)){
		tx_state = BUSY;
		tx_data = 0;
		encoder_bouncing = 0;
		data_encoder = 0;

		HAL_UART_Transmit_IT(&huart6,(uint8_t*) MIDI_scroll,3);
	}
*/
	if( (tx_state == READY) && (flagSend_MIDI_pot[1] == 1) ){
		flagSend_MIDI_pot[1] = 0;
		tx_state = BUSY;
		VCP_Transmit(MIDI_pot1,3);
	}

	if( (tx_state == READY) && (flagSend_MIDI_pot[2] == 1) ){
		flagSend_MIDI_pot[2] = 0;
		tx_state = BUSY;
		VCP_Transmit(MIDI_pot2,3);
	}

	if( (tx_state == READY) && (flagSend_MIDI_pot[3] == 1) ){
		flagSend_MIDI_pot[3] = 0;
		tx_state = BUSY;
		VCP_Transmit(MIDI_pot3,3);
	}

	if( (tx_state == READY) && (flagSend_MIDI_pot[4] == 1) ){
		flagSend_MIDI_pot[4] = 0;
		tx_state = BUSY;
		VCP_Transmit(MIDI_pot4,3);
	}

/*	if( (tx_state == READY) && (flagSend_MIDI_swEncoder == 1) ){
		flagSend_MIDI_swEncoder = 0;
		tx_state = BUSY;
		HAL_UART_Transmit_IT(&huart6,(uint8_t*) MIDI_swEncoder,3);
	}
*/
}

void VCP_ReceiveCpltCallback(uint8_t *buffer, uint32_t size){

	//HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
	//VCP_Transmit(buffer,size);

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
  MX_DMA_Init();
  MX_USB_DEVICE_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_Encoder_Start_IT(&htim2, TIM_CHANNEL_ALL);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *) &ADC_buffer, 4);

  tx_buffer[0] = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

	  //swEncoder_Task();
	  ADC_task();
	  tx_task();

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
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Rank = 4;
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
  htim4.Init.Period = 15999;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
