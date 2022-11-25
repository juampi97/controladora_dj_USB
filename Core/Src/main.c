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
#define SW1_PIN		GPIO_PIN_0

#define SW2_PORT	GPIOE
#define SW2_PIN		GPIO_PIN_1

#define SW3_PORT	GPIOE
#define SW3_PIN		GPIO_PIN_2

#define SW4_PORT	GPIOE
#define SW4_PIN		GPIO_PIN_3

#define SW5_PORT	GPIOE
#define SW5_PIN		GPIO_PIN_4

#define SW6_PORT	GPIOE
#define SW6_PIN		GPIO_PIN_5

#define SW7_PORT	GPIOB
#define SW7_PIN		GPIO_PIN_6

#define SW8_PORT	GPIOB
#define SW8_PIN		GPIO_PIN_7

#define SW9_PORT	GPIOB
#define SW9_PIN		GPIO_PIN_8

#define SW10_PORT	GPIOB
#define SW10_PIN	GPIO_PIN_9

#define SW11_PORT	GPIOB
#define SW11_PIN	GPIO_PIN_10

#define SW12_PORT	GPIOB
#define SW12_PIN	GPIO_PIN_11

#define SW13_PORT	GPIOB
#define SW13_PIN	GPIO_PIN_12

#define SW14_PORT	GPIOB
#define SW14_PIN	GPIO_PIN_13

#define SW15_PORT	GPIOB
#define SW15_PIN	GPIO_PIN_14

#define SW16_PORT	GPIOB
#define SW16_PIN	GPIO_PIN_15

#define SW17_PORT	GPIOE
#define SW17_PIN	GPIO_PIN_12

#define SW18_PORT	GPIOE
#define SW18_PIN	GPIO_PIN_13

#define SW19_PORT	GPIOE
#define SW19_PIN	GPIO_PIN_14

#define SW20_PORT	GPIOE
#define SW20_PIN	GPIO_PIN_15

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

uint8_t   	ADC_buffer[13];
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
uint8_t 	MIDI_pot6[3] = {0xB0,06,00};
uint16_t	pot6;
uint8_t 	MIDI_pot7[3] = {0xB0,07,00};
uint16_t	pot7;
uint8_t 	MIDI_pot8[3] = {0xB0,10,00};
uint16_t	pot8;
uint8_t 	MIDI_pot9[3] = {0xB0,11,00};
uint16_t	pot9;
uint8_t 	MIDI_pot10[3] = {0xB0,12,00};
uint16_t	pot10;
uint8_t 	MIDI_pot11[3] = {0xB0,13,00};
uint16_t	pot11;
uint8_t 	MIDI_pot12[3] = {0xB0,14,00};
uint16_t	pot12;
uint8_t 	MIDI_pot13[3] = {0xB0,15,00};
uint16_t	pot13;


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

uint8_t		MIDI_sw6[3] = {0x90, 0x39, 0x47};
uint32_t	counter_sw6;

uint8_t		MIDI_sw7[3] = {0x90, 0x40, 0x47};
uint32_t	counter_sw7;

uint8_t		MIDI_sw8[3] = {0x90, 0x41, 0x47};
uint32_t	counter_sw8;

uint8_t		MIDI_sw9[3] = {0x90, 0x42, 0x47};
uint32_t	counter_sw9;

uint8_t		MIDI_sw10[3] = {0x90, 0x43, 0x47};
uint32_t	counter_sw10;

uint8_t		MIDI_sw11[3] = {0x90, 0x44, 0x47};
uint32_t	counter_sw11;

uint8_t		MIDI_sw12[3] = {0x90, 0x45, 0x47};
uint32_t	counter_sw12;

uint8_t		MIDI_sw13[3] = {0x90, 0x46, 0x47};
uint32_t	counter_sw13;

uint8_t		MIDI_sw14[3] = {0x90, 0x47, 0x47};
uint32_t	counter_sw14;

uint8_t		MIDI_sw15[3] = {0x90, 0x48, 0x47};
uint32_t	counter_sw15;

uint8_t		MIDI_sw16[3] = {0x90, 0x49, 0x47};
uint32_t	counter_sw16;

uint8_t		MIDI_sw17[3] = {0x90, 0x50, 0x47};
uint32_t	counter_sw17;

uint8_t		MIDI_sw18[3] = {0x90, 0x51, 0x47};
uint32_t	counter_sw18;

uint8_t		MIDI_sw19[3] = {0x90, 0x52, 0x47};
uint32_t	counter_sw19;

uint8_t		MIDI_sw20[3] = {0x90, 0x53, 0x47};
uint32_t	counter_sw20;


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
	counter_sw6++;
	counter_sw7++;
	counter_sw8++;
	counter_sw9++;
	counter_sw10++;
	counter_sw11++;
	counter_sw12++;
	counter_sw13++;
	counter_sw14++;
	counter_sw15++;
	counter_sw16++;
	counter_sw17++;
	counter_sw18++;
	counter_sw19++;
	counter_sw20++;
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

		if( ( (ADC_buffer[5]) < (pot6-5) ) || ( (ADC_buffer[5]) > (pot6+5) ) ){
			pot6 = ADC_buffer[5];
			MIDI_pot6[2] = ADC_buffer[5] * 127 / 256;
			VCP_Transmit(MIDI_pot6,3);
		}


		if( ( (ADC_buffer[6]) < (pot7-5) ) || ( (ADC_buffer[6]) > (pot7+5) ) ){
			pot7 = ADC_buffer[6];
			MIDI_pot7[2] = ADC_buffer[6] * 127 / 256;
			VCP_Transmit(MIDI_pot7,3);
		}


		if( ( (ADC_buffer[7]) < (pot8-5) ) || ( (ADC_buffer[7]) > (pot8+5) ) ){
			pot8 = ADC_buffer[7];
			MIDI_pot8[2] = ADC_buffer[7] * 127 / 256;
			VCP_Transmit(MIDI_pot8,3);
		}


		if( ( (ADC_buffer[8]) < (pot9-5) ) || ( (ADC_buffer[8]) > (pot9+5) ) ){
			pot9 = ADC_buffer[8];
			MIDI_pot9[2] = ADC_buffer[8] * 127 / 256;
			VCP_Transmit(MIDI_pot9,3);
		}

		if( ( (ADC_buffer[9]) < (pot10-5) ) || ( (ADC_buffer[9]) > (pot10+5) ) ){
			pot10 = ADC_buffer[9];
			MIDI_pot10[2] = ADC_buffer[9] * 127 / 256;
			VCP_Transmit(MIDI_pot10,3);
		}

		if( ( (ADC_buffer[10]) < (pot11-5) ) || ( (ADC_buffer[10]) > (pot11+5) ) ){
			pot11 = ADC_buffer[10];
			MIDI_pot11[2] = ADC_buffer[10] * 127 / 256;
			VCP_Transmit(MIDI_pot11,3);
		}

		if( ( (ADC_buffer[11]) < (pot12-5) ) || ( (ADC_buffer[11]) > (pot12+5) ) ){
			pot12 = ADC_buffer[11];
			MIDI_pot12[2] = ADC_buffer[11] * 127 / 256;
			VCP_Transmit(MIDI_pot12,3);
		}

		if( ( (ADC_buffer[12]) < (pot13-5) ) || ( (ADC_buffer[12]) > (pot13+5) ) ){
			pot13 = ADC_buffer[12];
			MIDI_pot13[2] = ADC_buffer[12] * 127 / 256;
			VCP_Transmit(MIDI_pot13,3);
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

void sw6_task(void){
	static uint8_t state6 = 1;

	switch(state6)
	{
		case 1:
			if((HAL_GPIO_ReadPin(SW6_PORT, SW6_PIN) == 0))
			{
				state6 = 2;
				counter_sw6= 0;
			}
		break;
		case 2:
			if(counter_sw6 >= DEBOUNCING_TIME)
			{
				if(HAL_GPIO_ReadPin(SW6_PORT, SW6_PIN) == 0)
				{
					state6 = 3;
				}
				else
					state6 = 1;
			}
		break;
		case 3:
			if(HAL_GPIO_ReadPin(SW6_PORT, SW6_PIN) == 1)
			{
				VCP_Transmit(MIDI_sw6,3);
				state6 = 1;
			}
		break;
	}
}

void sw7_task(void){
	static uint8_t state7 = 1;

	switch(state7)
	{
		case 1:
			if((HAL_GPIO_ReadPin(SW7_PORT, SW7_PIN) == 0))
			{
				state7 = 2;
				counter_sw7= 0;
			}
		break;
		case 2:
			if(counter_sw7 >= DEBOUNCING_TIME)
			{
				if(HAL_GPIO_ReadPin(SW7_PORT, SW7_PIN) == 0)
				{
					state7 = 3;
				}
				else
					state7 = 1;
			}
		break;
		case 3:
			if(HAL_GPIO_ReadPin(SW7_PORT, SW7_PIN) == 1)
			{
				VCP_Transmit(MIDI_sw7,3);
				state7 = 1;
			}
		break;
	}
}

void sw8_task(void){
	static uint8_t state8 = 1;

	switch(state8)
	{
		case 1:
			if((HAL_GPIO_ReadPin(SW8_PORT, SW8_PIN) == 0))
			{
				state8 = 2;
				counter_sw8= 0;
			}
		break;
		case 2:
			if(counter_sw8 >= DEBOUNCING_TIME)
			{
				if(HAL_GPIO_ReadPin(SW8_PORT, SW8_PIN) == 0)
				{
					state8 = 3;
				}
				else
					state8 = 1;
			}
		break;
		case 3:
			if(HAL_GPIO_ReadPin(SW8_PORT, SW8_PIN) == 1)
			{
				VCP_Transmit(MIDI_sw8,3);
				state8 = 1;
			}
		break;
	}
}

void sw9_task(void){
	static uint8_t state9 = 1;

	switch(state9)
	{
		case 1:
			if((HAL_GPIO_ReadPin(SW9_PORT, SW9_PIN) == 0))
			{
				state9 = 2;
				counter_sw9= 0;
			}
		break;
		case 2:
			if(counter_sw9 >= DEBOUNCING_TIME)
			{
				if(HAL_GPIO_ReadPin(SW9_PORT, SW9_PIN) == 0)
				{
					state9 = 3;
				}
				else
					state9 = 1;
			}
		break;
		case 3:
			if(HAL_GPIO_ReadPin(SW9_PORT, SW9_PIN) == 1)
			{
				VCP_Transmit(MIDI_sw9,3);
				state9 = 1;
			}
		break;
	}
}

void sw10_task(void){
	static uint8_t state10 = 1;

	switch(state10)
	{
		case 1:
			if((HAL_GPIO_ReadPin(SW10_PORT, SW10_PIN) == 0))
			{
				state10 = 2;
				counter_sw10= 0;
			}
		break;
		case 2:
			if(counter_sw10 >= DEBOUNCING_TIME)
			{
				if(HAL_GPIO_ReadPin(SW10_PORT, SW10_PIN) == 0)
				{
					state10 = 3;
				}
				else
					state10 = 1;
			}
		break;
		case 3:
			if(HAL_GPIO_ReadPin(SW10_PORT, SW10_PIN) == 1)
			{
				VCP_Transmit(MIDI_sw10,3);
				state10 = 1;
			}
		break;
	}
}

void sw11_task(void){
	static uint8_t state11 = 1;

	switch(state11)
	{
		case 1:
			if((HAL_GPIO_ReadPin(SW11_PORT, SW11_PIN) == 0))
			{
				state11 = 2;
				counter_sw11= 0;
			}
		break;
		case 2:
			if(counter_sw11 >= DEBOUNCING_TIME)
			{
				if(HAL_GPIO_ReadPin(SW11_PORT, SW11_PIN) == 0)
				{
					state11 = 3;
				}
				else
					state11 = 1;
			}
		break;
		case 3:
			if(HAL_GPIO_ReadPin(SW11_PORT, SW11_PIN) == 1)
			{
				VCP_Transmit(MIDI_sw11,3);
				state11 = 1;
			}
		break;
	}
}

void sw12_task(void){
	static uint8_t state12 = 1;

	switch(state12)
	{
		case 1:
			if((HAL_GPIO_ReadPin(SW12_PORT, SW12_PIN) == 0))
			{
				state12 = 2;
				counter_sw12= 0;
			}
		break;
		case 2:
			if(counter_sw12 >= DEBOUNCING_TIME)
			{
				if(HAL_GPIO_ReadPin(SW12_PORT, SW12_PIN) == 0)
				{
					state12 = 3;
				}
				else
					state12 = 1;
			}
		break;
		case 3:
			if(HAL_GPIO_ReadPin(SW12_PORT, SW12_PIN) == 1)
			{
				VCP_Transmit(MIDI_sw12,3);
				state12 = 1;
			}
		break;
	}
}

void sw13_task(void){
	static uint8_t state13 = 1;

	switch(state13)
	{
		case 1:
			if((HAL_GPIO_ReadPin(SW13_PORT, SW13_PIN) == 0))
			{
				state13 = 2;
				counter_sw13= 0;
			}
		break;
		case 2:
			if(counter_sw13 >= DEBOUNCING_TIME)
			{
				if(HAL_GPIO_ReadPin(SW13_PORT, SW13_PIN) == 0)
				{
					state13 = 3;
				}
				else
					state13 = 1;
			}
		break;
		case 3:
			if(HAL_GPIO_ReadPin(SW13_PORT, SW13_PIN) == 1)
			{
				VCP_Transmit(MIDI_sw13,3);
				state13 = 1;
			}
		break;
	}
}

void sw14_task(void){
	static uint8_t state14 = 1;

	switch(state14)
	{
		case 1:
			if((HAL_GPIO_ReadPin(SW14_PORT, SW14_PIN) == 0))
			{
				state14 = 2;
				counter_sw14= 0;
			}
		break;
		case 2:
			if(counter_sw14 >= DEBOUNCING_TIME)
			{
				if(HAL_GPIO_ReadPin(SW14_PORT, SW14_PIN) == 0)
				{
					state14 = 3;
				}
				else
					state14 = 1;
			}
		break;
		case 3:
			if(HAL_GPIO_ReadPin(SW14_PORT, SW14_PIN) == 1)
			{
				VCP_Transmit(MIDI_sw14,3);
				state14 = 1;
			}
		break;
	}
}

void sw15_task(void){
	static uint8_t state15 = 1;

	switch(state15)
	{
		case 1:
			if((HAL_GPIO_ReadPin(SW15_PORT, SW15_PIN) == 0))
			{
				state15 = 2;
				counter_sw15= 0;
			}
		break;
		case 2:
			if(counter_sw15 >= DEBOUNCING_TIME)
			{
				if(HAL_GPIO_ReadPin(SW15_PORT, SW15_PIN) == 0)
				{
					state15 = 3;
				}
				else
					state15 = 1;
			}
		break;
		case 3:
			if(HAL_GPIO_ReadPin(SW15_PORT, SW15_PIN) == 1)
			{
				VCP_Transmit(MIDI_sw15,3);
				state15 = 1;
			}
		break;
	}
}

void sw16_task(void){
	static uint8_t state16 = 1;

	switch(state16)
	{
		case 1:
			if((HAL_GPIO_ReadPin(SW16_PORT, SW16_PIN) == 0))
			{
				state16 = 2;
				counter_sw16= 0;
			}
		break;
		case 2:
			if(counter_sw16 >= DEBOUNCING_TIME)
			{
				if(HAL_GPIO_ReadPin(SW16_PORT, SW16_PIN) == 0)
				{
					state16 = 3;
				}
				else
					state16 = 1;
			}
		break;
		case 3:
			if(HAL_GPIO_ReadPin(SW16_PORT, SW16_PIN) == 1)
			{
				VCP_Transmit(MIDI_sw16,3);
				state16 = 1;
			}
		break;
	}
}

void sw17_task(void){
	static uint8_t state17 = 1;

	switch(state17)
	{
		case 1:
			if((HAL_GPIO_ReadPin(SW17_PORT, SW17_PIN) == 0))
			{
				state17 = 2;
				counter_sw17= 0;
			}
		break;
		case 2:
			if(counter_sw17 >= DEBOUNCING_TIME)
			{
				if(HAL_GPIO_ReadPin(SW17_PORT, SW17_PIN) == 0)
				{
					state17 = 3;
				}
				else
					state17 = 1;
			}
		break;
		case 3:
			if(HAL_GPIO_ReadPin(SW17_PORT, SW17_PIN) == 1)
			{
				VCP_Transmit(MIDI_sw17,3);
				state17 = 1;
			}
		break;
	}
}

void sw18_task(void){
	static uint8_t state18 = 1;

	switch(state18)
	{
		case 1:
			if((HAL_GPIO_ReadPin(SW18_PORT, SW18_PIN) == 0))
			{
				state18 = 2;
				counter_sw18= 0;
			}
		break;
		case 2:
			if(counter_sw18 >= DEBOUNCING_TIME)
			{
				if(HAL_GPIO_ReadPin(SW18_PORT, SW18_PIN) == 0)
				{
					state18 = 3;
				}
				else
					state18 = 1;
			}
		break;
		case 3:
			if(HAL_GPIO_ReadPin(SW18_PORT, SW18_PIN) == 1)
			{
				VCP_Transmit(MIDI_sw18,3);
				state18 = 1;
			}
		break;
	}
}

void sw19_task(void){
	static uint8_t state19 = 1;

	switch(state19)
	{
		case 1:
			if((HAL_GPIO_ReadPin(SW19_PORT, SW19_PIN) == 0))
			{
				state19 = 2;
				counter_sw19= 0;
			}
		break;
		case 2:
			if(counter_sw19 >= DEBOUNCING_TIME)
			{
				if(HAL_GPIO_ReadPin(SW19_PORT, SW19_PIN) == 0)
				{
					state19 = 3;
				}
				else
					state19 = 1;
			}
		break;
		case 3:
			if(HAL_GPIO_ReadPin(SW19_PORT, SW19_PIN) == 1)
			{
				VCP_Transmit(MIDI_sw19,3);
				state19 = 1;
			}
		break;
	}
}

void sw20_task(void){
	static uint8_t state20 = 1;

	switch(state20)
	{
		case 1:
			if((HAL_GPIO_ReadPin(SW20_PORT, SW20_PIN) == 0))
			{
				state20 = 2;
				counter_sw20= 0;
			}
		break;
		case 2:
			if(counter_sw20 >= DEBOUNCING_TIME)
			{
				if(HAL_GPIO_ReadPin(SW20_PORT, SW20_PIN) == 0)
				{
					state20 = 3;
				}
				else
					state20 = 1;
			}
		break;
		case 3:
			if(HAL_GPIO_ReadPin(SW20_PORT, SW20_PIN) == 1)
			{
				VCP_Transmit(MIDI_sw20,3);
				state20 = 1;
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
	  sw6_task();
	  sw7_task();
	  sw8_task();
	  sw9_task();
	  sw10_task();
	  sw11_task();
	  sw12_task();
	  sw13_task();
	  sw14_task();
	  sw15_task();
	  sw16_task();
	  sw17_task();
	  sw18_task();
	  sw19_task();
	  sw20_task();
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
