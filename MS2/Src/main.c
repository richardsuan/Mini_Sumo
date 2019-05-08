/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
typedef enum{
	DISTANCE_CHECK,
	DISTANCE_STB
}ST_DISTANCE;
typedef enum{
	LED_STB=0,
	LED_CHECK,
	LED_BLINK
}ST_LED;
typedef enum{
	P_PUSH=0,
	P_TIME,
	P_CHECK
}ST_P;
enum{
	Tled1=0,
	Tled2,
	Tled3,
	Tled4,
	TledT1,
	TledT2,
	TledT3,
	TledT4,
	Tp1AR1,
	Tp1AR2,
	Tp1AR3,
	Tp1AR4,
	TLed1_push,
	timers
};

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
volatile int TIMERS[timers];
ST_LED led1,led2,led3,led4;
ST_P p1,p2,p3,p4;
ST_DISTANCE s_proximidad;
volatile uint16_t i;
volatile uint8_t estado=0;
char bufer[90];
uint16_t AR=200;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void distance_process();
void led1_process();
void p1_process();
void led2_process();
void p2_process();
void led3_process();
void p3_process();
void led4_process();
void p4_process();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_TIM2_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  s_proximidad=DISTANCE_STB;
  led1=LED_STB,led2=LED_STB,led3=LED_STB,led4=LED_STB,p1=P_CHECK,p2=P_CHECK,p3=P_CHECK,p4=P_CHECK;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  /*led1_process();
	  p1_process();
	  led2_process();
	  p2_process();
	  led3_process();
	  p3_process();
	  led4_process();
	  p4_process();*/
	  distance_process();

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB11 PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 PA10 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){  //timer cada 1ms
	if(htim->Instance==TIM2){
		for(i=0;i<timers;i++){
			if(TIMERS[i]!=0 && (i>=0 && i<12)){
				TIMERS[i]--;
			}else if(i>=12){
				TIMERS[i]++;
			}
		}
	}
}
void distance_process(){
	switch(s_proximidad){
			case DISTANCE_CHECK:
				if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_11)==1){
					//NO HAY NADA
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13, GPIO_PIN_SET);
					s_proximidad=DISTANCE_STB;
				}
				if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_11)==0){
					//HAY ALGO
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13, GPIO_PIN_RESET);
					s_proximidad=DISTANCE_STB;
								}
				break;
			case DISTANCE_STB:
				s_proximidad=DISTANCE_CHECK;
				break;

		}

}

void led1_process(){

}

void p1_process(){
	switch(p1){
		case P_CHECK:
			if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_8)==1){
				if(led1!=LED_STB){
					sprintf(bufer,"1. Quedan: %d ms \n",TIMERS[TledT1]);
					CDC_Transmit_FS((uint8_t*)bufer,(uint16_t)strlen(bufer));
				}else{
					p1=P_TIME;
					TIMERS[TLed1_push]=0;
				}
			}
			break;
		case P_TIME:
			if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_8)==1){

			}else{
				p1=P_PUSH;
				led1=LED_BLINK;
				TIMERS[TledT1]=TIMERS[TLed1_push];
				TIMERS[Tp1AR1]=AR;
			}
			break;
		case P_PUSH:
			if(TIMERS[Tp1AR1]==0){
				p1=P_CHECK;
			}
			break;
	}
}

void led2_process(){
	switch(led2){
		case LED_STB:
			//do nothing
			break;
		case LED_CHECK:
			led2=LED_BLINK;
			if(TIMERS[TledT2]==0){
				led2=LED_STB;
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,0);
			}
			break;
		case LED_BLINK:
			led2=LED_CHECK;
			if(TIMERS[Tled2]==0){
				TIMERS[Tled2]=80;
				HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_13);
			}
			break;
	}
}

void p2_process(){
	switch(p2){
		case P_CHECK:
			if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_9)==1){
				if(led2!=LED_STB){
					sprintf(bufer,"2. Quedan: %d ms \n",TIMERS[TledT2]);
					CDC_Transmit_FS((uint8_t*)bufer,(uint16_t)strlen(bufer));
				}else{
					led2=LED_BLINK;
					TIMERS[TledT2]=5000;
				}
				TIMERS[Tp1AR2]=AR;
				p2=P_PUSH;
			}

		case P_PUSH:
			if(TIMERS[Tp1AR2]==0){
				p2=P_CHECK;
			}
			break;
	}
}

void led3_process(){
	switch(led3){
		case LED_STB:
			//do nothing
			break;
		case LED_CHECK:
			led3=LED_BLINK;
			if(TIMERS[TledT3]==0){
				led3=LED_STB;
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,0);
			}
			break;
		case LED_BLINK:
			led3=LED_CHECK;
			if(TIMERS[Tled3]==0){
				TIMERS[Tled3]=500;
				HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_14);
			}
			break;
	}
}

void p3_process(){
	switch(p3){
		case P_CHECK:
			if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_10)==1){
				if(led3!=LED_STB){
					sprintf(bufer,"3. Quedan: %d ms \n",TIMERS[TledT3]);
					CDC_Transmit_FS((uint8_t*)bufer,(uint16_t)strlen(bufer));
				}else{
					led3=LED_BLINK;
					TIMERS[TledT3]=8000;
				}
				TIMERS[Tp1AR3]=AR;
				p3=P_PUSH;
			}

		case P_PUSH:
			if(TIMERS[Tp1AR3]==0){
				p3=P_CHECK;
			}
			break;
	}
}

void led4_process(){
	switch(led4){
		case LED_STB:
			//do nothing
			break;
		case LED_CHECK:
			led4=LED_BLINK;
			if(TIMERS[TledT4]==0){
				led4=LED_STB;
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,0);
			}
			break;
		case LED_BLINK:
			led4=LED_CHECK;
			if(TIMERS[Tled4]==0){
				TIMERS[Tled4]=2000;
				HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_15);
			}
			break;
	}
}

void p4_process(){
	switch(p4){
		case P_CHECK:
			if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_15)==1){
				if(led4!=LED_STB){
					sprintf(bufer,"4. Quedan: %d ms \n",TIMERS[TledT4]);
					CDC_Transmit_FS((uint8_t*)bufer,(uint16_t)strlen(bufer));
				}else{
					led4=LED_BLINK;
					TIMERS[TledT4]=10000;
				}
				TIMERS[Tp1AR4]=AR;
				p4=P_PUSH;
			}

		case P_PUSH:
			if(TIMERS[Tp1AR4]==0){
				p4=P_CHECK;
			}
			break;
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
