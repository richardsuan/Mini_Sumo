/* USER CODE BEGIN Header */
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum{
	IR_BLANCO=0,
	IR_NEGRO
}ST_IR_PISO;
typedef enum{
	IR_ON=0,
	IR_OFF

}ST_IR_PROXIMIDAD;
typedef enum{
	M_RUN_MAX_DER=0,
	M_RUN_MAX_IZQ,
	M_STOP,
	M_RUN_MIN_DER,
	M_RUN_MIN_IZQ,
	M_RUN_MID_IZQ,
	M_RUN_MID_DER,
	M_STB
}ST_MOTOR;

typedef enum{
	LOC_CHECK=0,
	LOC_STB
}ST_LOC;
typedef enum{
	LOC_CHECKING=0,
	LOC_MOVE,
	LOC_ST
}ST_SEARCH;
typedef enum{
	B_COUNT=0,
	B_STB
}ST_BOTON;
enum{
	T_MOTOR=0,
	T_BOTON,
	T_TURN,
	timers
};
uint8_t aux=0;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
ST_IR_PISO IR1,IR2,IR3,IR4;
ST_MOTOR M1,M2;
ST_LOC LOC;
ST_IR_PROXIMIDAD SP;
ST_SEARCH buscar;
ST_BOTON B1;
volatile uint16_t i;
volatile int TIMERS[timers];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
void IR1_process();
void SP_process();//
void IR2_process();
void IR3_process();
void IR4_process();
void M1_process();
void M2_process();
void LOC_process();
void B1_process();
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
  MX_TIM1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
  IR1=IR_NEGRO,IR2=IR_NEGRO,IR3=IR_NEGRO,IR4=IR_NEGRO;
  M1=M_STOP,M2=M_STOP;
  SP=IR_ON;
  LOC=LOC_CHECK;
  B1=B_STB;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(aux==0){
		  B1_process();
		  M2=M_STOP;
		  M1=M_STOP;
		  M1_process();
		  M2_process();
	  }else if(aux==1){
		  IR1_process();
		  IR2_process();
		  IR3_process();
		  IR4_process();

		  M1_process();
		  M2_process();

		  LOC_process();
		  SP_process();
	  }
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 5;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 9999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  htim2.Init.Period = 99;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB11 PB12 PB13 
                           PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13 
                          |GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){  //timer cada 100us
	if(htim->Instance==TIM2){
		for(i=0;i<timers;i++){
			if(TIMERS[i]!=0){
				TIMERS[i]--;
			}
		}
	}
}

void IR1_process(){
	switch(IR1){
		case IR_BLANCO:
			if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_12)==1){
				IR1=IR_NEGRO;
			}
			break;
		case IR_NEGRO:
			if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_12)==0){
				IR1=IR_BLANCO;
			}
			break;
	}
}

void SP_process(){
	switch(SP){
	case(IR_OFF):
		if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_11)==1){
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13, GPIO_PIN_SET);
			SP=IR_ON;
		}
			break;
	case(IR_ON):
		if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_11)==0){
			//hay alguien
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13, GPIO_PIN_RESET);
			SP=IR_OFF;
		}
			break;
	}
}

void IR2_process(){
	switch(IR2){
		case IR_BLANCO:
			if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_13)==1){
				IR2=IR_NEGRO;
			}
			break;
		case IR_NEGRO:
			if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_13)==0){
				IR2=IR_BLANCO;
			}
			break;
	}
}

void IR3_process(){
	switch(IR3){
		case IR_BLANCO:
			if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14)==1){
				IR3=IR_NEGRO;
			}
			break;
		case IR_NEGRO:
			if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14)==0){
				IR3=IR_BLANCO;
			}
			break;
	}
}

void IR4_process(){
	switch(IR4){
		case IR_BLANCO:
			if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_15)==1){
				IR4=IR_NEGRO;
			}
			break;
		case IR_NEGRO:
			if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_15)==0){
				IR4=IR_BLANCO;
			}
			break;
	}
}

void M1_process(){
	switch(M1){
		case M_STB:
			//Do nothing
			break;
		case M_STOP:
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,5000);
			M1=M_STB;
			break;
		case M_RUN_MIN_DER:
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,4500);
			M1=M_STB;
			break;
		case M_RUN_MID_IZQ:
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,7000);
			M1=M_STB;
			break;
		case M_RUN_MID_DER:
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,3000);
			M1=M_STB;
			break;
		case M_RUN_MIN_IZQ:
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,5500);
			M1=M_STB;
			break;
		case M_RUN_MAX_DER:
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,1);
			M1=M_STB;
			break;
		case M_RUN_MAX_IZQ:
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,9999);
			M1=M_STB;
			break;
	}
}

void M2_process(){
	switch(M2){
		case M_STB:
			//Do nothing
			break;
		case M_STOP:
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,5000);
			M2=M_STB;
			break;
		case M_RUN_MIN_DER:
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,4000);
			M2=M_STB;
			break;
		case M_RUN_MIN_IZQ:
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,6000);
			M2=M_STB;
			break;
		case M_RUN_MAX_DER:
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,1);
			M2=M_STB;
			break;
		case M_RUN_MAX_IZQ:
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,9999);
			M2=M_STB;
			break;
		case M_RUN_MID_IZQ:
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,7000);
			M2=M_STB;
			break;
		case M_RUN_MID_DER:
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,3000);
			M2=M_STB;
			break;
	}
}

void LOC_process(){
	switch(LOC){
		case LOC_CHECK:
			if(IR1==IR_BLANCO && IR2==IR_NEGRO){
				M1=M_STOP;
				M2=M_RUN_MID_DER;
			}else if(IR2==IR_BLANCO && IR1==IR_NEGRO){
				M2=M_STOP;
				M1=M_RUN_MID_DER;
			}else if(IR1==IR_BLANCO && IR2==IR_BLANCO){
				M1=M_RUN_MID_IZQ;
				M2=M_RUN_MID_IZQ;
				TIMERS[T_MOTOR]=9000;
				LOC=LOC_STB;
			}else if(IR3==IR_BLANCO && IR4==IR_NEGRO){
				M2=M_STOP;
				M1=M_RUN_MID_DER;
			}else if(IR4==IR_BLANCO && IR3==IR_NEGRO){
				M1=M_STOP;
				M2=M_RUN_MID_DER;
			}else if(IR3==IR_BLANCO && IR4==IR_BLANCO){
				M1=M_RUN_MID_DER;
				M2=M_RUN_MID_DER;
				TIMERS[T_MOTOR]=9000;
				LOC=LOC_STB;
			}else if(IR1==IR_NEGRO && IR2==IR_NEGRO && IR3==IR_NEGRO && IR4==IR_NEGRO){
				if(SP==IR_ON){
					M1=M_RUN_MID_DER;
					M2=M_RUN_MID_IZQ;
				}else if(SP==IR_OFF){
					M2=M_RUN_MID_DER;
					M1=M_RUN_MID_DER;
				}
			}
			break;
		case LOC_STB:
			if(TIMERS[T_MOTOR]==0){
				LOC=LOC_CHECK;
			}
			break;
	}
}

void B1_process(){
	switch(B1){
		case B_STB:
			if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_10)==1){
				TIMERS[T_BOTON]=50000;
				B1=B_COUNT;
			}
			break;
		case B_COUNT:
			if(TIMERS[T_BOTON]==0){
				B1=B_STB;
				aux=1;
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
