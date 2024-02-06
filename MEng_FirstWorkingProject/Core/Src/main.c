/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PROGRAM 1000
#define DELAY 1001
#define WIDTH 1002
#define ARMED 1003

#define SENTENCESIZE 32

#define RESPONSETIME 64 //Time from external trigger to timer starting (measured with oscilloscope
#define QTIME 380 //time between trigger pulse and light
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint32_t CLOCKTIME = 10;
uint32_t lineDelay_ns = 0; //delay from trigger pulse line + rf line to maser (needs to be measured)
uint32_t swithDelay_ns = 150; //The switch turn on time (from the datasheet)

int current_state = PROGRAM;
uint32_t targetDelay_ns = 0; //The desired delay for the pulse
uint32_t targetWidth_ns = 20000; //The desired width of the pulse
uint32_t registerDelay = 1; //The value for delay that will be put in the register
uint32_t registerWidth = 2000; //The value for width that will be put in the register


uint8_t newLine[1] = {0x0A};
uint8_t carriageReturn[1] = {0x0D};

uint8_t UART1_rxBuffer[1] = {0};
uint8_t commandBuffer[SENTENCESIZE] = {0};
uint8_t commandIndex = 0;


//Here we define the words to look for
uint8_t width_call[5] = "width";
uint8_t delay_call[5] = "delay";
uint8_t arm_call[3] = "arm";
uint8_t stop_call[4] = "stop";

uint8_t read_call[5] = "read";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void Control_HandleMessage(uint8_t[]);
int Message_Equality(uint8_t[],uint8_t[], int);
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
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  HAL_UART_Receive_IT(&huart1, UART1_rxBuffer, 1); //Prepare the UART Interrupt for receiving the commands

  TIM2->ARR = registerWidth; // Time pulse is on = ((ARR) * time per clock tick)-CCR3
  TIM2->CCR3 = registerDelay; // if it is 1 there is no delay (measured reaction time is 64ns). numbers greater than 1 add multiple of time per clock tick

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim2, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_RISING;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PowerLED_GPIO_Port, PowerLED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ArmedLED_GPIO_Port, ArmedLED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PowerLED_Pin ArmedLED_Pin */
  GPIO_InitStruct.Pin = PowerLED_Pin|ArmedLED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int Message_Equality(uint8_t *message, uint8_t *keyWord, int keyword_size){
/*Gets the message from the UART (message +trailing 0s) and compares the first characters of it with a keyword.
 * It also checks whether the next character is 0 to confirm that was the message. Returns 1 if equal.*/
	for(int i=0; i<keyword_size; i++){
		if (message[i] != keyWord[i]){
			return 0;
		}
	}
	if (message[keyword_size] == 0){
		return 1;
	}
	return 0;
}


void Control_HandleMessage(uint8_t *message){
/*Function gets the UART message buffer and acts on it based on the current state of the device*/
	switch(current_state){
	case PROGRAM:
		HAL_UART_Transmit(&huart1, "Program Mode", 12, 100);
		HAL_UART_Transmit(&huart1, newLine, 1, 100); //\n
		HAL_UART_Transmit(&huart1, carriageReturn, 1, 100); //\r

		//WIDTH
		if (Message_Equality(message, width_call, 5)){
			HAL_UART_Transmit(&huart1, "type width (ns):", 16, 100);
			HAL_UART_Transmit(&huart1, newLine, 1, 100); //\n
			HAL_UART_Transmit(&huart1, carriageReturn, 1, 100); //\r
			current_state = WIDTH;
		}
		//DELAY
		if (Message_Equality(message, delay_call, 5)){
			HAL_UART_Transmit(&huart1, "type delay (ns):", 16, 100);
			HAL_UART_Transmit(&huart1, newLine, 1, 100); //\n
			HAL_UART_Transmit(&huart1, carriageReturn, 1, 100); //\r
			current_state = DELAY;
		}
		//READ
		if (Message_Equality(message, read_call, 5)){
			uint8_t number_out_str[8] = {0};
			sprintf(number_out_str, "%d", targetDelay_ns);

			HAL_UART_Transmit(&huart1, "delay (ns): ", 12, 100);
			HAL_UART_Transmit(&huart1, number_out_str, 8, 100);
			HAL_UART_Transmit(&huart1, newLine, 1, 100); //\n
			HAL_UART_Transmit(&huart1, carriageReturn, 1, 100); //\r

			for (int i = 0; i<8; i++){
				number_out_str[i]= 0;
			}
			sprintf(number_out_str, "%d", registerDelay);

			HAL_UART_Transmit(&huart1, "delay (CCR): ", 13, 100);
			HAL_UART_Transmit(&huart1, number_out_str, 8, 100);
			HAL_UART_Transmit(&huart1, newLine, 1, 100); //\n
			HAL_UART_Transmit(&huart1, carriageReturn, 1, 100); //\r

			for (int i = 0; i<8; i++){
				number_out_str[i]= 0;
			}

			sprintf(number_out_str, "%d", targetWidth_ns);

			HAL_UART_Transmit(&huart1, "width (ns): ", 12, 100);
			HAL_UART_Transmit(&huart1, number_out_str, 8, 100);
			HAL_UART_Transmit(&huart1, newLine, 1, 100); //\n
			HAL_UART_Transmit(&huart1, carriageReturn, 1, 100); //\r
			for (int i = 0; i<8; i++){
				number_out_str[i]= 0;
			}

			sprintf(number_out_str, "%d", registerWidth);

			HAL_UART_Transmit(&huart1, "width (ARR): ", 13, 100);
			HAL_UART_Transmit(&huart1, number_out_str, 8, 100);
			HAL_UART_Transmit(&huart1, newLine, 1, 100); //\n
			HAL_UART_Transmit(&huart1, carriageReturn, 1, 100); //\r
		}
		//ARM
		if (Message_Equality(message, arm_call, 3)){
			HAL_UART_Transmit(&huart1, "ARMING", 6, 100);
			HAL_UART_Transmit(&huart1, newLine, 1, 100); //\n
			HAL_UART_Transmit(&huart1, carriageReturn, 1, 100); //\r

			current_state = ARMED;
			HAL_GPIO_WritePin(GPIOA, ArmedLED_Pin, GPIO_PIN_SET);

			TIM2->ARR = registerWidth; // Time pulse is on = ((ARR) * time per clock tick)-CCR3
			TIM2->CCR3 = registerDelay; // if it is 1 there is no delay (measured reaction time is 64ns).
			HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_3); //Starts the timer in interrupt mode
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
		}
		break;
	case ARMED:
		if (Message_Equality(message, stop_call, 4)){
			HAL_TIM_PWM_Stop_IT(&htim2, TIM_CHANNEL_3);
			current_state = PROGRAM;
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, ArmedLED_Pin, GPIO_PIN_RESET);
		}
		break;
	default:
		//if not armed or programming, it is in delay or width setting mode so we start by interpreting a number
		uint32_t value = 0;
		uint32_t multiplier = 1;
		for (int i = SENTENCESIZE-1; i>=0; i--){
			if (message[i] >= 0x30){
				value += (((uint32_t) message[i])- 0x30) * multiplier;
				multiplier = multiplier * 10;
			}
		}
		if (current_state == WIDTH){
			targetWidth_ns = value;
			registerWidth = (value/CLOCKTIME) + registerDelay;
			current_state = PROGRAM;
		}else {
			targetDelay_ns = value;
			registerDelay = ((value+QTIME-(RESPONSETIME + lineDelay_ns + swithDelay_ns))/CLOCKTIME)+1;
			registerWidth = (targetWidth_ns/CLOCKTIME) + registerDelay;
			current_state = PROGRAM;
		}

	}
}


//------------------INTERRUPT HANDLING-----------------------------//
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){ //Function callback on UART data received
    if (UART1_rxBuffer[0] == 0x0D){
    	HAL_UART_Transmit(&huart1, newLine, 1, 100); //\n
    	HAL_UART_Transmit(&huart1, carriageReturn, 1, 100); //\r

    	//interpret the message
    	Control_HandleMessage(commandBuffer);
    	//reset the buffer
    	commandIndex = 0;
    	for(int i = 0; i < SENTENCESIZE; i++){ //reset the buffer to 00000...
    		commandBuffer[i] = 0;
    	}
    }else {
    	if (commandIndex != 0 && UART1_rxBuffer[0] == 0x08){
    		commandIndex--;
    		commandBuffer[commandIndex] = 0;
    	}else {
    		commandBuffer[commandIndex] = UART1_rxBuffer[0];
    		commandIndex++;
    	}
    	HAL_UART_Transmit(&huart1, carriageReturn, 1, 100); //\r
    	HAL_UART_Transmit(&huart1, commandBuffer, SENTENCESIZE, 100);
    }

	HAL_UART_Receive_IT(&huart1, UART1_rxBuffer, 1); //Prepare the UART Interrupt for receiving the commands again
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
