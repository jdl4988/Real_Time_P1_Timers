/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include <string.h>
#include <stdio.h>
#include <ctype.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MIN_PERIOD 50
#define MAX_PERIOD 9950
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

// ask user for expected period, sets timer clock accordingly. Return period or 0 if invalid
int set_timer_base( int i );

//POST
_bool power_on_self_test(void);

//prints user messages
void print_bad_input(void);
void print_setting_clock(void);

// Captures 1 line of text from console. Returns nul terminated string when \n is entered
void get_line ( void *buffer, int max_length);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char message[128];
char buffer[128];
char invalid_messaage[] = "Invalid period input\r\n";
char setting_clock_messaage[] = "Setting Clock Parameters\r\n";
int buckets[MAX_PERIOD+100+1];
int duty_buckets[101];

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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  //start timer 2
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);

  //configure some variables to be used later
  int mess_len = sprintf((char *)message, "Enter expected period or <CR> if no change\r\n");
  int n = 0;
  int period = 950;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //send the query message
	  HAL_UART_Transmit(&huart2, (void*)&message, mess_len, HAL_MAX_DELAY);

	  //POST
	  while(power_on_self_test() == false);

	  //store the old period in case the user wants the same one
	  int old_period = period;

	  //get the new period and keep asking until a valid input is given
	  period = set_timer_base(old_period);
	  while(period == 0){
		  period = set_timer_base(old_period);
	  }


	  //clear the buckets
	  memset(buckets, 0, MAX_PERIOD+100);
	  memset(duty_buckets, 0, 100);

	  //confirm the period
	  n = sprintf((char *)buffer, "Period is %d\r\n", period);
	  HAL_UART_Transmit(&huart2, (void*)&buffer, n, HAL_MAX_DELAY);


	  //wait for the next rising edge. This is to allow the duty cycle calculation to be correct
	  while((GPIOA->IDR)&1){
		  continue;
	  }

	  //get the initial timer value
	  uint32_t tim_val1 = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);
	  uint32_t tim_val2 = tim_val1;

	  //capture 100 cycles
	  for(int i = 0; i < 1000; i++){
		  //keep checking for a new value
		  while(tim_val2 == tim_val1){
		  	  tim_val2 = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);
		  }
		  //find the time for the high portion of the wave
		  uint32_t up_diff = tim_val2-tim_val1;
		  tim_val1 = tim_val2;
		  //wait until the next edge
		  while(tim_val2 == tim_val1){
			  tim_val1 = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);
		  }

		  //find the time of the low signal
		  uint32_t down_diff = tim_val1-tim_val2;

		  //find the total wave period
		  uint32_t total_diff = up_diff+down_diff;

		  //increment the period at the given location
		  buckets[total_diff]++;

		  //calculate the duty cycle
		  int cycle = (up_diff*100)/total_diff;
		  //increment the duty cycle bucket
		  duty_buckets[100-cycle]++;
		  //set the value for the next loop
		  tim_val2 = tim_val1;
	  }

	  //print out the period buckets
	  n = sprintf((char *)buffer, "Timing Buckets:\r\n");
	  HAL_UART_Transmit(&huart2, (void*)&buffer, n, HAL_MAX_DELAY);

	  for(int i = 0; i <= MAX_PERIOD+100; i++){
		  //only print non empty buckets
		  if(buckets[i] != 0){
			  n = sprintf((char *)buffer, "%d %d\r\n", i, buckets[i]);
			  HAL_UART_Transmit(&huart2, (void*)&buffer, n, HAL_MAX_DELAY);
			  //reset the bucket value
			  buckets[i] = 0;
		  }
	  }

	  //print out the duty cycle buckets
	  n = sprintf((char *)buffer, "Duty Cycle Buckets:\r\n");
	  HAL_UART_Transmit(&huart2, (void*)&buffer, n, HAL_MAX_DELAY);
	  for(int i = 0; i <= 100; i++){
		  //only print non empty buckets
	  	  if(duty_buckets[i] != 0){
	  		  n = sprintf((char *)buffer, "%%%d %d\r\n", i, duty_buckets[i]);
	  		  HAL_UART_Transmit(&huart2, (void*)&buffer, n, HAL_MAX_DELAY);
	  		  duty_buckets[i] = 0;

	  	  }
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 79;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
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
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/*
 * POST method to ensure proper bring-up
 */
_bool power_on_self_test(void){
	 uint32_t tim_val1 = TIM2->CCR1;
	 uint32_t tim_val2 = tim_val1;
	 uint32_t start_time = TIM2->CNT;
	 while(1){
		if(tim_val1 == tim_val2){
			tim_val2 = TIM2->CCR1;
			if(TIM2->CNT - start_time > 100000){
				return false;
			}
		}
		else{
			return true;
		}
	 }
}

/*
 * prints the message indicating clock settings are updating
 */
void print_setting_clock(){
	HAL_UART_Transmit(&huart2, (void*)&setting_clock_messaage, sizeof(setting_clock_messaage), HAL_MAX_DELAY);
}

/*
 * tells the user bad input was given
 */
void print_bad_input(void){
	HAL_UART_Transmit(&huart2, (void*)&invalid_messaage, sizeof(invalid_messaage), HAL_MAX_DELAY);
}

/*
 * parses the user input and returns the new period or an error
 */
int set_timer_base(int old_period){
	//get the next user input
	get_line(buffer, sizeof(buffer));
	//if the string is empty return error
	if(buffer[0] == 0){
		print_bad_input();
		return 0;
	}
	//if the entry is a CR return the old period
	else if(buffer[0] == 13){
		return old_period;
	}
	//current temp period
	int period = 0;
	for(int i = 0; i < sizeof(buffer); i++){
		//stop at the newline
		if(buffer[i] == 13){
			break;
		}
		//make sure input is a number
		if(buffer[i] > 57 || buffer[i] < 48){
			print_bad_input();
			return 0;
		}
		//multiply temp period by 10 then add the current digit
		period *= 10;
		period += (buffer[i]-48);
		//make sure the period isnt too large
		if(period > MAX_PERIOD){
			print_bad_input();
			return 0;
		}
	}
	//make sure the period isnt too small
	if(period < MIN_PERIOD){
		print_bad_input();
		return 0;
	}
	print_setting_clock();
	return period;
}

/*
 * reads in the uart bytes until the next newline
 */
void get_line( void *buffer, int max_length){
	//temp variables
	int char_count = 0;
	char out_buf[max_length];

	//loop until the max length of the buffer
	while(char_count < max_length-1){
		char rxByte;
		//receive the next byte
		HAL_UART_Receive(&huart2, (void*)&rxByte, 1, HAL_MAX_DELAY);
		//store the byte in the temp buffer
		out_buf[char_count] = rxByte;
		//increment the index
		char_count++;
		//loopback the user input so they can see what they put in
		HAL_UART_Transmit(&huart2, (void*)&rxByte, 1, HAL_MAX_DELAY);
		//break on new line
		if(rxByte == 13){
			if(char_count == 1){
				//if a newline was entered only, clear the output buffer
				memset(buffer, 0, max_length);
			}
			break;
		}
	}
	memcpy(buffer, out_buf, max_length);
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
