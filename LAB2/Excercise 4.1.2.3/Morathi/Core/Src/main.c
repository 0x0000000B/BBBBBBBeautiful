/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "software_timer.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "FreeRTOS.h"
//#include "task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Define the array for number patterns
	uint8_t numbers[10][7] = {
    {1,1,1,1,1,1,0}, // 0
    {0,1,1,0,0,0,0}, // 1
    {1,1,0,1,1,0,1}, // 2
    {1,1,1,1,0,0,1}, // 3
    {0,1,1,0,0,1,1}, // 4
    {1,0,1,1,0,1,1}, // 5
    {1,0,1,1,1,1,1}, // 6
    {1,1,1,0,0,0,0}, // 7
    {1,1,1,1,1,1,1}, // 8
    {1,1,1,1,0,1,1}  // 9
};

#define TOTAL_LEDS 12
	uint16_t LED_PINS[TOTAL_LEDS] = {C1_Pin, C2_Pin, C3_Pin, C4_Pin, C5_Pin, C6_Pin, C7_Pin, C8_Pin, C9_Pin, C10_Pin, C12_Pin}; // Adjust pin numbers
	GPIO_TypeDef* LED_GPIO_PORT = GPIOA; // Adjust to your GPIO port

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
	int hour = 0, minute = 0, second = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
void vLEDControlTask_handler(void *params);
void controlLED(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState) {
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, PinState);}
void LEDs_Off() {
    for (int i = 0; i < TOTAL_LEDS; i++) {
        HAL_GPIO_WritePin(LED_GPIO_PORT, LED_PINS[i], GPIO_PIN_RESET);
    }
}

// Function to display hour, minute, and second
void displayTime(int hour, int minute, int second) {
    LEDs_Off(); // Turn off all LEDs initially

    // Calculate LED indices (0-11) for hour, minute, and second
    int hourLED = hour % 12;
    int minuteLED = (minute / 5) % 12; // Every 5 minutes
    int secondLED = (second / 5) % 12; // Every 5 seconds

    // Turn on LEDs for hour, minute, and second. Ensure they are unique to avoid overwriting.
    if(hourLED == minuteLED || hourLED == secondLED) hourLED = (hourLED + 1) % 12;
    if(minuteLED == secondLED) minuteLED = (minuteLED + 1) % 12;

    HAL_GPIO_WritePin(LED_GPIO_PORT, LED_PINS[hourLED], GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_GPIO_PORT, LED_PINS[minuteLED], GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_GPIO_PORT, LED_PINS[secondLED], GPIO_PIN_SET);
}
// Function to display a number
void displayNumber(uint8_t num) {
    GPIO_PinState segmentState[7];
    for(int i = 0; i < 7; i++) {
        //segmentState[i] = numbers[num][i] ? GPIO_PIN_SET : GPIO_PIN_RESET;
    	if (numbers[num][i]){
    		segmentState[i] = GPIO_PIN_RESET; //inverted SET/RESET value due to wiring, particularly RESET to turn ON the SEG LED
    	}
    	else {
    		segmentState[i] = GPIO_PIN_SET;
    	}
    }

    HAL_GPIO_WritePin(SEG1_GPIO_Port, SEG1_Pin, segmentState[0]);
    HAL_GPIO_WritePin(SEG2_GPIO_Port, SEG2_Pin, segmentState[1]);
    HAL_GPIO_WritePin(SEG3_GPIO_Port, SEG3_Pin, segmentState[2]);
    HAL_GPIO_WritePin(SEG4_GPIO_Port, SEG4_Pin, segmentState[3]);
    HAL_GPIO_WritePin(SEG5_GPIO_Port, SEG5_Pin, segmentState[4]);
    HAL_GPIO_WritePin(SEG6_GPIO_Port, SEG6_Pin, segmentState[5]);
    HAL_GPIO_WritePin(SEG7_GPIO_Port, SEG7_Pin, segmentState[6]);
}
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
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim1);

  setTimer(0, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (isTimerExpired(0) == 1) {
	  		//Up
	  		HAL_GPIO_WritePin(C1_GPIO_Port, C1_Pin, GPIO_PIN_SET); 			//RED - ON
	  		HAL_GPIO_WritePin(C12_GPIO_Port, C12_Pin, GPIO_PIN_RESET);	 	//YELLOW - OFF
	  		HAL_GPIO_WritePin(C11_GPIO_Port, C11_Pin, GPIO_PIN_RESET); 		//GREEN - OFF

	  		//Down
	  		HAL_GPIO_WritePin(C7_GPIO_Port, C7_Pin, GPIO_PIN_SET); 			//RED - ON
	  		HAL_GPIO_WritePin(C6_GPIO_Port, C6_Pin, GPIO_PIN_RESET); 		//YELLOW - OFF
	  		HAL_GPIO_WritePin(C5_GPIO_Port, C5_Pin, GPIO_PIN_RESET); 		//GREEN - OFF

	  		//Left
	  		HAL_GPIO_WritePin(C10_GPIO_Port, C10_Pin, GPIO_PIN_RESET); 		//RED - OFF
	  		HAL_GPIO_WritePin(C9_GPIO_Port, C9_Pin, GPIO_PIN_RESET);	 	//YELLOW - OFF
	  		HAL_GPIO_WritePin(C8_GPIO_Port, C8_Pin, GPIO_PIN_SET); 			//GREEN - ON

	  		//Right
	  		HAL_GPIO_WritePin(C4_GPIO_Port, C4_Pin, GPIO_PIN_RESET); 		//RED - OFF
	  		HAL_GPIO_WritePin(C3_GPIO_Port, C3_Pin, GPIO_PIN_RESET);	 	//YELLOW - OFF
	  		HAL_GPIO_WritePin(C2_GPIO_Port, C2_Pin, GPIO_PIN_SET); 			//GREEN - ON

	  		setTimer(1, 300);
	  	}
	  	if (isTimerExpired(1) == 1) {
	  		//Up
	  		HAL_GPIO_WritePin(C1_GPIO_Port, C1_Pin, GPIO_PIN_SET); 			//RED - ON
	  		HAL_GPIO_WritePin(C12_GPIO_Port, C12_Pin, GPIO_PIN_RESET);	 	//YELLOW - OFF
	  		HAL_GPIO_WritePin(C11_GPIO_Port, C11_Pin, GPIO_PIN_RESET); 		//GREEN - OFF

	  		//Down
	  		HAL_GPIO_WritePin(C7_GPIO_Port, C7_Pin, GPIO_PIN_SET); 			//RED - ON
	  		HAL_GPIO_WritePin(C6_GPIO_Port, C6_Pin, GPIO_PIN_RESET); 		//YELLOW - OFF
	  		HAL_GPIO_WritePin(C5_GPIO_Port, C5_Pin, GPIO_PIN_RESET); 		//GREEN - OFF

	  		//Left
	  		HAL_GPIO_WritePin(C10_GPIO_Port, C10_Pin, GPIO_PIN_RESET); 		//RED - OFF
	  		HAL_GPIO_WritePin(C9_GPIO_Port, C9_Pin, GPIO_PIN_SET);	 		//YELLOW - ON
	  		HAL_GPIO_WritePin(C8_GPIO_Port, C8_Pin, GPIO_PIN_RESET); 		//GREEN - OFF

	  		//Right
	  		HAL_GPIO_WritePin(C4_GPIO_Port, C4_Pin, GPIO_PIN_RESET); 		//RED - OFF
	  		HAL_GPIO_WritePin(C3_GPIO_Port, C3_Pin, GPIO_PIN_SET);	 		//YELLOW - ON
	  		HAL_GPIO_WritePin(C2_GPIO_Port, C2_Pin, GPIO_PIN_RESET); 		//GREEN - OFF

	  		setTimer(2, 200);
	  	}
	  	if (isTimerExpired(2) == 1) {
	  			//Up
	  			HAL_GPIO_WritePin(C1_GPIO_Port, C1_Pin, GPIO_PIN_RESET); 		//RED - OFF
	  			HAL_GPIO_WritePin(C12_GPIO_Port, C12_Pin, GPIO_PIN_RESET);	 	//YELLOW - OFF
	  			HAL_GPIO_WritePin(C11_GPIO_Port, C11_Pin, GPIO_PIN_SET); 		//GREEN - ON

	  			//Down
	  			HAL_GPIO_WritePin(C7_GPIO_Port, C7_Pin, GPIO_PIN_RESET); 			//RED - OFF
	  			HAL_GPIO_WritePin(C6_GPIO_Port, C6_Pin, GPIO_PIN_RESET); 		//YELLOW - OFF
	  			HAL_GPIO_WritePin(C5_GPIO_Port, C5_Pin, GPIO_PIN_SET); 		//GREEN - ON

	  			//Left
	  			HAL_GPIO_WritePin(C10_GPIO_Port, C10_Pin, GPIO_PIN_SET); 		//RED - ON
	  			HAL_GPIO_WritePin(C9_GPIO_Port, C9_Pin, GPIO_PIN_RESET);	 		//YELLOW - OFF
	  			HAL_GPIO_WritePin(C8_GPIO_Port, C8_Pin, GPIO_PIN_RESET); 		//GREEN - OFF

	  			//Right
	  			HAL_GPIO_WritePin(C4_GPIO_Port, C4_Pin, GPIO_PIN_SET); 		//RED - ON
	  			HAL_GPIO_WritePin(C3_GPIO_Port, C3_Pin, GPIO_PIN_RESET);	 		//YELLOW - OFF
	  			HAL_GPIO_WritePin(C2_GPIO_Port, C2_Pin, GPIO_PIN_RESET); 		//GREEN - OFF

	  			setTimer(3, 300);
	  		}
	  	if (isTimerExpired(3) == 1) {
	  				//Up
	  				HAL_GPIO_WritePin(C1_GPIO_Port, C1_Pin, GPIO_PIN_RESET); 		//RED - OFF
	  				HAL_GPIO_WritePin(C12_GPIO_Port, C12_Pin, GPIO_PIN_SET);	 	//YELLOW - ON
	  				HAL_GPIO_WritePin(C11_GPIO_Port, C11_Pin, GPIO_PIN_RESET); 		//GREEN - OFF

	  				//Down
	  				HAL_GPIO_WritePin(C7_GPIO_Port, C7_Pin, GPIO_PIN_RESET); 			//RED - OFF
	  				HAL_GPIO_WritePin(C6_GPIO_Port, C6_Pin, GPIO_PIN_SET); 		//YELLOW - ON
	  				HAL_GPIO_WritePin(C5_GPIO_Port, C5_Pin, GPIO_PIN_RESET); 		//GREEN - OFF

	  				//Left
	  				HAL_GPIO_WritePin(C10_GPIO_Port, C10_Pin, GPIO_PIN_SET); 		//RED - ON
	  				HAL_GPIO_WritePin(C9_GPIO_Port, C9_Pin, GPIO_PIN_RESET);	 		//YELLOW - OFF
	  				HAL_GPIO_WritePin(C8_GPIO_Port, C8_Pin, GPIO_PIN_RESET); 		//GREEN - OFF

	  				//Right
	  				HAL_GPIO_WritePin(C4_GPIO_Port, C4_Pin, GPIO_PIN_SET); 		//RED - ON
	  				HAL_GPIO_WritePin(C3_GPIO_Port, C3_Pin, GPIO_PIN_RESET);	 		//YELLOW - OFF
	  				HAL_GPIO_WritePin(C2_GPIO_Port, C2_Pin, GPIO_PIN_RESET); 		//GREEN - OFF

	  				setTimer(0, 200);
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 7999;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 9;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, C1_Pin|C2_Pin|C3_Pin|C4_Pin
                          |C5_Pin|C6_Pin|C7_Pin|C8_Pin
                          |C9_Pin|C10_Pin|C11_Pin|C12_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RV_Pin|YV_Pin|SEG2_Pin|SEG3_Pin
                          |SEG4_Pin|SEG5_Pin|SEG6_Pin|SEG7_Pin
                          |GV_Pin|SEG1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : C1_Pin C2_Pin C3_Pin C4_Pin
                           C5_Pin C6_Pin C7_Pin C8_Pin
                           C9_Pin C10_Pin C11_Pin C12_Pin */
  GPIO_InitStruct.Pin = C1_Pin|C2_Pin|C3_Pin|C4_Pin
                          |C5_Pin|C6_Pin|C7_Pin|C8_Pin
                          |C9_Pin|C10_Pin|C11_Pin|C12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : RV_Pin YV_Pin SEG2_Pin SEG3_Pin
                           SEG4_Pin SEG5_Pin SEG6_Pin SEG7_Pin
                           GV_Pin SEG1_Pin */
  GPIO_InitStruct.Pin = RV_Pin|YV_Pin|SEG2_Pin|SEG3_Pin
                          |SEG4_Pin|SEG5_Pin|SEG6_Pin|SEG7_Pin
                          |GV_Pin|SEG1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	timerRun();
}
void vLEDControlTask_handler(void *params)
{
    while(1)
    {
        /* Turn RED LED ON and Yellow LED OFF */
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);   // RED LED ON
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET); // Yellow LED OFF
        //vTaskDelay(pdMS_TO_TICKS(2000)); // Wait for 2 seconds

        /* Turn RED LED OFF and Yellow LED ON */
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); // RED LED OFF
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);   // Yellow LED ON
        //vTaskDelay(pdMS_TO_TICKS(2000)); // Wait for 2 seconds
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
