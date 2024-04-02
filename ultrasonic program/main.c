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
#include <string.h>
#include <stdio.h>

#define MSG_BUF_SIZE 80

UART_HandleTypeDef huart2;

volatile int distance;
//volatile int show_distance;
char msg[MSG_BUF_SIZE];

volatile uint32_t start_time = 0;
volatile uint32_t end_time = 0;
volatile uint8_t capture_done = 0;
volatile uint8_t IRstate1 = 0;
volatile uint8_t IRstate2 = 0;
volatile uint8_t IRstate3 = 0;
volatile uint8_t IRstate4 = 0;
volatile uint8_t IRstate5 = 0;

void DisplayDistance(int dist);
void pin_init(void);
void TIM3_Init(void);
void TIM4_Init(void);


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  pin_init();

  // Initialize TIM3 and TIM4
  TIM3_Init();
  TIM4_Init();

  while (1)
  {

  }
}

void pin_init(void) {
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN; // Enable GPIOB clock

    GPIOB->MODER |= GPIO_MODER_MODER6_1;  // Set PB6 to alternate function mode
    GPIOB->AFR[0] |= (GPIOB->AFR[0] & ~GPIO_AFRL_AFRL6) | (0b0010 << GPIO_AFRL_AFRL6_Pos); // Set PB6 to use TIM4_CH1

    // Configure PB5 for alternate function mode (TIM3)
    GPIOB->MODER |= GPIO_MODER_MODER5_1;  //x Set PB5 to alternate function mode
    GPIOB->AFR[0] |= (GPIOB->AFR[0] & ~GPIO_AFRL_AFRL5)| (0b0010 << GPIO_AFRL_AFRL5_Pos); // Set PB5 to use TIM3_CH2 (AF2)
}

void TIM3_Init(void) {
    // Enable the TIM3 peripheral clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    // Configure TIM3 in PWM mode
    TIM3->PSC = 15;       // Prescaler set to 15
    TIM3->ARR = 0xFFFF;   // Auto-reload value (maximum value for 16-bit timer)
    TIM3->CCR2 = 10;      // Set the capture/compare value for channel 2
    TIM3->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;  // PWM mode 1
    TIM3->CCER |= TIM_CCER_CC2E;  // Enable capture/compare channel 2

    // Start the timer
    TIM3->CR1 |= TIM_CR1_CEN;
}


void TIM4_Init(void) {
	// Enable the clock of timer 4
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

	// Set the prescaler register to divide the 16MHz clock down to 1MHz
	TIM4->PSC = 15;

	// Set the auto-reload register to the maximum value
	TIM4->ARR = 0xFFFF;

	// Configure capture mode for Channel 1 (CCMR1 register)
	TIM4->CCMR1 |= TIM_CCMR1_CC1S_0;

	// Set the input filter duration to 0 in CCMR1 for Channel 1
	TIM4->CCMR1 &= ~(TIM_CCMR1_IC1F);

	// Set the input prescaler so that we capture each transition for Channel 1
	TIM4->CCMR1 &= ~(TIM_CCMR1_IC1PSC);

	// Set the capture to be on both rising and falling edges in CCER for Channel 1
	TIM4->CCER |= (TIM_CCER_CC1P | TIM_CCER_CC1NP);

	// Enable capture for Channel 1
	TIM4->CCER |= TIM_CCER_CC1E;

	// Enable the update interrupt for Timer 4
	TIM4->DIER |= TIM_DIER_UIE;

	// Enable Timer 4
	TIM4->CR1 |= TIM_CR1_CEN;

	// Enable the interrupt in the NVIC
	NVIC_EnableIRQ(TIM4_IRQn);
}

void DisplayDistance(int dist)
{
	snprintf(msg, MSG_BUF_SIZE, "Distance: %d\n", dist);
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
}

//void debugSession(){
//	snprintf(msg, MSG_BUF_SIZE, "interrupt occured", dist);
//	HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
//}

void TIM4_IRQHandler(void)
{
    if (TIM4->SR & TIM_SR_CC1IF) {
//    	debugSession();
        TIM4->SR &= ~TIM_SR_CC1IF;
        if (capture_done == 0) {
            start_time = TIM4->CCR1;
            capture_done = 1;
        } else {
            end_time = TIM4->CCR1;
            // Speed of sound is approximately 343 meters per second or 0.0343 cm per microsecond
            uint32_t pulse_duration = end_time - start_time;
            distance = (pulse_duration * 0.0343);
            DisplayDistance(distance);
            // Do something with the distance data, e.g., send it over UART or process it as needed
            capture_done = 0;
        }
    }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart2.Init.BaudRate = 38400;
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
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
