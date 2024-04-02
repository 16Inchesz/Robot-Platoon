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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

//constants
const int MOTOR_STOP_PWM = 1500;
const int RIGHT_MOTOR_FORWARD = 1480;	//RMF - PID output = servo pulse
const int LEFT_MOTOR_FORWARD = 1520; 	//LMF + PID output = servo pulse
const int MAX_OUTPUT = 200;

/* PID control variables */
const double Kp = 0.700;			//change for base speed
const double Ki = 0.150;		//change for rate of speed up
const double Kd = 0.100;
const double SETPOINT = 10;
const int PID_THRESHOLD = 15;

//distance measuring variables
volatile uint32_t distance = 0;
volatile uint32_t start_time = 0;
volatile uint32_t end_time = 0;
volatile uint8_t capture_done = 0;

int pid_state = 0;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

//configuration functions
void Pin_Config(void);		//ultrasonic (in + out), servo and infrared
void Timer2_Config(void);	//servo output
void Timer3_Config(void);	//ultrasonic output
void Timer4_Config(void);	//ultrasonic input

//map function
long map(long x, long in_min, long in_max, long out_min, long out_max);

//motor control functions(value 0-100%)
void Right_Wheel_Forward_Rotation(double value);
void Left_Wheel_Forward_Rotation(double value);
void Stop_Wheels();
void Servo_Controller(double pid_output);

//function to convert PID output to steering ready variable
double convert_output(double pid_output);

void Steering_Algorithm(double refined_value);

//PID functions
double PID_Controller(double current_value);
void PID_Controller_loop();


int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();

  Pin_Config();
  Timer2_Config();
  Timer3_Config();
  Timer4_Config();
  /* Infinite loop */
  while (1)
  {
	  if (distance > PID_THRESHOLD){
		  pid_state = 0;
		  Steering_Algorithm(60);
	  }
	  else{
		  pid_state = 1;
		  PID_Controller_loop();
		  if(distance < SETPOINT)
		  {
			  Stop_Wheels();
		  }
	  }

  }
}

//******************************************CONFIGURATION*************************************************
void Pin_Config(){
	//enabling clocks
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;	//Port A clock
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;	//Port B clock

	//setting MODER for PA1 and PA0 (alternating function) /servo motors
	GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODER1)) | (0b10 << GPIO_MODER_MODER1_Pos);
	GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODER0)) | (0b10 << GPIO_MODER_MODER0_Pos);
	//setting MODER for PB5 and PB6 (alternation function) /timer I/O
	GPIOB->MODER = (GPIOB->MODER & ~(GPIO_MODER_MODER5)) | (0b10 << GPIO_MODER_MODER5_Pos);
	GPIOB->MODER = (GPIOB->MODER & ~(GPIO_MODER_MODER6)) | (0b10 << GPIO_MODER_MODER6_Pos);

	//setting alternate function AF1 for GPIO PA1 and PA0
	GPIOA->AFR[0] = (GPIOA->AFR[0] & ~(GPIO_AFRL_AFRL1)) | (0b0001 << GPIO_AFRL_AFRL1_Pos);
	GPIOA->AFR[0] = (GPIOA->AFR[0] & ~(GPIO_AFRL_AFRL0)) | (0b0001 << GPIO_AFRL_AFRL0_Pos);
	//setting alternate function AF for GPIO PB5 and PB6
	GPIOB->AFR[0] = (GPIOB->AFR[0] & ~(GPIO_AFRL_AFRL5)) | (0b0010 << GPIO_AFRL_AFRL5_Pos);
	GPIOB->AFR[0] = (GPIOB->AFR[0] & ~(GPIO_AFRL_AFRL6)) | (0b0010 << GPIO_AFRL_AFRL6_Pos);

    // Configure the GPIO pins as input, pins for infrared
    GPIOA->MODER &= ~(GPIO_MODER_MODER10 | GPIO_MODER_MODER8);
    GPIOB->MODER &= ~(GPIO_MODER_MODER3 | GPIO_MODER_MODER4 | GPIO_MODER_MODER10);
}


void Timer2_Config(){
	//enable timer clock
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	//set the PreScaler value.
	TIM2->PSC = 144;	//servo PSC
	//set the auto reload register value.
	TIM2->ARR = 10000;	//servo ARR
	//reset the counter
	TIM2->CNT &= ~(TIM_CNT_CNT_Msk);
	//enable counter
	TIM2->CR1 |= TIM_CR1_CEN;
	//set the output compare mode (PWM)
	TIM2->CCMR1 |= (TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2);
	TIM2->CCMR1 |= (TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2);
	//enable the compare output channel
	TIM2->CCER |= TIM_CCER_CC2E;
	TIM2->CCER |= TIM_CCER_CC1E;
}

void Timer3_Config(){
	// Enable the TIM3 peripheral clock
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	// Configure TIM3 in PWM mode
	TIM3->PSC = 71;       // Prescaler set to 15
	TIM3->ARR = 0xFFFF;   // Auto-reload value (maximum value for 16-bit timer)
	TIM3->CCR2 = 10;      // Set the capture/compare value for channel 2
	TIM3->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;  // PWM mode 1
	TIM3->CCER |= TIM_CCER_CC2E;  // Enable capture/compare channel 2
	TIM3->CNT = 0;	//reset the counter
	// Start the timer
	TIM3->CR1 |= TIM_CR1_CEN;
}

void Timer4_Config(){
	//1. enable timer 4 clocks
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
	//2. set prescaler to bring clock to 1MHz
	TIM4->PSC = 71;
	//3. set ARR to maximum
	TIM4->ARR = 0xFFFF;
	//4. set the mode of the timer (input)
	TIM4->CCMR1 |= TIM_CCMR1_CC1S_0;
	//5. set capture to rising and falling edge + enable channel
	TIM4->CCER |= (TIM_CCER_CC1P | TIM_CCER_CC1NP);
	TIM4->CCER |= TIM_CCER_CC1E;
	//5. set and enable the interrupt
	TIM4->DIER |= TIM_DIER_CC1IE;	//UIE
	NVIC_EnableIRQ(TIM4_IRQn);
	//clear interrupt flag
	TIM4->SR = ~(TIM_SR_CC1IF);
	//6. set filter duration to 0. (no filters)
	TIM4->CCMR1 &= ~(TIM_CCMR1_IC1F);
	//7. set the input prescaler to capture all transitions (no prescaler)
	TIM4->CCMR1 &= ~(TIM_CCMR1_IC1PSC);
	//8. enable timer
	TIM4->CR1 |= TIM_CR1_CEN;
}


void Steering_Algorithm(double refined_speed)
{
	int steerHardleft = (GPIOB->IDR & GPIO_IDR_10) == 0;
	int steerMiddleLeft = (GPIOA->IDR & GPIO_IDR_8) == 0;
	int straight = (GPIOA->IDR & GPIO_IDR_10) == 0;
	int steerMiddleRight = (GPIOB->IDR & GPIO_IDR_3) == 0;
	int steerHardRight = (GPIOB->IDR & GPIO_IDR_4) == 0;
	double motor_speed;

	if(pid_state){
		motor_speed = refined_speed;
	}
	else{
		motor_speed = 60;
	}
	//left most pin
	if (steerHardleft && !steerMiddleLeft) {
	Right_Wheel_Forward_Rotation((motor_speed * 1.0));    	//100% power
	Left_Wheel_Forward_Rotation((motor_speed * 0.0));     	//0% power
	}
	//pin left of the center pin
	if (steerMiddleLeft && !steerHardleft) {
	Right_Wheel_Forward_Rotation((motor_speed * 0.583));	//58.3% power
	Left_Wheel_Forward_Rotation((motor_speed * 0.0));       //0% power
	}
	if(steerMiddleLeft && steerHardleft)
	{
	Right_Wheel_Forward_Rotation((motor_speed * 0.75));     //75% power
	Left_Wheel_Forward_Rotation((motor_speed * 0.0));       //0% power
	}
	//middle pin
	if (straight) {
	Right_Wheel_Forward_Rotation((motor_speed * 0.833));  	//83.3% power
	Left_Wheel_Forward_Rotation((motor_speed * 0.583));		//58.3% power
	}
	//pin right of the center pin
	if (steerMiddleRight && !steerHardRight) {
	Right_Wheel_Forward_Rotation((motor_speed * 0.017));	//1.7% power
	Left_Wheel_Forward_Rotation((motor_speed * 0.45));		//45% power
	}
	if (steerHardRight && !steerMiddleRight) {
	Right_Wheel_Forward_Rotation((motor_speed * 0.017));	//1.7% power
	Left_Wheel_Forward_Rotation((motor_speed * 0.85));		//85% power
	}
	if(steerHardRight && steerMiddleRight)
	{
	Right_Wheel_Forward_Rotation((motor_speed * 0.017)); 	//1.7% power
	Left_Wheel_Forward_Rotation((motor_speed * 0.9));		//90% power
	}
}

//------------------------------------------PID---------------------------------------------------
double PID_Controller(double current_distance){
	//error variable
	double error = current_distance - SETPOINT;
	static double previous_error = 0;
	static double integral = 0;
	double derivative = 0;

	if (error == 0 || error > SETPOINT){
		integral = 0;
	}

	//calculate integral and derivative
	integral = integral + error;
	derivative = error - previous_error;
	previous_error = error;

	//calculate output signal
	double output = (Kp * error) + (Ki * integral) + (Kd * derivative);	//improving beginners PID

	if (output >= MAX_OUTPUT){
		output = MAX_OUTPUT;
	}
	//output signal
	if(output < 0)
	{
		output = 0;
	}
	return output;
}

void PID_Controller_loop(){
	double current_distance = distance;

	//calculate and pass output_signal to actuator.
	double output_signal = PID_Controller(current_distance);
	double refined_speed = convert_output(output_signal);
	Steering_Algorithm(refined_speed);
}
//******************************************LOGIC*************************************************

//------------------------------------------Wheels------------------------------------------------
long map(long x, long in_min, long in_max, long out_min, long out_max){
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

double convert_output(double pid_output){
    double percentage = map(pid_output, 0, 200, 0, 80);
    return percentage;
}

void Right_Wheel_Forward_Rotation(double value) {
	//values from the clockwise rotation section in datasheet.
	value = map(value, 0, 100, 1470, 1280);
	TIM2->CCR1 = (value/2);
}

void Left_Wheel_Forward_Rotation(double value){
	//values from the counter clockwise rotation section in datasheet.
	value = map(value, 0, 100, 1520, 1730);
	TIM2->CCR2 = (value/2);
}

void Stop_Wheels(){
	TIM2->CCR2 = (MOTOR_STOP_PWM/2);
	TIM2->CCR1 = (MOTOR_STOP_PWM/2);
}

void TIM4_IRQHandler(void)
{
    if (TIM4->SR & TIM_SR_CC1IF) {
        TIM4->SR &= ~TIM_SR_CC1IF;
        if (capture_done == 0) {
            start_time = TIM4->CCR1;
            capture_done = 1;
        } else {
            end_time = TIM4->CCR1;
            // Speed of sound is approximately 343 meters per second or 0.0343 cm per microsecond
            uint32_t pulse_duration = end_time - start_time;
            distance = (pulse_duration / 58);
            capture_done = 0;
        }
    }
}

//POC servo driving with PID using datasheet values
void Servo_Controller(double PID_output){
	int speed_right = RIGHT_MOTOR_FORWARD - PID_output;
	int speed_left = LEFT_MOTOR_FORWARD + PID_output;
	int corrected_right = speed_right - 16;	//needed to maintain same straight speed as left motor.
	TIM2->CCR1 = (corrected_right/2);
	TIM2->CCR2 = (speed_left/2);
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
