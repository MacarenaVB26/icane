/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <PID_Class.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PPR (float)8192
#define eps_0 0
#define PI 3.14159
#define NMOTORS 3
#define SQRT_3 sqrt(3.0)
#define r 0.05 // wheel radius
#define L 0.195 // Distance center-wheel
#define TAO 0.6538
#define sampleTime 0.010
#define RX_BUFFER_SIZE 64

extern UART_HandleTypeDef huart6;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
// Control PID parameters (CONTINUOUS)

float Kp1 = 0.3, Ki1 = 96.9838, Kd1 = 0.02;
float Kp2 = 0.3, Ki2 = 98.7907, Kd2 = 0.01;
float Kp3 = 0.2, Ki3 = 100.0, Kd3 = 0.0; // Velocity

// 40 40 -- 1.7
// 30 30 -- 1.4
// 30 20 -- 1.45
// 20 40 -- 0.9
// 25 50 -- 1.2  ----
// 30 50 -- 1.4

//Ultima prueba
//float Kp1 = 20, Ki1 = 30, Kd1 = 0.0; // 30 50 o 20 50
//float Kp2 = 20, Ki2 = 40, Kd2 = 0.0;
//float Kp3 = 10, Ki3 = 30, Kd3 = 0.0; // Torque
/*
float Kp1 = 53.8, Ki1 = 2.12, Kd1 = 0.03; // 30 50 o 20 50
float Kp2 = 55.5, Ki2 = 2.2, Kd2 = 0.03;
float Kp3 = 54, Ki3 = 2, Kd3 = 0.03;
*/
//pid[1] // Motor 3 - kp2
//pid[2] // Motor 1 - kp3
//pid[0] // Motor 2 - kp1

// Control PID parameters (DISCRETE)
/*
float Kp1 = 1.5719, Ki1 = 161.226, Kd1 = 0.0038314;
float Kp2 = 2.6585, Ki2 = 192.7245, Kd2 = 0.0091679;
float Kp3 = 2.0, Ki3 = 189.0592, Kd3 = 0.0052892; //Velocity
*/
// Platform pose
float x = 0.0, y = 0.0, phi_c = 0.0;
float xPrev = 0, yPrev = 0, phi_cPrev = 0;

//Kinematics
float C_eps = cos(eps_0);
float S_eps = sin(eps_0);
const float CA_eps_pi6 = cos(eps_0 + PI/6);
const float CM_eps_pi3 = cos(eps_0 - PI/3);
const float CA_eps_pi3 = cos(eps_0 + PI/3);
const float SA_eps_pi6 = sin(eps_0 + PI/6);
const float SM_eps_pi3 = sin(eps_0 - PI/3);
const float SA_eps_pi3 = sin(eps_0 + PI/3);
volatile float ct = 1; //cos(phi_c);
volatile float st = 0; //sin(phi_c);
float w_angle = PI/3;
float J0[2][3];
float JI[3][2];
float vx = 0, vy = 0, vw = 0;

//Current measurements
float prev_curr[3][5]={{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0}};
float filter_curr[3][2]={{0,0},{0,0},{0,0}};
volatile float CURRENT[3] = {0.0,0.0,0.0};
volatile float VOUT[3] = {0.0,0.0,0.0};
volatile float torque[3] = {0.0,0.0,0.0};
uint32_t adc_val[3] = {0,0,0};

//Encoder measurement
volatile  int32_t current_capture[3] = {0,0,0};
volatile  int32_t current_captureA[3] = {0,0,0};
volatile float enc_count_0[3] = {0.0,0.0,0.0};
volatile float enc_count_1[3] = {0.0,0.0,0.0};
volatile float vel_rpm[3] = {0.0,0.0,0.0};
float prev_vel_rpm[3] = {0.0,0.0,0.0};
float w_rad[3] = {0.0,0.0,0.0};

// Go to goal control
float xGoal;
float yGoal;
float phi_cGoal_1 = 0.0;
float vx_sp = 0, vy_sp = 0, vw_sp = 0; //Setpoints
float eint_GGX = 0.0;
float eint_GGY = 0.0;
volatile float errorX;
volatile float errorY;
uint8_t ready;
uint16_t control =0;
float eint = 0.0;
float e = 0.0;
//Targets for different controllers
float torque_sp[3]= {0.00,0.00,0.00};
/*
 * +0.4 a -0.5 N.m sin carga
 * */
float vt[3] = {0.0,0.0,0.0};

float vt_UART[3] = {0.0,0.0,0.0};

//Serial printing
uint8_t rx_byte;
uint8_t rx_ready=0;
char rx_buffer[RX_BUFFER_SIZE];
uint16_t rx_index = 0;

int len;
char tx_buffer[64];

//uint8_t rxBuffer[25];

// Kalman
volatile float state[3] = {0.0,0.0,0.0}; // Kalman estimated state
float P = 1.0;     // Initial estimation error
float K = 0.0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM5_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void CalculatePositionError(float xGoal_l, float yGoal_l, float *vx_set, float *vy_set, float *vw_set, volatile float deltaT);
void IKinematics(float vx_set, float vy_set, float vw_set);
void DKinematics(float w_rad[3]);
float kalmanFilter(float dt, float u_n, float z_n, float state);
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
  HAL_InitTick(TICK_INT_PRIORITY);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM5_Init();
  MX_USART6_UART_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  //Initializing DMA for UART6
  //HAL_UART_Receive_DMA(&huart6, rxBuffer, 56);
  HAL_UART_Receive_IT(&huart6, &rx_byte, 1);
  //Initializing timers for encoder and PWM
  HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_3);
  HAL_TIM_Encoder_Start_IT(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start_IT(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start_IT(&htim5, TIM_CHANNEL_ALL);

  HAL_NVIC_SetPriority(USART6_IRQn,0,0);
  HAL_NVIC_EnableIRQ(USART6_IRQn);

  HAL_NVIC_SetPriority(TIM4_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(TIM4_IRQn);

  HAL_NVIC_SetPriority(TIM5_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(TIM5_IRQn);

  HAL_NVIC_SetPriority(TIM3_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);

  HAL_NVIC_SetPriority(TIM1_CC_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);

  __enable_irq();

  //Defining PID controller for each motor
  SimplePID pid[NMOTORS];
  SimplePID_Init(&pid[0]);
  SimplePID_Init(&pid[1]);
  SimplePID_Init(&pid[2]);
  SimplePID_SetParams(&pid[0], Kp1, Ki1, Kd1, 1000.0, 10.0);
  SimplePID_SetParams(&pid[1], Kp2, Ki2, Kd2, 1000.0, 10.0);
  SimplePID_SetParams(&pid[2], Kp3, Ki3, Kd3, 1000.0, 10.0);


  uint32_t start_time = HAL_GetTick();
  volatile float deltaT = 0.0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  deltaT = (HAL_GetTick() - start_time)/1000.0;

	  if(deltaT >= sampleTime){
		  start_time = HAL_GetTick(); // Obtén el tiempo actual

		  for(int k = 0; k < NMOTORS; k++){

			  if (k==1){//0
				  current_capture[k] = TIM5->CNT;
			  } else if (k==0){//1
				  current_capture[k] = TIM3->CNT;
			  } else{
				  current_capture[k] = TIM4->CNT;
			  }

			  if (current_capture[k]>=4096){
				  current_captureA[k] = current_capture[k]-8192;
			  } else {
				  current_captureA[k] = current_capture[k];
			  }

			  enc_count_0[k] = current_captureA[k]*1.0;
			  if (enc_count_1[k]<0 && enc_count_0[k]>=0){

				  vel_rpm[k] = -((enc_count_0[k] - enc_count_1[k] - PPR*1.0) * 60.0) / (PPR*deltaT);

			  } else if (enc_count_1[k]>0 && enc_count_0[k]<0){
				  vel_rpm[k] = -((enc_count_0[k] - enc_count_1[k] + PPR*1.0) * 60.0) / (PPR*deltaT);
			  } else{
				  vel_rpm[k] = -((enc_count_0[k] - enc_count_1[k]) * 60.0) / (PPR*deltaT);
			  }
			  if (fabs(vel_rpm[k]) > 150.0){
				  vel_rpm[k] = prev_vel_rpm[k];
			  }
			  w_rad[k] = vel_rpm[k] * PI / 30; //rad/s
			  enc_count_1[k]=enc_count_0[k];
			  prev_vel_rpm[k] = vel_rpm[k];

		  }

		  for (int i = 0; i < 3; i++) {
			  HAL_ADC_Start(&hadc1);
			  HAL_ADC_PollForConversion(&hadc1,HAL_MAX_DELAY);
			  adc_val[i] =  HAL_ADC_GetValue(&hadc1);
			  VOUT[i] = (adc_val[i] * 3.30) / (1024.0); // Variar la división según la cantidad de bits configurada (2^12)
			  CURRENT[i] = 36.7*VOUT[i]/3.30 - 18.35;
			  torque[i] = kalmanFilter(deltaT, 0, CURRENT[i]*TAO, torque[i]);
			  HAL_ADC_Stop(&hadc1);
		  }

		  DKinematics(w_rad);

		  x = x + vx*deltaT;
		  y = y + vy*deltaT;
		  phi_c = phi_c + vw*deltaT;
//
		  for (int k=0;k<NMOTORS;k++){

			  SimpleCPID_Evaluate(&pid[k], w_rad[k], vt[k], deltaT);


			  if (pid[k].sgn>0) {
				  if (k==0){ //motor 1
					  HAL_GPIO_WritePin(INA1_GPIO_Port, INA1_Pin, GPIO_PIN_SET);
					  HAL_GPIO_WritePin(GPIOB, INB1_Pin, GPIO_PIN_RESET);
					  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pid[k].control);

				  } else if (k==1){// motor 2
					  HAL_GPIO_WritePin(GPIOC, INA2_Pin, GPIO_PIN_SET);
					  HAL_GPIO_WritePin(GPIOB, INB2_Pin, GPIO_PIN_RESET);
					  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pid[k].control);

				  } else{ //motor 3
					  HAL_GPIO_WritePin(GPIOB, INA3_Pin, GPIO_PIN_SET);
					  HAL_GPIO_WritePin(GPIOB, INB3_Pin, GPIO_PIN_RESET);
					  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pid[k].control);

				  }
			  } else{
				  if (k==0){
					  HAL_GPIO_WritePin(INA1_GPIO_Port, INA1_Pin, GPIO_PIN_RESET);
					  HAL_GPIO_WritePin(GPIOB, INB1_Pin, GPIO_PIN_SET);
					  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pid[k].control);

				  } else if (k==1){
					  HAL_GPIO_WritePin(GPIOC, INA2_Pin, GPIO_PIN_RESET);
					  HAL_GPIO_WritePin(GPIOB, INB2_Pin, GPIO_PIN_SET);
					  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pid[k].control);

				  } else{
					  HAL_GPIO_WritePin(GPIOB, INA3_Pin, GPIO_PIN_RESET);
					  HAL_GPIO_WritePin(GPIOB, INB3_Pin, GPIO_PIN_SET);
					  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pid[k].control);
					  control = pid[k].control;
					  eint = pid[k].e_int;
					  e = pid[k].eprev1;

				  }
			  }
		  }

//
/*
// poner /*
		  CalculatePositionError(xGoal, yGoal, &vx_sp, &vy_sp, &vw_sp, deltaT);
		  IKinematics(vx_sp,vy_sp,vw_sp);

		  if(fabs(xGoal - x) < 0.03 && fabs(yGoal - y) < 0.03){

			  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
			  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
			  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
			  ready = 1;
			  //pid[0].
		  }
		  else{
			  ready = 0;
			  for (int k=0;k<NMOTORS;k++){

				  SimpleCPID_Evaluate(&pid[k], w_rad[k], vt[k], deltaT);

				  if (pid[k].sgn>0) {
					  if (k==0){ //motor 1
						  HAL_GPIO_WritePin(INA1_GPIO_Port, INA1_Pin, GPIO_PIN_SET);
						  HAL_GPIO_WritePin(GPIOB, INB1_Pin, GPIO_PIN_RESET);
						  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pid[k].control);

					  } else if (k==1){// motor 2
						  HAL_GPIO_WritePin(GPIOC, INA2_Pin, GPIO_PIN_SET);
						  HAL_GPIO_WritePin(GPIOB, INB2_Pin, GPIO_PIN_RESET);
						  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pid[k].control);

					  } else{ //motor 3
						  HAL_GPIO_WritePin(GPIOB, INA3_Pin, GPIO_PIN_SET);
						  HAL_GPIO_WritePin(GPIOB, INB3_Pin, GPIO_PIN_RESET);
						  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pid[k].control);

					  }
				  } else{
					  if (k==0){
						  HAL_GPIO_WritePin(INA1_GPIO_Port, INA1_Pin, GPIO_PIN_RESET);
						  HAL_GPIO_WritePin(GPIOB, INB1_Pin, GPIO_PIN_SET);
						  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pid[k].control);

					  } else if (k==1){
						  HAL_GPIO_WritePin(GPIOC, INA2_Pin, GPIO_PIN_RESET);
						  HAL_GPIO_WritePin(GPIOB, INB2_Pin, GPIO_PIN_SET);
						  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pid[k].control);

					  } else{
						  HAL_GPIO_WritePin(GPIOB, INA3_Pin, GPIO_PIN_RESET);
						  HAL_GPIO_WritePin(GPIOB, INB3_Pin, GPIO_PIN_SET);
						  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pid[k].control);

					  }
				  }
			  }
		  }

//  poner */

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hadc1.Init.Resolution = ADC_RESOLUTION_10B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 3;
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 8192;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 8192;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 8192;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(INA1_GPIO_Port, INA1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, INB2_Pin|INB3_Pin|INB1_Pin|INA3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(INA2_GPIO_Port, INA2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : INA1_Pin */
  GPIO_InitStruct.Pin = INA1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(INA1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : INB2_Pin INB3_Pin INB1_Pin INA3_Pin */
  GPIO_InitStruct.Pin = INB2_Pin|INB3_Pin|INB1_Pin|INA3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : INA2_Pin */
  GPIO_InitStruct.Pin = INA2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(INA2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void DKinematics(float w_rad[3]) {
  /*
  Function that computes the velocity in rpm and the direction
  of each wheel from the absolute velocity.

  Inputs:
    - vel: Linear velocity of each motor in rad/s.
  */
	ct = cos(phi_c);
	st = sin(phi_c);
  // Angular velocity of each motor in rad/s
	J0[0][0] = (-2.0/3.0)*(S_eps       * ct + st * C_eps);
	J0[0][1] = (-2.0/3.0)*(CA_eps_pi6  * ct - st * SA_eps_pi6);
	J0[0][2] =  (2.0/3.0)*(SA_eps_pi3  * ct + st * CA_eps_pi3);
	J0[1][0] =  (2.0/3.0)*(C_eps       * ct - st * S_eps);
	J0[1][1] = (-2.0/3.0)*(CM_eps_pi3  * ct - st * SM_eps_pi3);
	J0[1][2] = (-2.0/3.0)*(CA_eps_pi3  * ct - st * SA_eps_pi3);

	vx = r*(-w_rad[0] *J0[0][0] + w_rad[1]*J0[0][1] + w_rad[2]*J0[0][2]);
	vy = r*(w_rad[0]  *J0[1][0] + w_rad[1]*J0[1][1] + w_rad[2]*J0[1][2]);
	vw = r*(w_rad[0] + w_rad[1] + w_rad[2])/(3.0*L);
}

void CalculatePositionError(float xGoal_l, float yGoal_l, float *vx_set, float *vy_set, float *vw_set, volatile float dt) {
    // Pose error
    float errorX = xGoal_l - x;
    float errorY = yGoal_l - y;
    if (ready == 1){
    	errorX = 0.0;
    	errorY = 0.0;
    	ready = 0;
    } else {
    	errorX = xGoal_l - x;
    	errorY = yGoal_l - y;
    }

    // PID Gains

    float kp = 0.5;
    float ki = 0.0;

    float propX = errorX*kp;
    float propY = errorY*kp;

    // Integral
	eint_GGX += errorX*dt;
	float integralX = eint_GGX*ki;

	eint_GGY += errorY*dt;
	float integralY = eint_GGY*ki;

    // Control signal
    double vx_global = propX + integralX;
    double vy_global = propY + integralY;

    *vw_set = 0;

    // Calcular la velocidad relativa para el robot vx y vy
    *vx_set = ct*vx_global+st*vy_global;
    *vy_set = -st*vx_global+ct*vy_global;

}

void IKinematics(float vx_set, float vy_set, float vw_set) {
  /*
  Function that computes the velocity in rpm and the direction
  of each wheel from the absolute velocity.

  Inputs:
    - vx: Linear velocity in X axis, in m/s.
    - vy: Linear velocity in Y axis, in m/s.
    - vw: Angular velocity in Z axis, in rad/s.
  */
	float w[] = {0.0, 0.0, 0.0};
  // Angular velocity of each motor in rad/s
	JI[0][0] = 3.0*J0[0][0]/2.0;
    JI[0][1] = 3.0*J0[1][0]/2.0;
    JI[1][0] = 3.0*J0[0][1]/2.0;
    JI[1][1] = 3.0*J0[1][1]/2.0;
    JI[2][0] = 3.0*J0[0][2]/2.0;
    JI[2][1] = 3.0*J0[1][2]/2.0;

    w[0] = (JI[0][0]*vx_set + JI[0][1]*vy_set + L*vw_set)/r;
    w[1] = (JI[1][0]*vx_set + JI[1][1]*vy_set + L*vw_set)/r;
    w[2] = (JI[2][0]*vx_set + JI[1][1]*vy_set + L*vw_set)/r;

    for (int i = 0; i < NMOTORS; i++) {
    // Calculate desired angular velocity in rpm
    	vt[i] = w[i];//*30/PI;
    }
}

float kalmanFilter(float dt, float u_n, float z_n, float state) {
  float Q = 0.00002;
  float R = 0.0019;

  float state1 = state + u_n * dt;
  P     = 3.3124e-04;
  P     = P + Q;

  K     = P / (P + R);
  state1 = state1 + K * (z_n - state1);
  //(1 - K) * P;

  return state1;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{


	if (huart->Instance == USART6) {
	    /*if (rx_byte == '\n') {
	        // Fin de frame (detectado por \n)
	        rx_buffer[rx_index] = '\0';

	        // Parsear los dos números separados por coma


	        if (sscanf(rx_buffer, "%f,%f", &xGoal, &yGoal) == 2) {
	            // Datos válidos recibidos

	            // Preparar respuesta
	            len = snprintf(tx_buffer, sizeof(tx_buffer),
	                         "%.3f,%.3f,%.3f\r\n",
	                         x, y, phi_c);

	            // Enviar por UART (bloqueante)
	            HAL_UART_Transmit(&huart6, (uint8_t*)tx_buffer, len, HAL_MAX_DELAY);
	        }

	        // Reiniciar para siguiente frame
	        rx_index = 0;
	        memset(rx_buffer, 0, sizeof(rx_buffer));
	    }
	    else if (rx_byte != '\r') {
	        // Guardar datos (ignorar \r, solo usar \n como terminador)
	        if (rx_index < RX_BUFFER_SIZE - 1) {
	            rx_buffer[rx_index++] = rx_byte;
	        }
	    }*/

		if (rx_byte != '\n' && rx_index < RX_BUFFER_SIZE - 1) {
			rx_buffer[rx_index++] = rx_byte;
		} else {
			rx_buffer[rx_index] = '\0'; // Terminar string
			rx_index = 0;

			// ---- Procesar mensaje completo ----

			sscanf(rx_buffer, "%f,%f", &xGoal, &yGoal);
			//sscanf(rx_buffer, "%f,%f,%f", &vt[0], &vt[1], &vt[2]);
			// ---- Armar mensaje de respuesta ----
			char tx_buffer[150];
			int len = snprintf(tx_buffer, sizeof(tx_buffer),
							   //"%.2f, %.2f, %.2f, %.2f, %.2f, %.2f\r\n",
							   "[x:%.2f, y:%.2f]|[%.2f, %.2f, %.2f],[%.2f, %.2f, %.2f],[%.2f, %.2f, %.2f]\r\n",
							   //vt[0], vt[1], vt[2], x, y, phi_c);
							   x, y, vt[0], w_rad[0], torque[0], vt[1], w_rad[1], torque[1], vt[2], w_rad[2], torque[2]);
			// ---- Enviar por UART (bloqueante) ----
			HAL_UART_Transmit(&huart6, (uint8_t*)tx_buffer, len, HAL_MAX_DELAY);
			/*len = snprintf(tx_buffer, sizeof(tx_buffer),
							   //"%.2f, %.2f, %.2f, %.2f, %.2f, %.2f\r\n",
							   "%.2f, %.2f, %.2f\r\n",
							   //vt[0], vt[1], vt[2], x, y, phi_c);
							   w_rad[0], w_rad[1], w_rad[2]);
			// ---- Enviar por UART (bloqueante) ----
			HAL_UART_Transmit(&huart6, (uint8_t*)tx_buffer, len, HAL_MAX_DELAY);
			len = snprintf(tx_buffer, sizeof(tx_buffer),
							   //"%.2f, %.2f, %.2f, %.2f, %.2f, %.2f\r\n",
							   "%.2f, %.2f, %.2f\r\n",
							   //vt[0], vt[1], vt[2], x, y, phi_c);
							   vt[0]-w_rad[0], vt[1]-w_rad[1], vt[2]-w_rad[2]);
			// ---- Enviar por UART (bloqueante) ----
			HAL_UART_Transmit(&huart6, (uint8_t*)tx_buffer, len, HAL_MAX_DELAY);
*/

			// Aquí puedes usar los valores, ejemplo:
			// printf("V1=%.3f, V2=%.3f, V3=%.3f\n", valor1, valor2, valor3);
		}


	    // Reactivar recepción
	    HAL_UART_Receive_IT(&huart6, &rx_byte, 1);
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
