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
#define ARM_MATH_CM7
#include "arm_math.h"
#include <math.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define buff_len 512
#define clock_speed 128000000


#if defined( __ICCARM__ )
  #define DMA_BUFFER \
      _Pragma("location=\".dma_buffer\"")
#else
  #define DMA_BUFFER \
      __attribute__((section(".dma_buffer")))
#endif

//DMA_BUFFER uint32_t buff_in[buff_len * 2];



#define tim2_arr_period 100
#define tim3_arr_period 10
#define tim2_psc 64
#define tim3_psc 10

#define sample_rate clock_speed / (tim3_arr_period * tim3_psc)
#define fft_res sample_rate / buff_len   //195.3 //390.625

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */


//uint32_t buff_in[(uint16_t)buff_len*2];
DMA_BUFFER int16_t buff_in[buff_len*2];
DMA_BUFFER uint16_t pwm_in[buff_len*2];
ALIGN_32BYTES (static float fft_in[(uint16_t)buff_len]);
ALIGN_32BYTES (static float pwm_fft_in[(uint32_t)buff_len]);
float fft_out[(uint32_t)buff_len*2];
float pwm_fft_out[(uint32_t)buff_len*2];
float fft_mag[(uint16_t)buff_len];

static arm_rfft_fast_instance_f32 fft_handle;
static arm_rfft_fast_instance_f32 fft_handle2;

volatile uint8_t buff_state = 0;
volatile uint8_t pwm_buff_state = 0;
volatile uint32_t fft_index = 0;
volatile uint8_t result_flag = 0;
//const uint32_t sample_rate = clock_speed / (tim3_arr_period * tim3_psc);
uint32_t test_freq = (clock_speed / (tim2_arr_period * tim2_psc));
//float fft_res =  sample_rate/ buff_len;

volatile float DC_magnitude;
volatile float DC_sum = 0;
volatile float test_phase = 0;
volatile float averaged_phase = 0;
volatile float test_phase2 = 0;
volatile float averaged_phase2 = 0;
volatile float test_phase3 = 0;
volatile float averaged_phase3 = 0;
volatile float test_phase4 = 0;
volatile float averaged_phase4 = 0;
volatile float test_magnitude = 0;
int phase_index = 20000 / fft_res;
volatile int average_index;


uint8_t echo_state = 0;
	uint8_t ultra_flag = 0;
	uint8_t ultra_count_flag = 0;
	uint8_t us_flag = 0;
uint16_t Distance = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM7_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void setup();

void load_FFT(uint32_t start, uint32_t stop);

void perform_FFT();

void uart_send();

uint16_t ultra_sonic();

uint16_t peak_finder();

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

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */
  //setup();  //checks with Spyder program if system ready
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC3_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_TIM8_Init();
  MX_TIM7_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_DMA(&hadc3, (uint32_t *) buff_in, buff_len);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *) pwm_in, buff_len);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_Base_Start(&htim4);
  HAL_TIM_Base_Start_IT(&htim8);

  HAL_GPIO_WritePin(trig_GPIO_Port, trig_Pin, 0);

  HAL_GPIO_WritePin(test_out_GPIO_Port, test_out_Pin, 0);

  TIM1 -> CCR1 = 50;
  TIM2 -> CCR1 = 50;
  TIM3 -> CCR1 = 5;

  arm_rfft_fast_init_f32(&fft_handle, buff_len);
  arm_rfft_fast_init_f32(&fft_handle2, buff_len);

  phase_index = test_freq * buff_len / sample_rate;
  //SCB_CleanDCache();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  if(buff_state == 1){    //for averaging (later on)
		  load_FFT(0, buff_len);
		  perform_FFT();

	  }
	  else if (buff_state == 2){
		  load_FFT(buff_len, buff_len*2);
		  //HAL_ADC_Stop_DMA(&hadc3);
		  perform_FFT();
	  }

	  if (DC_magnitude >= 3.2){
		  HAL_GPIO_WritePin(RGB_BLUE_GPIO_Port, RGB_BLUE_Pin, 1);
		  HAL_GPIO_WritePin(RGB_GREEN_GPIO_Port, RGB_GREEN_Pin, 0);
		  HAL_GPIO_WritePin(RGB_RED_GPIO_Port, RGB_RED_Pin, 0);

	  }
	  else if (DC_magnitude < 3.2 && DC_magnitude > 0.3){
		  HAL_GPIO_WritePin(RGB_BLUE_GPIO_Port, RGB_BLUE_Pin, 0);
		  HAL_GPIO_WritePin(RGB_GREEN_GPIO_Port, RGB_GREEN_Pin, 1);
		  HAL_GPIO_WritePin(RGB_RED_GPIO_Port, RGB_RED_Pin, 0);
	  }
	  else if(DC_magnitude >= 0.5){
		  HAL_GPIO_WritePin(RGB_BLUE_GPIO_Port, RGB_BLUE_Pin, 0);
		  HAL_GPIO_WritePin(RGB_GREEN_GPIO_Port, RGB_GREEN_Pin, 0);
		  HAL_GPIO_WritePin(RGB_RED_GPIO_Port, RGB_RED_Pin, 1);
	  }

/*

	  if (ultra_flag == 0){
		  ultra_flag = 1;
		  HAL_GPIO_WritePin(trig_GPIO_Port, trig_Pin, 1);  // pull the TRIG pin HIGH
		  __HAL_TIM_SET_COUNTER(&htim4, 0);
		  //while (__HAL_TIM_GET_COUNTER (&htim4) < 10);  // wait for 10 us
		  HAL_TIM_Base_Start_IT(&htim7);
		  while(!us_flag);
		  HAL_TIM_Base_Stop(&htim7);
		  us_flag = 0;

		  HAL_GPIO_WritePin(trig_GPIO_Port, trig_Pin, 0);  // pull the TRIG pin low
		  pMillis = HAL_GetTick(); // used this to avoid infinite while loop  (for timeout)
	  }

	  if(ultra_flag == 1 && HAL_GetTick() < (pMillis + 10)){
		  if (echo_state){
			  Value1 = __HAL_TIM_GET_COUNTER (&htim4);
			  pMillis = HAL_GetTick(); // used this to avoid infinite while loop (for timeout)
			  ultra_count_flag = 1;
		  }
	  }
	  else
		  ultra_flag = 0;

	  if(ultra_flag == 1 && ultra_count_flag == 1 && HAL_GetTick() < pMillis + 50){
	  		  if (!echo_state){
	  			  Value2 = __HAL_TIM_GET_COUNTER (&htim4);
	  			  Distance = (Value2-Value1)* 0.034/2;
	  			  ultra_flag = 0;
	  			  ultra_count_flag = 0;
	  		  }
	  }
	  else
		  ultra_flag = 0;

	  */


	      // wait for the echo pin to go high
	  /*while (!(HAL_GPIO_ReadPin (echo_GPIO_Port, echo_Pin)) && pMillis + 10 >  HAL_GetTick());
	  Value1 = __HAL_TIM_GET_COUNTER (&htim4);

	  pMillis = HAL_GetTick(); // used this to avoid infinite while loop (for timeout)
	      // wait for the echo pin to go low
	  while ((HAL_GPIO_ReadPin (echo_GPIO_Port, echo_Pin)) && pMillis + 50 > HAL_GetTick());
	      Value2 = __HAL_TIM_GET_COUNTER (&htim4); */


	     // printf("%lu cm, magnitude is %f, phase is %f \n", Distance, (float32_t)(magnitude/10000000), (float32_t) phase);


	      //printf("%lu \n", buff_in[0]);
	      //HAL_ADC_Start_DMA(&hadc3, (uint32_t *) buff_in, buff_len*2);



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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 32;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInitStruct.PLL2.PLL2M = 4;
  PeriphClkInitStruct.PLL2.PLL2N = 9;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 3072;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_14B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T3_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = ENABLE;
  hadc1.Init.Oversampling.Ratio = 8;
  hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_3;
  hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc1.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.Resolution = ADC_RESOLUTION_14B;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T3_TRGO;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc3.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  hadc3.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc3.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc3.Init.OversamplingMode = ENABLE;
  hadc3.Init.Oversampling.Ratio = 8;
  hadc3.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_3;
  hadc3.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc3.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_8CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

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
  htim1.Init.Prescaler = 63;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 99;
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
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
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
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 100;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 63;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
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
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 9;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 127;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
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
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 127;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 9;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 12799;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 9999;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

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
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 1, 0);
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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, RGB_BLUE_Pin|RGB_GREEN_Pin|RGB_RED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(trig_GPIO_Port, trig_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(test_out_GPIO_Port, test_out_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(UART_led_GPIO_Port, UART_led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RGB_BLUE_Pin RGB_GREEN_Pin RGB_RED_Pin */
  GPIO_InitStruct.Pin = RGB_BLUE_Pin|RGB_GREEN_Pin|RGB_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : trig_Pin */
  GPIO_InitStruct.Pin = trig_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(trig_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : echo_Pin */
  GPIO_InitStruct.Pin = echo_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(echo_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : test_out_Pin */
  GPIO_InitStruct.Pin = test_out_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(test_out_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : UART_led_Pin */
  GPIO_InitStruct.Pin = UART_led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(UART_led_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc){
	if(HAL_NVIC_GetActive(DMA1_Stream0_IRQn)==1){
		buff_state = 1;
		HAL_GPIO_WritePin(test_out_GPIO_Port, test_out_Pin, 1);
	}
	if(HAL_NVIC_GetActive(DMA2_Stream0_IRQn)==1){

			//HAL_GPIO_WritePin(trig_GPIO_Port, trig_Pin, 1);
		}

}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	/*for(int i = 0; i< 30; i++){
		printf("%lu \n", buff_in[i]);
	}*/
	if(HAL_NVIC_GetActive(DMA1_Stream0_IRQn)==1){
		buff_state = 2;
		HAL_GPIO_WritePin(test_out_GPIO_Port, test_out_Pin, 0);
	}
	if(HAL_NVIC_GetActive(DMA2_Stream0_IRQn)==1){

				//HAL_GPIO_WritePin(trig_GPIO_Port, trig_Pin, 0);
			}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	  /*if(GPIO_Pin == echo_Pin) {
		  if (!echo_state){
			  echo_state = 1;
		  }
		  else
			  echo_state = 0;
	}*/
}

void setup(){
	HAL_GPIO_WritePin(test_out_GPIO_Port, test_out_Pin, 0);
	HAL_GPIO_WritePin(RGB_BLUE_GPIO_Port, RGB_BLUE_Pin, 0);
	HAL_GPIO_WritePin(RGB_GREEN_GPIO_Port, RGB_GREEN_Pin, 0);
	HAL_GPIO_WritePin(RGB_RED_GPIO_Port, RGB_RED_Pin, 0);
	HAL_GPIO_WritePin(UART_led_GPIO_Port, UART_led_Pin, 0);
	char start_msg[] = "ready \n";
	//uint8_t rcv_msg[1];
	//uint16_t Timeout = 1000;
	HAL_UART_Transmit(&huart3, (uint8_t*) start_msg, strlen(start_msg), 1);
	/*HAL_UART_Receive(&huart3, rcv_msg, 1, Timeout);
	if (rcv_msg[0] == 0){
		char error_msg[] = "Not ready for data presentation! Pending...";
		HAL_UART_Transmit(&huart3, (uint8_t*) error_msg, strlen(error_msg), 1);
		while(rcv_msg[0] != 1){
			HAL_UART_Receive(&huart3, rcv_msg, 1, Timeout);
		}

	}

	else if(rcv_msg[0] != 1){
		char error_msg[] = "Error!";
		HAL_UART_Transmit(&huart3, (uint8_t*) error_msg, strlen(error_msg), 1);
		while(rcv_msg[0] != 1){
				HAL_UART_Receive(&huart3, rcv_msg, 1, Timeout);
		}

	}*/
	char confirm_msg[] = "ready to transmit data";
	HAL_UART_Transmit(&huart3, (uint8_t*) confirm_msg, strlen(confirm_msg), 1);
	HAL_GPIO_WritePin(UART_led_GPIO_Port, UART_led_Pin, 1);
}







void load_FFT(uint32_t start, uint32_t stop){
	fft_index = 0;
	for(int i = start; i < stop; i++){
		fft_in[fft_index] = (float) buff_in[i];
		pwm_fft_in[fft_index] = (float)pwm_in[i];
		fft_index++;
	}
	buff_state = 0;
	pwm_buff_state = 0;
}


void perform_FFT(){
	HAL_GPIO_WritePin(RGB_RED_GPIO_Port, RGB_RED_Pin, 1);
	arm_rfft_fast_f32(&fft_handle, pwm_fft_in, pwm_fft_out, 0);
	arm_rfft_fast_f32(&fft_handle, fft_in, fft_out, 0);
	//phase = atan2(fft_out[phase_index + 1], fft_out[phase_index]);
	//magnitude = sqrt(fft_out[phase_index + 1]*fft_out[phase_index + 1]+ fft_out[phase_index] * fft_out[phase_index]);
	//arm_cmplx_mag_f32(fft_out, fft_mag, buff_len/2);
	DC_sum += fft_out[0];
	//int peak = 29084 * 2/fft_res; //peak_finder()*2/fft_res;
	float RX_phase = (float) atan2(fft_out[17], fft_out[16]);    //20k should be 204
	float RX_phase2 = (float) atan2(fft_out[9], fft_out[8]);
	float RX_phase3 = (float) atan2(fft_out[33], fft_out[32]);    //20k should be 204
	//float RX_phase4 = (float) atan2(fft_out[peak+1], fft_out[peak]);
	//float pwm_phase = (float) atan2(pwm_fft_out[17], pwm_fft_out[16]);
	//float pwm_phase2 = (float) atan2(pwm_fft_out[3392*2/fft_res+1], pwm_fft_out[3392*2/fft_res]);  //282
	//float pwm_phase3 = (float) atan2(pwm_fft_out[103], pwm_fft_out[102]);
	//float pwm_phase4 = (float) atan2(pwm_fft_out[peak+1], pwm_fft_out[peak]);  //282
	//float sum_phase = (pwm_phase - RX_phase);
	//float sum_phase2 = (pwm_phase2 - RX_phase2);
	//float sum_phase3 = (pwm_phase3 - RX_phase3);
    //float sum_phase4 = (pwm_phase4 - RX_phase4);
	test_phase += RX_phase;
	test_phase2 += RX_phase2;
	test_phase3 += RX_phase3;
	//test_phase4 += sum_phase4;
   /* if (sum_phase < 0){
		sum_phase = -1 * sum_phase;
	}
    if (sum_phase != 0){
    		test_phase += sum_phase;
    }

    //-----
    if (pwm_phase2 < 0){
    		pwm_phase2 = -1 * pwm_phase2;
    	}
        if (pwm_phase2 != 0){
        		test_phase2 += pwm_phase2;
        }*/
    //------*/
    average_index++;
	HAL_GPIO_WritePin(RGB_RED_GPIO_Port, RGB_RED_Pin, 0);
	result_flag = 1;
}


void uart_send(){
	char msg0[33];
	char msg1[] = " V \t";
	char msg2[33];
	char msg3[] = " (20k) \t";
	char msg4[33];
	char msg5[] = " (10k) \t";
	char msg6[33];
	char msg7[] = " (40k) \n";


	int resolution = (int) fft_res;
    snprintf(msg0, 33, "%f", DC_magnitude);
    snprintf(msg2, 33, "%f", averaged_phase);
    snprintf(msg4, 33, "%f", averaged_phase2);
    snprintf(msg6, 33, "%f", averaged_phase3);

   HAL_UART_Transmit(&huart2, (uint8_t*) msg0, strlen(msg0), 1);
   HAL_UART_Transmit(&huart2, (uint8_t*) msg1, strlen(msg1), 1);
   HAL_UART_Transmit(&huart2, (uint8_t*) msg2, strlen(msg2), 1);
   HAL_UART_Transmit(&huart2, (uint8_t*) msg3, strlen(msg3), 1);
   HAL_UART_Transmit(&huart2, (uint8_t*) msg4, strlen(msg4), 1);
   HAL_UART_Transmit(&huart2, (uint8_t*) msg5, strlen(msg5), 1);
   HAL_UART_Transmit(&huart2, (uint8_t*) msg6, strlen(msg6), 1);
   HAL_UART_Transmit(&huart2, (uint8_t*) msg7, strlen(msg7), 1);
   /*HAL_UART_Transmit(&huart2, (uint8_t*) msg8, strlen(msg8), 1);
   HAL_UART_Transmit(&huart2, (uint8_t*) msg9, strlen(msg9), 1);
   HAL_UART_Transmit(&huart2, (uint8_t*) msg10, strlen(msg10), 1);
   HAL_UART_Transmit(&huart2, (uint8_t*) msg11, strlen(msg11), 1);*/
   HAL_GPIO_WritePin(UART_led_GPIO_Port, UART_led_Pin, 1);
}

uint16_t ultra_sonic(){
	uint32_t pMillis;
	uint32_t Value1 = 0;
	uint32_t Value2 = 0;


	HAL_GPIO_WritePin(trig_GPIO_Port, trig_Pin, GPIO_PIN_SET);
			  	  __HAL_TIM_SET_COUNTER(&htim1, 0);
			  	   while (__HAL_TIM_GET_COUNTER (&htim4) < 10);  // wait for 10 us
			  	   HAL_GPIO_WritePin(trig_GPIO_Port, trig_Pin, GPIO_PIN_RESET);

			  	   pMillis = HAL_GetTick();
			  	   while (!(HAL_GPIO_ReadPin (echo_GPIO_Port, echo_Pin)) && pMillis + 10 >  HAL_GetTick());
			  	   Value1 = __HAL_TIM_GET_COUNTER (&htim4);

			  	   pMillis = HAL_GetTick();
			  	   while ((HAL_GPIO_ReadPin (echo_GPIO_Port, echo_Pin)) && pMillis + 50 > HAL_GetTick());
			  	   Value2 = __HAL_TIM_GET_COUNTER (&htim4);

			  	   //Distance = (Value2 - Value1) * 0.034/2;
			  	   return ((Value2 - Value1) * 0.034/2);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if (htim == &htim8){
		averaged_phase = test_phase/average_index;
		int upper_lim = 8100000;
		int lower_lim = 2300000;
		DC_magnitude = DC_sum/average_index;
		DC_magnitude = 3.3 * (DC_sum/average_index - lower_lim)/(upper_lim - lower_lim);
		averaged_phase2 = test_phase2/average_index;
		averaged_phase3 = test_phase3/average_index;
		//averaged_phase4 = test_phase4/average_index;
		test_phase = 0;
		test_phase2 = 0;
		test_phase3 = 0;
		//test_phase4 = 0;
		DC_sum = 0;
		average_index = 0;
		/*if (result_flag){
			uart_send();
		    result_flag = 0;
		}*/
		Distance = 0;//ultra_sonic();*/
		uart_send();
	}
	/*if (htim == &htim7){
		us_flag = 1;
	}*/
}

uint16_t peak_finder(){
	float peak_freq = fft_out[2];
	int index;
	for(int i = 2; i < buff_len*2; i = (i + 2)){
		if (sqrt(fft_out[i] * fft_out[i] + fft_out[i+1] * fft_out[i+1]) > peak_freq){
			index = i;
			peak_freq = fft_out[i];
		}
	}
	return index/2*fft_res;
}

int _write(int file, char *ptr, int len)
{
  (void)file;
  int DataIdx;

  for (DataIdx = 0; DataIdx < len; DataIdx++)
  {
    ITM_SendChar(*ptr++);
  }
  return len;
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
