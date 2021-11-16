/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define ARM_MATH_CM4
#include "arm_math.h"
#include "stm32l475e_iot01_qspi.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac_ch1;

QSPI_HandleTypeDef hqspi;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC1_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//The timer frequency is 44.1 kHz,
#define pi 3.14159265
	uint8_t play[22050] = {0};

	uint8_t counter = 0;

//Tone 1 C6 1046.5 Hz
	uint8_t C6[42];
void get_C6(){
	for(int i = 0; i < 42; i++){
		C6[i] =  0.33*(1 + arm_sin_f32(2*pi*i/42))*256;
	}
}

//Tone 2 E6 1318.5 Hz
	uint8_t E6[34];
void get_E6(){
	for(int i = 0; i < 34; i++){
		E6[i] =  0.33*(1 + arm_sin_f32(2*pi*i/34))*256;
	}
}

//Tone 3 G6 1568.0 Hz
	uint8_t G6[28];
void get_G6(){
	for(int i = 0; i < 28; i++){
		G6[i] =  0.33*(1 + arm_sin_f32(2*pi*i/28))*256;
	}
}

//Tone 4 A6 1760.0 Hz

	uint8_t A6[25];
void get_A6(){
	for(int i = 0; i < 25; i++){
		A6[i] =  0.33*(1 + arm_sin_f32(2*pi*i/25))*256;
	}
}

//Tone 5 B6 1975.53 Hz
	uint8_t B6[22];
void get_B6(){
	for(int i = 0; i < 22; i++){
		B6[i] =  0.33*(1 + arm_sin_f32(2*pi*i/22))*256;
	}
}

//Tone 6 B5 987.78 Hz
	uint8_t B5[45];
void get_B5(){
	for(int i = 0; i < 45; i++){
		B5[i] =  0.33*(1 + arm_sin_f32(2*pi*i/45))*256;
	}
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
  MX_DMA_Init();
  MX_DAC1_Init();
  MX_QUADSPI_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  BSP_QSPI_Init();
  HAL_TIM_Base_Start_IT(&htim2);




  get_B5();
  get_C6();
  get_E6();
  get_G6();
  get_A6();
  get_B6();
  uint32_t addr = 0x000000;
  for(int i = 0; i < 3; i++){
	  if(BSP_QSPI_Erase_Block(addr + i * 0x010000) != QSPI_OK){
		  Error_Handler();
	  }
  }

  uint32_t tone_addr = 0x000000;
  for(int i = 0; i < 490; i++){
	  if(BSP_QSPI_Write((uint8_t *)B5, tone_addr, 45) != QSPI_OK){
		  Error_Handler();
	  }
	  tone_addr += 45;
  }
  for(int i = 0; i < 525; i++){
	  if(BSP_QSPI_Write((uint8_t *)C6, tone_addr, 42) != QSPI_OK){
		  Error_Handler();
	  }
	  tone_addr += 42;
  }
  for(int i = 0; i < 648; i++){
	  if(BSP_QSPI_Write((uint8_t *)E6, tone_addr, 34) != QSPI_OK){
		  Error_Handler();
	  }
	  tone_addr += 34;
  }
  tone_addr = 0x010266;
  for(int i = 0; i < 787; i++){
	  if(BSP_QSPI_Write((uint8_t *)G6, tone_addr, 28) != QSPI_OK){
		  Error_Handler();
	  }
	  tone_addr += 28;
  }
  tone_addr = 0x015888;
  for(int i = 0; i < 882; i++){
	  if(BSP_QSPI_Write((uint8_t *)A6, tone_addr, 25) != QSPI_OK){
		  Error_Handler();
	  }
	  tone_addr += 25;
  }
  for(int i = 0; i < 1002; i++){
	  if(BSP_QSPI_Write((uint8_t *)B6, tone_addr, 22) != QSPI_OK){
		  Error_Handler();
	  }
	  tone_addr += 22;
  }

  //Read the data
  if(BSP_QSPI_Read((uint8_t *)play, 0x00000000, 22050) != QSPI_OK){
	  Error_Handler();
  }

  HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, play, 22050, DAC_ALIGN_8B_R);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  int GPIO_PinState = 0;//When you press the button, status is 0
	  GPIO_PinState =  HAL_GPIO_ReadPin (Button_GPIO_Port, Button_Pin);
	  if(GPIO_PinState==1){
		  //HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
	  }
	  else
	  {
		  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
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
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */
  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief QUADSPI Initialization Function
  * @param None
  * @retval None
  */
static void MX_QUADSPI_Init(void)
{

  /* USER CODE BEGIN QUADSPI_Init 0 */

  /* USER CODE END QUADSPI_Init 0 */

  /* USER CODE BEGIN QUADSPI_Init 1 */

  /* USER CODE END QUADSPI_Init 1 */
  /* QUADSPI parameter configuration*/
  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = 255;
  hqspi.Init.FifoThreshold = 1;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_NONE;
  hqspi.Init.FlashSize = 1;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */

  /* USER CODE END QUADSPI_Init 2 */

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
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1814;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Button_Pin */
  GPIO_InitStruct.Pin = Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
int lower = 1;
int upper = 3;
void HAL_DAC_ConvCpltCallbackCh1 (DAC_HandleTypeDef * hdac){


	int randomnumber = (rand() % (upper - lower + 1)) + lower;
	if(randomnumber%3 == 0){
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	}


	if(counter == 0){
		if(BSP_QSPI_Read((uint8_t *)play, 0x000000, 22050) != QSPI_OK){
			  Error_Handler();
		  }
		HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, play, 22050, DAC_ALIGN_8B_R);
	}
	if(counter == 1){
		if(BSP_QSPI_Read((uint8_t *)play, 0x005622, 22050) != QSPI_OK){
			  Error_Handler();
		  }
		HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, play, 22050, DAC_ALIGN_8B_R);
	}
	if(counter == 2){
		if(BSP_QSPI_Read((uint8_t *)play, 0x00AC44, 22050) != QSPI_OK){
			  Error_Handler();
		  }
		HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, play, 22050, DAC_ALIGN_8B_R);
	}
	if(counter == 3){
		if(BSP_QSPI_Read((uint8_t *)play, 0x010266, 22050) != QSPI_OK){
			  Error_Handler();
		  }
		HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, play, 22050, DAC_ALIGN_8B_R);
	}
	if(counter == 4){
		if(BSP_QSPI_Read((uint8_t *)play, 0x015888, 22050) != QSPI_OK){
			  Error_Handler();
		  }
		HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, play, 22050, DAC_ALIGN_8B_R);
	}
	if(counter == 5){
		if(BSP_QSPI_Read((uint8_t *)play, 0x01AEAA, 22050) != QSPI_OK){
			  Error_Handler();
		  }
		HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, play, 22050, DAC_ALIGN_8B_R);
		counter=0;
	}
	counter++;
}



/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM17 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM17) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
	__BKPT();

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
