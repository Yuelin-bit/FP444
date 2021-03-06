/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cmsis_os.h"
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

UART_HandleTypeDef huart1;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
osThreadId TransmitTaskHandle;
osThreadId SystemControllerHandle;
void button_pressed_task(void);
void control(void);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC1_Init(void);
static void MX_QUADSPI_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define pi 3.14159265
uint8_t play[22050] = {0};
uint8_t empty[22050] = {0};
int LED_0_status = 0;
int LED_1_status = 0;
int LED_2_status = 0;
int LED_3_status = 0;
int LED_4_status = 0;
//int DAC_status = 0;
int isDelaying = 1;//Control DAC
int isStop = 1;
int isPause = 0;
int score = 0;
int time_counter = 0;
int difficulty = 6;


char buffer[100] = {0};
char tBuff0[20];
char tBuff1[20];

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


void printWelcome(){

	 sprintf(buffer, "\033[9A");
	 HAL_UART_Transmit(&huart1, (uint8_t *) buffer, (uint16_t) strlen(buffer), 30000);
	 memset(buffer, 0, strlen(buffer));

	  sprintf(buffer, "         Welcome to our game! \r \n");
	  HAL_UART_Transmit(&huart1, (uint8_t *) buffer, (uint16_t) strlen(buffer), 30000);
	  memset(buffer, 0, strlen(buffer));

	  sprintf(buffer, "------------------------------------ \r \n");
	  HAL_UART_Transmit(&huart1, (uint8_t *) buffer, (uint16_t) strlen(buffer), 30000);
	  memset(buffer, 0, strlen(buffer));

	  sprintf(buffer, "|                                  | \r \n");
	  HAL_UART_Transmit(&huart1, (uint8_t *) buffer, (uint16_t) strlen(buffer), 30000);
	  memset(buffer, 0, strlen(buffer));

	  sprintf(buffer, "|       Press blue button          | \r \n");
	  HAL_UART_Transmit(&huart1, (uint8_t *) buffer, (uint16_t) strlen(buffer), 30000);
	  memset(buffer, 0, strlen(buffer));

	  sprintf(buffer, "|        to get started!           | \r \n");
	  HAL_UART_Transmit(&huart1, (uint8_t *) buffer, (uint16_t) strlen(buffer), 30000);
	  memset(buffer, 0, strlen(buffer));

	  sprintf(buffer, "|                                  | \r \n");
	  HAL_UART_Transmit(&huart1, (uint8_t *) buffer, (uint16_t) strlen(buffer), 30000);
	  memset(buffer, 0, strlen(buffer));

	  sprintf(buffer, "|      Difficulty is %d 	    | \r \n",difficulty);
	  HAL_UART_Transmit(&huart1, (uint8_t *) buffer, (uint16_t) strlen(buffer), 30000);
	  memset(buffer, 0, strlen(buffer));

	  sprintf(buffer, "------------------------------------ \r \n");
	  HAL_UART_Transmit(&huart1, (uint8_t *) buffer, (uint16_t) strlen(buffer), 30000);
	  memset(buffer, 0, strlen(buffer));

		 sprintf(buffer, "\033[9A");
		 HAL_UART_Transmit(&huart1, (uint8_t *) buffer, (uint16_t) strlen(buffer), 30000);
		 memset(buffer, 0, strlen(buffer));
}

void printFailed(){

	 sprintf(buffer, "\033[2J\033[9A");
	 HAL_UART_Transmit(&huart1, (uint8_t *) buffer, (uint16_t) strlen(buffer), 30000);
	 memset(buffer, 0, strlen(buffer));

	  sprintf(buffer, "           You Failed !!!            \r \n");
	  HAL_UART_Transmit(&huart1, (uint8_t *) buffer, (uint16_t) strlen(buffer), 30000);
	  memset(buffer, 0, strlen(buffer));

	  sprintf(buffer, "------------------------------------ \r \n");
	  HAL_UART_Transmit(&huart1, (uint8_t *) buffer, (uint16_t) strlen(buffer), 30000);
	  memset(buffer, 0, strlen(buffer));

	  sprintf(buffer, "|                                  | \r \n");
	  HAL_UART_Transmit(&huart1, (uint8_t *) buffer, (uint16_t) strlen(buffer), 30000);
	  memset(buffer, 0, strlen(buffer));

	  sprintf(buffer, "|           Time is up!            | \r \n");
	  HAL_UART_Transmit(&huart1, (uint8_t *) buffer, (uint16_t) strlen(buffer), 30000);
	  memset(buffer, 0, strlen(buffer));

	  sprintf(buffer, "|          Your score is %d .       | \r \n",score);
	  HAL_UART_Transmit(&huart1, (uint8_t *) buffer, (uint16_t) strlen(buffer), 30000);
	  memset(buffer, 0, strlen(buffer));

	  sprintf(buffer, "|  Press blue button to continue.  | \r \n");
	  HAL_UART_Transmit(&huart1, (uint8_t *) buffer, (uint16_t) strlen(buffer), 30000);
	  memset(buffer, 0, strlen(buffer));

	  sprintf(buffer, "|      Difficulty is %d 	    | \r \n",difficulty);
	  HAL_UART_Transmit(&huart1, (uint8_t *) buffer, (uint16_t) strlen(buffer), 30000);
	  memset(buffer, 0, strlen(buffer));

	  sprintf(buffer, "------------------------------------ \r \n");
	  HAL_UART_Transmit(&huart1, (uint8_t *) buffer, (uint16_t) strlen(buffer), 30000);
	  memset(buffer, 0, strlen(buffer));
}

void printSuccess(){

	 sprintf(buffer, "\033[2J\033[9A");
	 HAL_UART_Transmit(&huart1, (uint8_t *) buffer, (uint16_t) strlen(buffer), 30000);
	 memset(buffer, 0, strlen(buffer));

	  sprintf(buffer, "              You Won !!!            \r \n");
	  HAL_UART_Transmit(&huart1, (uint8_t *) buffer, (uint16_t) strlen(buffer), 30000);
	  memset(buffer, 0, strlen(buffer));

	  sprintf(buffer, "------------------------------------ \r \n");
	  HAL_UART_Transmit(&huart1, (uint8_t *) buffer, (uint16_t) strlen(buffer), 30000);
	  memset(buffer, 0, strlen(buffer));

	  sprintf(buffer, "|                                  | \r \n");
	  HAL_UART_Transmit(&huart1, (uint8_t *) buffer, (uint16_t) strlen(buffer), 30000);
	  memset(buffer, 0, strlen(buffer));

	  sprintf(buffer, "|        Congratuations!           | \r \n");
	  HAL_UART_Transmit(&huart1, (uint8_t *) buffer, (uint16_t) strlen(buffer), 30000);
	  memset(buffer, 0, strlen(buffer));

	  sprintf(buffer, "|          Your score is %d .       | \r \n",score);
	  HAL_UART_Transmit(&huart1, (uint8_t *) buffer, (uint16_t) strlen(buffer), 30000);
	  memset(buffer, 0, strlen(buffer));

	  sprintf(buffer, "|  Press blue button to continue.  | \r \n");
	  HAL_UART_Transmit(&huart1, (uint8_t *) buffer, (uint16_t) strlen(buffer), 30000);
	  memset(buffer, 0, strlen(buffer));

	  sprintf(buffer, "|      Difficulty is %d 	    | \r \n",difficulty);
	  HAL_UART_Transmit(&huart1, (uint8_t *) buffer, (uint16_t) strlen(buffer), 30000);
	  memset(buffer, 0, strlen(buffer));

	  sprintf(buffer, "------------------------------------ \r \n");
	  HAL_UART_Transmit(&huart1, (uint8_t *) buffer, (uint16_t) strlen(buffer), 30000);
	  memset(buffer, 0, strlen(buffer));
}

void printPause(){

	 sprintf(buffer, "\033[2J\033[9A");
	 HAL_UART_Transmit(&huart1, (uint8_t *) buffer, (uint16_t) strlen(buffer), 30000);
	 memset(buffer, 0, strlen(buffer));

	  sprintf(buffer, "         Whac The Mole!!!            \r \n");
	  HAL_UART_Transmit(&huart1, (uint8_t *) buffer, (uint16_t) strlen(buffer), 30000);
	  memset(buffer, 0, strlen(buffer));

	  sprintf(buffer, "------------------------------------ \r \n");
	  HAL_UART_Transmit(&huart1, (uint8_t *) buffer, (uint16_t) strlen(buffer), 30000);
	  memset(buffer, 0, strlen(buffer));

	  sprintf(buffer, "|                                  | \r \n");
	  HAL_UART_Transmit(&huart1, (uint8_t *) buffer, (uint16_t) strlen(buffer), 30000);
	  memset(buffer, 0, strlen(buffer));

	  sprintf(buffer, "|   The game is paused currently.  | \r \n");
	  HAL_UART_Transmit(&huart1, (uint8_t *) buffer, (uint16_t) strlen(buffer), 30000);
	  memset(buffer, 0, strlen(buffer));

	  sprintf(buffer, "|                                  | \r \n");
	  HAL_UART_Transmit(&huart1, (uint8_t *) buffer, (uint16_t) strlen(buffer), 30000);
	  memset(buffer, 0, strlen(buffer));

	  sprintf(buffer, "|  Press blue button to continue.  | \r \n");
	  HAL_UART_Transmit(&huart1, (uint8_t *) buffer, (uint16_t) strlen(buffer), 30000);
	  memset(buffer, 0, strlen(buffer));

	  sprintf(buffer, "|                                  | \r \n");
	  HAL_UART_Transmit(&huart1, (uint8_t *) buffer, (uint16_t) strlen(buffer), 30000);
	  memset(buffer, 0, strlen(buffer));

	  sprintf(buffer, "------------------------------------ \r \n");
	  HAL_UART_Transmit(&huart1, (uint8_t *) buffer, (uint16_t) strlen(buffer), 30000);
	  memset(buffer, 0, strlen(buffer));
}

void refreshAndPrint(){

	 sprintf(buffer, "\033[2J\033[9A");
	 HAL_UART_Transmit(&huart1, (uint8_t *) buffer, (uint16_t) strlen(buffer), 30000);
	 memset(buffer, 0, strlen(buffer));

	  sprintf(buffer, "         Whac The Mole!!!            \r \n");
	  HAL_UART_Transmit(&huart1, (uint8_t *) buffer, (uint16_t) strlen(buffer), 30000);
	  memset(buffer, 0, strlen(buffer));

	  sprintf(buffer, "------------------------------------ \r \n");
	  HAL_UART_Transmit(&huart1, (uint8_t *) buffer, (uint16_t) strlen(buffer), 30000);
	  memset(buffer, 0, strlen(buffer));

	  if(score < 10){
		  if(score,time_counter<10){
			  sprintf(buffer, "|Score: %d                Time: %d   | \r \n",score,time_counter);
		  }
		  else{
			  sprintf(buffer, "|Score: %d                Time: %d  | \r \n",score,time_counter);
		  }
	  }else{
		  if(score,time_counter<10){
			  sprintf(buffer, "|Score: %d               Time: %d   | \r \n",score,time_counter);
		  }
		  else{
			  sprintf(buffer, "|Score: %d               Time: %d  | \r \n",score,time_counter);
		  }
	  }

	  HAL_UART_Transmit(&huart1, (uint8_t *) buffer, (uint16_t) strlen(buffer), 30000);
	  memset(buffer, 0, strlen(buffer));

	  sprintf(buffer, "|   ____    ____    ____    ____   | \r \n");
	  HAL_UART_Transmit(&huart1, (uint8_t *) buffer, (uint16_t) strlen(buffer), 30000);
	  memset(buffer, 0, strlen(buffer));

	  //if(LED_1_status = 0 && LED_2_status = 0 && LED_3_status = 0 && LED_4_status = 0)
	  sprintf(tBuff0, " |    | ");
	  sprintf(tBuff1, " | aa | ");
	  sprintf(buffer, "| ");

	  if(LED_1_status == 1) strcat(buffer, tBuff1);
	  else strcat(buffer, tBuff0);
	  if(LED_2_status == 1) strcat(buffer, tBuff1);
	  else strcat(buffer, tBuff0);
	  if(LED_3_status == 1) strcat(buffer, tBuff1);
	  else strcat(buffer, tBuff0);
	  if(LED_4_status == 1) strcat(buffer, tBuff1);
	  else strcat(buffer, tBuff0);

	  strcat(buffer, " | \r \n");

	  HAL_UART_Transmit(&huart1, (uint8_t *) buffer, (uint16_t) strlen(buffer), 30000);
	  memset(buffer, 0, strlen(buffer));

	  //sprintf(buffer, "|  |    |  |    |  |    |  |    |  | \r \n");
	  HAL_UART_Transmit(&huart1, (uint8_t *) buffer, (uint16_t) strlen(buffer), 30000);
	  memset(buffer, 0, strlen(buffer));

	  sprintf(buffer, "|  |    |  |    |  |    |  |    |  | \r \n");
	  HAL_UART_Transmit(&huart1, (uint8_t *) buffer, (uint16_t) strlen(buffer), 30000);
	  memset(buffer, 0, strlen(buffer));

	  sprintf(buffer, "|                                  | \r \n");
	  HAL_UART_Transmit(&huart1, (uint8_t *) buffer, (uint16_t) strlen(buffer), 30000);
	  memset(buffer, 0, strlen(buffer));

	  sprintf(buffer, "------------------------------------ \r \n");
	  HAL_UART_Transmit(&huart1, (uint8_t *) buffer, (uint16_t) strlen(buffer), 30000);
	  memset(buffer, 0, strlen(buffer));

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
  MX_USART1_UART_Init();
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
  uint32_t music_addr = 0x030000;
  uint32_t music2_addr = 0x050000;
  for(int i = 0; i < 490; i++){
	  if(BSP_QSPI_Write((uint8_t *)B5, tone_addr, 45) != QSPI_OK){
		  Error_Handler();
	  }
	  tone_addr += 45;
  }
  for(int i = 0; i < 525; i++){//22050 = 5*4096+6*256+2*16+2 = 0x005622;
	  if(BSP_QSPI_Write((uint8_t *)C6, tone_addr, 42) != QSPI_OK){
		  Error_Handler();
	  }
	  tone_addr += 42;
  }
  for(int i = 0; i < 648; i++){//0x00AC44;
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
////////////////////////
  //////////////
  ///////////
  for(int i = 0; i < 78; i++){
	  if(BSP_QSPI_Write((uint8_t *)G6, music_addr, 28) != QSPI_OK){
		  Error_Handler();
	  }
	  music_addr += 28;
  }
  for(int i = 0; i < 88; i++){
	  if(BSP_QSPI_Write((uint8_t *)A6, music_addr, 25) != QSPI_OK){
		  Error_Handler();
	  }
	  music_addr += 25;
  }
  for(int i = 0; i < 49; i++){
	  if(BSP_QSPI_Write((uint8_t *)B5, music_addr, 45) != QSPI_OK){
		  Error_Handler();
	  }
	  music_addr += 45;
  }
  for(int i = 0; i < 49; i++){
	  if(BSP_QSPI_Write((uint8_t *)B5, music_addr, 45) != QSPI_OK){
		  Error_Handler();
	  }
	  music_addr += 45;
  }
  for(int i = 0; i < 52; i++){//22050 = 5*4096+6*256+2*16+2 = 0x005622;
	  if(BSP_QSPI_Write((uint8_t *)C6, music_addr, 42) != QSPI_OK){
		  Error_Handler();
	  }
	  music_addr += 42;
  }
  for(int i = 0; i < 52; i++){//22050 = 5*4096+6*256+2*16+2 = 0x005622;
	  if(BSP_QSPI_Write((uint8_t *)C6, music_addr, 42) != QSPI_OK){
		  Error_Handler();
	  }
	  music_addr += 42;
  }
  for(int i = 0; i < 49; i++){
	  if(BSP_QSPI_Write((uint8_t *)B5, music_addr, 45) != QSPI_OK){
		  Error_Handler();
	  }
	  music_addr += 45;
  }
  for(int i = 0; i < 49; i++){
	  if(BSP_QSPI_Write((uint8_t *)B5, music_addr, 45) != QSPI_OK){
		  Error_Handler();
	  }
	  music_addr += 45;
  }
  for(int i = 0; i < 52; i++){//22050 = 5*4096+6*256+2*16+2 = 0x005622;
	  if(BSP_QSPI_Write((uint8_t *)C6, music_addr, 42) != QSPI_OK){
		  Error_Handler();
	  }
	  music_addr += 42;
  }
  for(int i = 0; i < 52; i++){//22050 = 5*4096+6*256+2*16+2 = 0x005622;
	  if(BSP_QSPI_Write((uint8_t *)C6, music_addr, 42) != QSPI_OK){
		  Error_Handler();
	  }
	  music_addr += 42;
  }

/////222////

 ///222///
  for(int i = 0; i < 78; i++){
	  if(BSP_QSPI_Write((uint8_t *)G6, music2_addr, 28) != QSPI_OK){
		  Error_Handler();
	  }
	  music2_addr += 28;
  }
  for(int i = 0; i < 88; i++){
	  if(BSP_QSPI_Write((uint8_t *)A6, music2_addr, 25) != QSPI_OK){
		  Error_Handler();
	  }
	  music2_addr += 25;
  }
  for(int i = 0; i < 49; i++){
	  if(BSP_QSPI_Write((uint8_t *)B5, music2_addr, 45) != QSPI_OK){
		  Error_Handler();
	  }
	  music2_addr += 45;
  }
  for(int i = 0; i < 49; i++){
	  if(BSP_QSPI_Write((uint8_t *)B5, music2_addr, 45) != QSPI_OK){
		  Error_Handler();
	  }
	  music2_addr += 45;
  }
  for(int i = 0; i < 52; i++){//22050 = 5*4096+6*256+2*16+2 = 0x005622;
	  if(BSP_QSPI_Write((uint8_t *)C6, music2_addr, 42) != QSPI_OK){
		  Error_Handler();
	  }
	  music2_addr += 42;
  }
  for(int i = 0; i < 52; i++){//22050 = 5*4096+6*256+2*16+2 = 0x005622;
	  if(BSP_QSPI_Write((uint8_t *)C6, music2_addr, 42) != QSPI_OK){
		  Error_Handler();
	  }
	  music2_addr += 42;
  }
  for(int i = 0; i < 49; i++){
	  if(BSP_QSPI_Write((uint8_t *)B5, music2_addr, 45) != QSPI_OK){
		  Error_Handler();
	  }
	  music2_addr += 45;
  }
  for(int i = 0; i < 49; i++){
	  if(BSP_QSPI_Write((uint8_t *)B5, music2_addr, 45) != QSPI_OK){
		  Error_Handler();
	  }
	  music2_addr += 45;
  }
  for(int i = 0; i < 88; i++){
	  if(BSP_QSPI_Write((uint8_t *)A6, music2_addr, 25) != QSPI_OK){
		  Error_Handler();
	  }
	  music2_addr += 25;
  }
  for(int i = 0; i < 100; i++){
	  if(BSP_QSPI_Write((uint8_t *)B6, music2_addr, 22) != QSPI_OK){
		  Error_Handler();
	  }
	  music2_addr += 22;
  }
  //Read the data
  if(BSP_QSPI_Read((uint8_t *)play, 0x030000, 22050) != QSPI_OK){
	  Error_Handler();
  }
  HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, play, 22050, DAC_ALIGN_8B_R);
  printWelcome();
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  osThreadDef(SystemController, control, osPriorityHigh, 0, 128);
  SystemControllerHandle = osThreadCreate(osThread(SystemController), NULL);

  osThreadDef(TransmitTask, button_pressed_task, osPriorityIdle, 0, 128);
  TransmitTaskHandle = osThreadCreate(osThread(TransmitTask), NULL);


  //osThreadDef(defaultTask, button_pressed_task, osPriorityNormal, 0, 128);
  //defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 5, 0);
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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED4_Pin|LED1_Pin|LED2_Pin|LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_RED_Pin */
  GPIO_InitStruct.Pin = LED_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_RED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BluePB_Pin */
  GPIO_InitStruct.Pin = BluePB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BluePB_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED4_Pin LED1_Pin LED2_Pin LED3_Pin */
  GPIO_InitStruct.Pin = LED4_Pin|LED1_Pin|LED2_Pin|LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB3_Pin */
  GPIO_InitStruct.Pin = PB3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PB3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2_Pin PB1 PB4_Pin */
  GPIO_InitStruct.Pin = PB2_Pin|GPIO_PIN_1|PB4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ROW1_Pin */
  GPIO_InitStruct.Pin = ROW1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ROW1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1_Pin */
  GPIO_InitStruct.Pin = PB1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PB1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void control(void){
	for(;;){
		osDelay(100);
		if(time_counter == 60){
			if(score >= difficulty){
				isStop = 1;
				isDelaying = 1;
				isPause = 1;
				printSuccess();
				HAL_Delay(500);
				time_counter = 0;
			}
			else{
				isStop = 1;
				isDelaying = 1;
				isPause = 1;
				printFailed();
				HAL_Delay(500);
				time_counter = 0;
			}
		}
		if(HAL_GPIO_ReadPin (BluePB_GPIO_Port, BluePB_Pin)==0){
			if(isStop==1){
				score = 0;
				isStop = 0;
				isPause = 0;
				isDelaying = 0;
				HAL_Delay(500);
			}
			else{
				if(isPause==0){
								isDelaying = 1;
								osThreadSuspend(TransmitTaskHandle);
								osThreadSuspend(defaultTaskHandle);
								isPause = 1;
								printPause();
								HAL_Delay(500);
								  if(BSP_QSPI_Read((uint8_t *)play, 0x050000, 22050) != QSPI_OK){
									  Error_Handler();
								  }
							 	HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, play, 22050, DAC_ALIGN_8B_R);


							}
				else if(isPause==1){
					isDelaying = 0;
					osThreadResume(TransmitTaskHandle);
					osThreadResume(defaultTaskHandle);
					isPause = 0;
					HAL_Delay(500);
				}
			}
		}
		if( HAL_GPIO_ReadPin(PB2_GPIO_Port, PB2_Pin)==1){
			if(isStop==1){
				difficulty = difficulty + 1;
				if(difficulty>=20) difficulty = 12;
				printWelcome();
				HAL_Delay(500);
			}
		}
		if( HAL_GPIO_ReadPin(PB1_GPIO_Port, PB1_Pin)==1){
			if(isStop==1){
				difficulty = difficulty - 1;
				if(difficulty<=2) difficulty = 2;
				printWelcome();
				HAL_Delay(500);
			}
		}
	}
}
void button_pressed_task(void)
{
	  for(;;)
	  {
	    osDelay(100);
	if(HAL_GPIO_ReadPin (BluePB_GPIO_Port, BluePB_Pin)==0){
		//osThreadSuspend(TransmitTaskHandle);
			  /*if(HAL_GPIO_ReadPin (LED_GPIO_Port, LED_Pin)){
				  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
				  if(BSP_QSPI_Read((uint8_t *)play, 0x01AEAA, 22050) != QSPI_OK){
				  		Error_Handler();
				  }
				  HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, play, 22050, DAC_ALIGN_8B_R);
				  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
				  LED_0_status = 0;
				  isDelaying = 1;
				  HAL_Delay(200);
				  isDelaying = 0;
				  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
				  if(BSP_QSPI_Read((uint8_t *)play, 0x02AEAA, 22050) != QSPI_OK){
				  			Error_Handler();
				  }
				  HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, empty, 22050, DAC_ALIGN_8B_R);
				  score = score+1;
				  refreshAndPrint();
				  TransmitTaskHandle->
			  }*/
		  }
		  if( HAL_GPIO_ReadPin(PB1_GPIO_Port, PB1_Pin)==1){
			  if(HAL_GPIO_ReadPin (LED1_GPIO_Port, LED1_Pin)){
				  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
				  if(BSP_QSPI_Read((uint8_t *)play, 0x000000, 22050) != QSPI_OK){
					  Error_Handler();
				  }
			 	HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, play, 22050, DAC_ALIGN_8B_R);
			 	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
			 	LED_1_status = 0;
			 	isDelaying = 1;
			 	HAL_Delay(200);
			 	isDelaying = 0;
			 	HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
				if(BSP_QSPI_Read((uint8_t *)play, 0x02AEAA, 22050) != QSPI_OK){
						 Error_Handler();
				}
				HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, empty, 22050, DAC_ALIGN_8B_R);
				score = score+1;
				refreshAndPrint();
			  }
			  else score = score - 1;
		  }

		  if( HAL_GPIO_ReadPin(PB2_GPIO_Port, PB2_Pin)==1){
			  if(HAL_GPIO_ReadPin (LED2_GPIO_Port, LED2_Pin)){
				  	  	  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
						  if(BSP_QSPI_Read((uint8_t *)play, 0x005622, 22050) != QSPI_OK){
							  Error_Handler();
						  }
					 	  HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, play, 22050, DAC_ALIGN_8B_R);
			 			  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
			 			  LED_2_status = 0;
			 			  isDelaying = 1;
			 			  HAL_Delay(200);
			 			  isDelaying = 0;
			 			  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
			 			  if(BSP_QSPI_Read((uint8_t *)play, 0x02AEAA, 22050) != QSPI_OK){
			 				  Error_Handler();
			 			  }
			 			  HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, empty, 22050, DAC_ALIGN_8B_R);
			 			 score = score+1;
			 			 refreshAndPrint();
			  }
			  else score = score - 1;
		  }

		  if( HAL_GPIO_ReadPin(PB3_GPIO_Port, PB3_Pin)==1){
			  if(HAL_GPIO_ReadPin (LED3_GPIO_Port, LED3_Pin)){
			 			  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
						  if(BSP_QSPI_Read((uint8_t *)play, 0x00AC44, 22050) != QSPI_OK){
							  Error_Handler();
						  }
					 	  HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, play, 22050, DAC_ALIGN_8B_R);
			 			  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
			 			  LED_3_status = 0;
			 			  isDelaying = 1;
			 			  HAL_Delay(200);
			 			  isDelaying = 0;
			 			  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
			 			  if(BSP_QSPI_Read((uint8_t *)play, 0x02AEAA, 22050) != QSPI_OK){
			 				  Error_Handler();
			 			  }
			 			  HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, empty, 22050, DAC_ALIGN_8B_R);
			 			 score = score+1;
			 			 refreshAndPrint();
			  }
			  else score = score - 1;
		  }

		  if( HAL_GPIO_ReadPin(PB4_GPIO_Port, PB4_Pin)==1){
			  if(HAL_GPIO_ReadPin (LED4_GPIO_Port, LED4_Pin)){
			 			  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
						  if(BSP_QSPI_Read((uint8_t *)play, 0x015888, 22050) != QSPI_OK){
							  Error_Handler();
						  }
					 	  HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, play, 22050, DAC_ALIGN_8B_R);
			 			  HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
			 			  LED_4_status = 0;
			 			  isDelaying = 1;
			 			  HAL_Delay(200);
			 			  isDelaying = 0;
			 			  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
			 			  if(BSP_QSPI_Read((uint8_t *)play, 0x02AEAA, 22050) != QSPI_OK){
			 				  Error_Handler();
			 			  }
			 			  HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, empty, 22050, DAC_ALIGN_8B_R);
			 			 refreshAndPrint();
			 			score = score+1;
			  }
			  else score = score - 1;
		  }
	  }
}

int lower = 1;
int upper = 3;
int DAC_status = 0;

void HAL_DAC_ConvCpltCallbackCh1 (DAC_HandleTypeDef * hdac){
	if(isDelaying == 1) return;
	time_counter  = time_counter + 1;
	/*if(DAC_status == 1){
		if(BSP_QSPI_Read((uint8_t *)play, 0x02AEAA, 22050) != QSPI_OK){
					  Error_Handler();
		}
		HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, empty, 22050, DAC_ALIGN_8B_R);
		DAC_status = 0;
	}*/
	int randomNumber0 = (rand() % (upper - lower + 1)) + lower;
	int randomNumber1 = (rand() % (upper - lower + 1)) + lower;
	int randomNumber2 = (rand() % (upper - lower + 1)) + lower;
	int randomNumber3 = (rand() % (upper - lower + 1)) + lower;
	int randomNumber4 = (rand() % (upper - lower + 1)) + lower;

	if(randomNumber0 % 3 == 0 && LED_0_status ==0 ){
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
		LED_0_status = 1;
	}
	else if(randomNumber0 % 3 == 0 && LED_0_status ==1 ){
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
			LED_0_status = 0;
	}

	if(randomNumber1 % 3 == 0 && LED_0_status ==0 ){
			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin,GPIO_PIN_SET);
			LED_1_status = 1;
	}
	else if(randomNumber1 % 3 == 0 && LED_0_status ==1 ){
			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
			LED_1_status = 0;
	}

	if(randomNumber2 % 3 == 0 && LED_0_status ==0 ){
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
			LED_2_status = 1;
	}
	else if(randomNumber2 % 3 == 0 && LED_0_status ==1 ){
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
			LED_2_status = 0;
	}

	if(randomNumber3 % 3 == 0 && LED_0_status ==0 ){
			HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
			LED_3_status = 1;
	}
	else if(randomNumber3 % 3 == 0 && LED_0_status ==1 ){
			HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
			LED_3_status = 0;
	}

	if(randomNumber4 % 3 == 0 && LED_0_status ==0 ){
			HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
			LED_4_status = 1;
	}
	else if(randomNumber4 % 3 == 0 && LED_0_status ==1 ){
			HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
			LED_4_status = 0;
	}
	refreshAndPrint();
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

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
