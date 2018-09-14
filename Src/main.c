/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * Copyright (c) 2018 STMicroelectronics International N.V.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted, provided that the following conditions are met:
 *
 * 1. Redistribution of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of other
 *    contributors to this software may be used to endorse or promote products
 *    derived from this software without specific written permission.
 * 4. This software, including modifications and/or derivative works of this
 *    software, must execute solely and exclusively on microcontroller or
 *    microprocessor devices manufactured by or for STMicroelectronics.
 * 5. Redistribution and use of this software other than as permitted under
 *    this license is void and will automatically terminate your rights under
 *    this license.
 *
 * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
 * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
 * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include "uart_print.h"
#include "flash_operations.h"
//#include "app_mems-library.h"
#include "lsm6dsl_kl.h"
#include "lsm303agr_kl.h"
#include <string.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

osThreadId BlinkTaskHandle;
osThreadId ImuTaskHandle;
osThreadId ButtonTaskHandle;
osThreadId PrintFlashTaskHandle;
osSemaphoreId ButtonBinarySemHandle;
osSemaphoreId ImuSampleBinarySemHandle;

/* USER CODE BEGIN PV */

/* Private macro -------------------------------------------------------------*/

#define UART_RX_SIZE 3

/* Private variables ---------------------------------------------------------*/

enum ImuTaskRunning
{
	IDLE, RUNNING
};
volatile enum ImuTaskRunning ImuIsRunning = IDLE;

typedef struct
{
	uint16_t Option;
} OptionStruct;

osMailQDef(UartPrintOptionsQueueHandle, 1, OptionStruct);
osMailQId UartPrintOptionsQueueHandle;

/* Global UART RX buffer */
uint8_t UartRxData[UART_RX_SIZE];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
void StartBlinkTask(void const * argument);
void StartImuTask(void const * argument);
void StartButtonTask(void const * argument);
void StartPrintFlashTask(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
//static void Print_Addr(uint32_t addr);
static uint32_t Pack_16to32(uint16_t temp1, uint16_t temp2);
static void Unpack_32to16(uint32_t temp1, uint16_t *temp2, uint16_t *temp3);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void Print_Banner();

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 *
 * @retval None
 */
int main(void)
{
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

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
	MX_I2C1_Init();
	/* USER CODE BEGIN 2 */
	HAL_UART_Receive_IT(&huart2, UartRxData, UART_RX_SIZE);
	//__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
	LSM6DS3_begin();
	LSM303AGR_magInit();
	//MX_MEMS_Library_Init();
	if (!Flash_EraseAll()) {
		UART_Print("Flash erase ERR\n\r");
	}
	Print_Banner();
	//UART_Print("Press USER Button to start sampling IMU. Send any character to print Flash.\n\r");

	/* USER CODE END 2 */

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* Create the semaphores(s) */
	/* definition and creation of ButtonBinarySem */
	osSemaphoreDef(ButtonBinarySem);
	ButtonBinarySemHandle = osSemaphoreCreate(osSemaphore(ButtonBinarySem), 1);

	/* definition and creation of ImuSampleBinarySem */
	osSemaphoreDef(ImuSampleBinarySem);
	ImuSampleBinarySemHandle = osSemaphoreCreate(
			osSemaphore(ImuSampleBinarySem), 1);

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* Create the thread(s) */
	/* definition and creation of BlinkTask */
	osThreadDef(BlinkTask, StartBlinkTask, osPriorityNormal, 0, 128);
	BlinkTaskHandle = osThreadCreate(osThread(BlinkTask), NULL);

	/* definition and creation of ImuTask */
	osThreadDef(ImuTask, StartImuTask, osPriorityNormal, 0, 128);
	ImuTaskHandle = osThreadCreate(osThread(ImuTask), NULL);

	/* definition and creation of ButtonTask */
	osThreadDef(ButtonTask, StartButtonTask, osPriorityNormal, 0, 128);
	ButtonTaskHandle = osThreadCreate(osThread(ButtonTask), NULL);

	/* definition and creation of PrintFlashTask */
	osThreadDef(PrintFlashTask, StartPrintFlashTask, osPriorityNormal, 0, 128);
	PrintFlashTaskHandle = osThreadCreate(osThread(PrintFlashTask), NULL);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* Create the queue(s) */
	/* definition and creation of UartPrintOptionsQueue */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	UartPrintOptionsQueueHandle = osMailCreate(
			osMailQ(UartPrintOptionsQueueHandle), NULL);
	/* USER CODE END RTOS_QUEUES */

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

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

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	/**Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE()
	;

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 16;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/** Configure pins as 
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 */
static void MX_GPIO_Init(void)
{

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE()
	;
	__HAL_RCC_GPIOH_CLK_ENABLE()
	;
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;
	__HAL_RCC_GPIOB_CLK_ENABLE()
	;

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_BUTTON_Pin */
	GPIO_InitStruct.Pin = B1_BUTTON_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_BUTTON_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LD2_Pin */
	GPIO_InitStruct.Pin = LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == B1_BUTTON_Pin) {
		osSemaphoreRelease(ButtonBinarySemHandle);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_UART_Receive_IT(&huart2, UartRxData, UART_RX_SIZE);

	static OptionStruct *PrintOption;
	PrintOption = osMailAlloc(UartPrintOptionsQueueHandle, osWaitForever);

	uint8_t UartRxString[UART_RX_SIZE + 1];

	strncpy((char *) UartRxString, (char *) UartRxData, 3);
	UartRxString[UART_RX_SIZE] = '\0';

	//UART_Print("%s\n\r", UartRxString);

	if (strcmp((char *) UartRxString, "reg") == 0) {
		PrintOption->Option = 1;
	}
	else if (strcmp((char *) UartRxString, "mag") == 0) {
		PrintOption->Option = 2;
	}
	else if (strcmp((char *) UartRxString, "acc") == 0) {
		PrintOption->Option = 3;
	}
	else if (strcmp((char *) UartRxString, "gyr") == 0) {
		PrintOption->Option = 4;
	}
	else if (strcmp((char *) UartRxString, "all") == 0) {
		PrintOption->Option = 5;
	}
	else {
		PrintOption->Option = 0;
	}
	osMailPut(UartPrintOptionsQueueHandle, PrintOption);
}

static uint32_t Pack_16to32(uint16_t temp1, uint16_t temp2)
{
	return ((uint32_t) temp1 << 16) + temp2;
}

static void Unpack_32to16(uint32_t temp1, uint16_t *temp2, uint16_t *temp3)
{
	*temp2 = (uint16_t) (temp1 >> 16);
	*temp3 = (uint16_t) (temp1 & 0xffff);
}

void Print_Banner()
{
	UART_Print(
			"********************************* FLASH DESTROYER v1.0 *********************************\n\r");
	UART_Print("*\n\r");
	UART_Print(
			"* Press USER BUTTON to start/stop sampling of the 9-DOF IMU\n\r");
	UART_Print("*\n\r");
	UART_Print("* Send 'reg' to print all raw registers\n\r");
	UART_Print("* Send 'all' to print all scaled sensors \n\r");
	UART_Print("* Send 'mag' to print scaled magnetometer \n\r");
	UART_Print("* Send 'acc' to print scaled accelerometer \n\r");
	UART_Print("* Send 'gyr' to print scaled gyrometer \n\r");
	UART_Print("*\n\r");
	UART_Print(
			"****************************************************************************************\n\r");
}
/*static void Print_Addr(uint16_t temp)
 {
 //uint16_t temp1, temp2;
 //Unpack_32to16(addr, &temp1, &temp2);
 static uint8_t count;
 UART_Print(" %04X", temp);
 count++;
 if (count > 9) {
 UART_Print("\n\r");
 count = 0;
 }
 }*/

/* USER CODE END 4 */

/* StartBlinkTask function */
void StartBlinkTask(void const * argument)
{

	/* USER CODE BEGIN 5 */
	/* Infinite loop */
	for (;;) {
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		osDelay(1000);
	}
	/* USER CODE END 5 */
}

/* StartImuTask function */
void StartImuTask(void const * argument)
{
	/* USER CODE BEGIN StartImuTask */
	osSemaphoreWait(ImuSampleBinarySemHandle, osWaitForever);
	//osThreadSuspend(ImuTaskHandle);
	/* Infinite loop */
	for (;;) {
		osSemaphoreWait(ImuSampleBinarySemHandle, osWaitForever);
		osSemaphoreRelease(ImuSampleBinarySemHandle);
		osDelay(500);

		uint16_t temp1, temp2;

		temp1 = (uint16_t) LSM303AGR_readRawMagX();
		temp2 = (uint16_t) LSM303AGR_readRawMagY();
		//UART_Print("%04X ", temp1);
		//UART_Print("%04X ", temp2);
		if (!Flash_WriteWord(Pack_16to32(temp1, temp2))) {
			UART_Print("Error writing flash\n\r");
		}
		temp1 = (uint16_t) LSM303AGR_readRawMagZ();
		temp2 = (uint16_t) LSM6DS3_readRawAccX();
		//UART_Print("%04X ", temp1);
		//UART_Print("%04X ", temp2);
		if (!Flash_WriteWord(Pack_16to32(temp1, temp2))) {
			UART_Print("Error writing flash\n\r");
		}
		temp1 = (uint16_t) LSM6DS3_readRawAccY();
		temp2 = (uint16_t) LSM6DS3_readRawAccZ();
		//UART_Print("%04X ", temp1);
		//UART_Print("%04X ", temp2);
		if (!Flash_WriteWord(Pack_16to32(temp1, temp2))) {
			UART_Print("Error writing flash\n\r");
		}
		temp1 = (uint16_t) LSM6DS3_readRawGyrX();
		temp2 = (uint16_t) LSM6DS3_readRawGyrY();
		//UART_Print("%04X ", temp1);
		//UART_Print("%04X ", temp2);
		//UART_Print("\n\r");
		if (!Flash_WriteWord(Pack_16to32(temp1, temp2))) {
			UART_Print("Error writing flash\n\r");
		}
		temp1 = (uint16_t) LSM6DS3_readRawGyrZ();
		temp2 = (uint16_t) 0x0000;
		//UART_Print("%04X ", temp1);
		//UART_Print("%04X ", temp2);
		if (!Flash_WriteWord(Pack_16to32(temp1, temp2))) {
			UART_Print("Error writing flash\n\r");
		}
		/*UART_Print("%04X ", LSM303AGR_readRawMagX());
		 UART_Print("%04X ", LSM303AGR_readRawMagY());
		 UART_Print("%04X ", LSM303AGR_readRawMagZ());

		 UART_Print("%04X ", LSM6DS3_readRawAccelX());
		 UART_Print("%04X ", LSM6DS3_readRawAccelY());
		 UART_Print("%04X ", LSM6DS3_readRawAccelZ());

		 UART_Print("%04X ", LSM6DS3_readRawGyroX());
		 UART_Print("%04X ", LSM6DS3_readRawGyroY());
		 UART_Print("%04X ", LSM6DS3_readRawGyroZ());
		 UART_Print("\n\r");*/
	}
	/* USER CODE END StartImuTask */
}

/* StartButtonTask function */
void StartButtonTask(void const * argument)
{
	/* USER CODE BEGIN StartButtonTask */

	/* Initialize ButtonBinarySem to 0 */
	osSemaphoreWait(ButtonBinarySemHandle, osWaitForever);

	/* Infinite loop */
	for (;;) {
		osSemaphoreWait(ButtonBinarySemHandle, osWaitForever);
		/* Read button value in order to do a debounce check */
		/* By reading it instead of assuming '1' we can change trigger without having to change this task */
		uint8_t button_debounce = HAL_GPIO_ReadPin(B1_BUTTON_GPIO_Port,
		B1_BUTTON_Pin);
		osDelay(40);
		/* Read button value again and compare with previous value */
		if (HAL_GPIO_ReadPin(B1_BUTTON_GPIO_Port, B1_BUTTON_Pin)
				== button_debounce) {
			/* Turn on IMU sampling if it's not already sampling */
			if (ImuIsRunning == IDLE) {
				ImuIsRunning = RUNNING;
				//osThreadResume(ImuTaskHandle);
				UART_Print("Start Sampling IMU\n\r");
				osSemaphoreRelease(ImuSampleBinarySemHandle);
			}
			/* Else turn off IMU sampling if it's already sampling */
			else {
				ImuIsRunning = IDLE;
				//osThreadSuspend(ImuTaskHandle);
				UART_Print("Stop Sampling IMU\n\r");
				osSemaphoreWait(ImuSampleBinarySemHandle, osWaitForever);
			}
		}
	}
	/* USER CODE END StartButtonTask */
}

/* StartPrintFlashTask function */
void StartPrintFlashTask(void const * argument)
{
	/* USER CODE BEGIN StartPrintFlashTask */
	static OptionStruct *PrintOption;
	osEvent OptionEvent;
	/* Infinite loop */
	for (;;) {
		OptionEvent = osMailGet(UartPrintOptionsQueueHandle, osWaitForever);
		if (OptionEvent.status == osEventMail) {
			PrintOption = OptionEvent.value.p;
			osMailFree(UartPrintOptionsQueueHandle, PrintOption);
			if (PrintOption->Option) {
				// Start printing from the first address FLASH_USER_START_ADDR
				uint32_t addr_start = FLASH_USER_START_ADDR;

				// Stop printing at the last written address
				uint32_t addr_end = Flash_GetEndAddr();
				uint16_t addr_arr[10];
				while (addr_start < addr_end) {
					uint8_t i;
					for (i = 0; i < 10; i += 2) {
						Unpack_32to16(Flash_Read(addr_start), &addr_arr[i], &addr_arr[i + 1]);
						addr_start = addr_start + 4;
					}
					switch (PrintOption->Option)
					{
					case 1:
						UART_Print(
								" Mag X %04X Y %04X Z %04X Acc X %04X Y %04X Z %04X Gyr X %04X Y %04X Z %04X Padding %04X\n\r",
								addr_arr[0], addr_arr[1], addr_arr[2],
								addr_arr[3], addr_arr[4], addr_arr[5],
								addr_arr[6], addr_arr[7], addr_arr[8],
								addr_arr[9]);
						break;
					case 2:
						UART_Print(" Mag X %.4f Y %.4f Z %.4f\n\r",
								LSM303AGR_calcMag((int8_t) addr_arr[0]),
								LSM303AGR_calcMag((int8_t) addr_arr[1]),
								LSM303AGR_calcMag((int8_t) addr_arr[2]));
						break;
					case 3:
						UART_Print(" Acc X %.4f Y %.4f Z %.4f\n\r",
								LSM6DS3_calcAcc((int8_t) addr_arr[3]),
								LSM6DS3_calcAcc((int8_t) addr_arr[4]),
								LSM6DS3_calcAcc((int8_t) addr_arr[5]));
						//UART_Print("print acc\n\r");
						break;
					case 4:
						UART_Print(" Gyr X Y Z %.4f %.4f %.4f\n\r",
								LSM6DS3_calcGyr((int8_t) addr_arr[6]),
								LSM6DS3_calcGyr((int8_t) addr_arr[7]),
								LSM6DS3_calcGyr((int8_t) addr_arr[8]));
						break;
					case 5:
						//UART_Print("print all\n\r");
						UART_Print(" Mag X Y Z %.4f %.4f %.4f\n\r",
								LSM303AGR_calcMag((int8_t) addr_arr[0]),
								LSM303AGR_calcMag((int8_t) addr_arr[1]),
								LSM303AGR_calcMag((int8_t) addr_arr[2]));
						UART_Print(" Acc X Y Z %.4f %.4f %.4f\n\r",
								LSM6DS3_calcAcc((int8_t) addr_arr[3]),
								LSM6DS3_calcAcc((int8_t) addr_arr[4]),
								LSM6DS3_calcAcc((int8_t) addr_arr[5]));
						UART_Print(" Gyr X Y Z %.4f %.4f %.4f\n\r",
								LSM6DS3_calcGyr((int8_t) addr_arr[6]),
								LSM6DS3_calcGyr((int8_t) addr_arr[7]),
								LSM6DS3_calcGyr((int8_t) addr_arr[8]));
						break;
					default:
						break;
						// Wrong flag
						//UART_Print("Wrong Option! Try again...\n\r");
					}
					osDelay(10);
				}
				/*//osSemaphoreWait(UartBinarySemHandle, osWaitForever);
				 //UART_Print("Print Flash\n\r");
				 // Start printing from the first address FLASH_USER_START_ADDR
				 uint32_t addr_start = FLASH_USER_START_ADDR;
				 // Stop printing at the last written address
				 uint32_t addr_end = Flash_GetEndAddr();
				 // Print from flash while
				 while (addr_start < addr_end) {		//FLASH_USER_END_ADDR){
				 // Print flash mem over UART
				 Print_Addr(Flash_Read(addr_start));
				 // Go to next block of memory
				 addr_start = addr_start + 4;
				 }*/
			}
			else {
				UART_Print("Wrong Option! Try again...\n\r");
			}
		}
	}
	/* USER CODE END StartPrintFlashTask */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM3 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM3) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  file: The file name as string.
 * @param  line: The line in file as a number.
 * @retval None
 */
void _Error_Handler(char *file, int line)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
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
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
