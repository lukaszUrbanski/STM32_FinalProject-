/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "i2c.h"
#include "usart.h"
#include "bmp280.h"
#include "printf.h"
#include "SSD1306_OLED.h"
#include "GFX_BW.h"
#include "fonts/fonts.h"
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
/* USER CODE BEGIN Variables */
typedef struct
{
	float Temperature;
	float Pressure;
}BmpData_t;

//BmpData_t BmpData;

/* USER CODE END Variables */
/* Definitions for HeartbeatTask */
osThreadId_t HeartbeatTaskHandle;
const osThreadAttr_t HeartbeatTask_attributes = {
  .name = "HeartbeatTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for BMP280Task */
osThreadId_t BMP280TaskHandle;
const osThreadAttr_t BMP280Task_attributes = {
  .name = "BMP280Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for OledTask */
osThreadId_t OledTaskHandle;
const osThreadAttr_t OledTask_attributes = {
  .name = "OledTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for QueueBmpData */
osMessageQueueId_t QueueBmpDataHandle;
const osMessageQueueAttr_t QueueBmpData_attributes = {
  .name = "QueueBmpData"
};
/* Definitions for TimerBmpData */
osTimerId_t TimerBmpDataHandle;
const osTimerAttr_t TimerBmpData_attributes = {
  .name = "TimerBmpData"
};
/* Definitions for MutexPrintf */
osMutexId_t MutexPrintfHandle;
const osMutexAttr_t MutexPrintf_attributes = {
  .name = "MutexPrintf"
};
/* Definitions for MutexI2C */
osMutexId_t MutexI2CHandle;
const osMutexAttr_t MutexI2C_attributes = {
  .name = "MutexI2C"
};
/* Definitions for MutexBmpData */
osMutexId_t MutexBmpDataHandle;
const osMutexAttr_t MutexBmpData_attributes = {
  .name = "MutexBmpData"
};
/* Definitions for SemaphoreBmpQueue */
osSemaphoreId_t SemaphoreBmpQueueHandle;
const osSemaphoreAttr_t SemaphoreBmpQueue_attributes = {
  .name = "SemaphoreBmpQueue"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartHeartbeatTask(void *argument);
void StartBMP280Task(void *argument);
void StartOledTask(void *argument);
void TimerBmpDataCallback(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of MutexPrintf */
  MutexPrintfHandle = osMutexNew(&MutexPrintf_attributes);

  /* creation of MutexI2C */
  MutexI2CHandle = osMutexNew(&MutexI2C_attributes);

  /* creation of MutexBmpData */
  MutexBmpDataHandle = osMutexNew(&MutexBmpData_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of SemaphoreBmpQueue */
  SemaphoreBmpQueueHandle = osSemaphoreNew(1, 1, &SemaphoreBmpQueue_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of TimerBmpData */
  TimerBmpDataHandle = osTimerNew(TimerBmpDataCallback, osTimerPeriodic, NULL, &TimerBmpData_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of QueueBmpData */
  QueueBmpDataHandle = osMessageQueueNew (8, sizeof(BmpData_t), &QueueBmpData_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of HeartbeatTask */
  HeartbeatTaskHandle = osThreadNew(StartHeartbeatTask, NULL, &HeartbeatTask_attributes);

  /* creation of BMP280Task */
  BMP280TaskHandle = osThreadNew(StartBMP280Task, NULL, &BMP280Task_attributes);

  /* creation of OledTask */
  OledTaskHandle = osThreadNew(StartOledTask, NULL, &OledTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartHeartbeatTask */
/**
* @brief Function implementing the Heartbeat thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartHeartbeatTask */
void StartHeartbeatTask(void *argument)
{
  /* USER CODE BEGIN StartHeartbeatTask */
  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	  osDelay(500);
  }
  /* USER CODE END StartHeartbeatTask */
}

/* USER CODE BEGIN Header_StartBMP280Task */
/**
* @brief Function implementing the BMP280Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBMP280Task */
void StartBMP280Task(void *argument)
{
  /* USER CODE BEGIN StartBMP280Task */

	BMP280_t Bmp280;
	BmpData_t _BmpData;

	uint32_t DelayTick = osKernelGetTickCount();

	osMutexAcquire(MutexI2CHandle, osWaitForever);
	BMP280_Init(&Bmp280, &hi2c1, BMP280_I2C_ADDR);
	osMutexRelease(MutexI2CHandle);

	osTimerStart(TimerBmpDataHandle, 100);
  /* Infinite loop */
	for(;;)
	{
		osMutexAcquire(MutexI2CHandle, osWaitForever);
		BMP280_ReadTemperatureAndPressure(&Bmp280, &_BmpData.Temperature, &_BmpData.Pressure);
		osMutexRelease(MutexI2CHandle);

		if(osOK == osSemaphoreAcquire(SemaphoreBmpQueueHandle, 0))
		{
			osMessageQueuePut(QueueBmpDataHandle, &_BmpData, 0, osWaitForever);
		}

		printf("Temperature: %.2f, Pressure %.2f\n\r", _BmpData.Temperature, _BmpData.Pressure);

//		osMutexAcquire(MutexBmpDataHandle, osWaitForever);
//		BmpData.Temperature = _Temperature;
//		BmpData.Pressure = _Pressure;
//		osMutexRelease(MutexBmpDataHandle);

		DelayTick += 10;
		osDelayUntil(DelayTick);

  }
  /* USER CODE END StartBMP280Task */
}

/* USER CODE BEGIN Header_StartOledTask */
/**
* @brief Function implementing the OledTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartOledTask */
void StartOledTask(void *argument)
{
  /* USER CODE BEGIN StartOledTask */
	uint8_t Message[32];
	uint16_t counter = 0;

	BmpData_t _BmpData;

	osMutexAcquire(MutexI2CHandle, osWaitForever);
	SSD1306_Init(&hi2c1);
	osMutexRelease(MutexI2CHandle);

	GFX_SetFont(font_8x5);

	SSD1306_Clear(BLACK);

	SSD1306_Display();

  /* Infinite loop */
  for(;;)
  {
		sprintf((char*) Message, "Weather parameter:%04d", counter++);
		GFX_DrawString(0, 0, (char*) Message, WHITE, BLACK);

//		osMutexAcquire(MutexBmpDataHandle, osWaitForever);
//		_Temperature = BmpData.Temperature;
//		_Pressure = BmpData.Pressure;
//		osMutexRelease(MutexBmpDataHandle);
		osMessageQueueGet(QueueBmpDataHandle, &_BmpData, 0, osWaitForever);

		sprintf((char*) Message, "Temperature: %.2f", _BmpData.Temperature);
		GFX_DrawString(0, 10, (char*) Message, WHITE, BLACK);

		sprintf((char*) Message, "Pressure: %.2f", _BmpData.Pressure);
		GFX_DrawString(0, 20, (char*) Message, WHITE, BLACK);

		SSD1306_Display();

		//osDelay(100);
  }
  /* USER CODE END StartOledTask */
}

/* TimerBmpDataCallback function */
void TimerBmpDataCallback(void *argument)
{
  /* USER CODE BEGIN TimerBmpDataCallback */
	osSemaphoreRelease(SemaphoreBmpQueueHandle);
  /* USER CODE END TimerBmpDataCallback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void _putchar(char Character)
{
	osMutexAcquire(MutexPrintfHandle, osWaitForever);
	HAL_UART_Transmit(&huart2, (uint8_t*)&Character, 1, 1000);
	osMutexRelease(MutexPrintfHandle);
}
/* USER CODE END Application */

