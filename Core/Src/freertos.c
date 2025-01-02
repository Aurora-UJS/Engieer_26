/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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

// #include "DBusSys.h"
// #include "dma.h"
// #include "usart.h"
// #include "PIDtool.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticSemaphore_t osStaticSemaphoreDef_t;
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

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for IMU_TempCtrl */
osThreadId_t IMU_TempCtrlHandle;
const osThreadAttr_t IMU_TempCtrl_attributes = {
  .name = "IMU_TempCtrl",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for imuBinarySem01 */
osSemaphoreId_t imuBinarySem01Handle;
osStaticSemaphoreDef_t imuBinarySemControlBlock;
const osSemaphoreAttr_t imuBinarySem01_attributes = {
  .name = "imuBinarySem01",
  .cb_mem = &imuBinarySemControlBlock,
  .cb_size = sizeof(imuBinarySemControlBlock),
};
/* Definitions for controlBinaryIMU */
osSemaphoreId_t controlBinaryIMUHandle;
osStaticSemaphoreDef_t controlBinaryIMUControlBlock;
const osSemaphoreAttr_t controlBinaryIMU_attributes = {
  .name = "controlBinaryIMU",
  .cb_mem = &controlBinaryIMUControlBlock,
  .cb_size = sizeof(controlBinaryIMUControlBlock),
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void IMU_TempCtrlTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of imuBinarySem01 */
  imuBinarySem01Handle = osSemaphoreNew(1, 0, &imuBinarySem01_attributes);

  /* creation of controlBinaryIMU */
  controlBinaryIMUHandle = osSemaphoreNew(1, 0, &controlBinaryIMU_attributes);

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of IMU_TempCtrl */
  IMU_TempCtrlHandle = osThreadNew(IMU_TempCtrlTask, NULL, &IMU_TempCtrl_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  UNUSED(argument);
  /* Infinite loop */
  for (;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_IMU_TempCtrlTask */
/**
* @brief Function implementing the IMU_TempCtrl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_IMU_TempCtrlTask */
__weak void IMU_TempCtrlTask(void *argument)
{
  /* USER CODE BEGIN IMU_TempCtrlTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END IMU_TempCtrlTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

