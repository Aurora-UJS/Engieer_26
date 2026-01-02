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
#include "cmsis_os2.h"
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
typedef StaticTask_t osStaticThreadDef_t;
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
uint32_t IMU_TempCtrlBuffer[ 128 ];
osStaticThreadDef_t IMU_TempCtrlControlBlock;
const osThreadAttr_t IMU_TempCtrl_attributes = {
  .name = "IMU_TempCtrl",
  .cb_mem = &IMU_TempCtrlControlBlock,
  .cb_size = sizeof(IMU_TempCtrlControlBlock),
  .stack_mem = &IMU_TempCtrlBuffer[0],
  .stack_size = sizeof(IMU_TempCtrlBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Remoter */
osThreadId_t RemoterHandle;
uint32_t RemoterBuffer[ 256 ];
osStaticThreadDef_t RemoterControlBlock;
const osThreadAttr_t Remoter_attributes = {
  .name = "Remoter",
  .cb_mem = &RemoterControlBlock,
  .cb_size = sizeof(RemoterControlBlock),
  .stack_mem = &RemoterBuffer[0],
  .stack_size = sizeof(RemoterBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for uartTest */
osThreadId_t uartTestHandle;
uint32_t uartTestBuffer[512];
osStaticThreadDef_t uartTestControlBlock;
const osThreadAttr_t uartTest_attributes = {
    .name       = "uartTest",
    .cb_mem     = &uartTestControlBlock,
    .cb_size    = sizeof(uartTestControlBlock),
    .stack_mem  = &uartTestBuffer[0],
    .stack_size = sizeof(uartTestBuffer),
    .priority   = (osPriority_t)osPriorityNormal,
};
/* Definitions for motor_test */
osThreadId_t motorTestHandle;
uint32_t motorTestBuffer[512];
osStaticThreadDef_t motorTestControlBlock;

const osThreadAttr_t motorTest_attributes = {
    .name       = "motorTest",
    .cb_mem     = &motorTestControlBlock,
    .cb_size    = sizeof(motorTestControlBlock),
    .stack_mem  = &motorTestBuffer[0],
    .stack_size = sizeof(motorTestBuffer),
    .priority   = (osPriority_t)osPriorityNormal,
};

/* Definitions for jointFollowAngle */

osThreadId_t jointFollowAngleHandle;
uint32_t jointFollowAngleBuffer[512];
osStaticThreadDef_t jointFollowAngleControlBlock;

const osThreadAttr_t jointFollowAngle_attributes = {
    .name       = "jointFollowAngle",
    .cb_mem     = &jointFollowAngleControlBlock,
    .cb_size    = sizeof(jointFollowAngleControlBlock),
    .stack_mem  = &jointFollowAngleBuffer[0],
    .stack_size = sizeof(jointFollowAngleBuffer),
    .priority   = (osPriority_t)osPriorityNormal,
};

/* Definitions for uart_Transmit_Angle */
osThreadId_t uart_Transmit_AngleHandle;
uint32_t uart_Transmit_AngleBuffer[1024];
osStaticThreadDef_t uart_Transmit_AngleControlBlock;
const osThreadAttr_t uart_Transmit_Angle_attributes = {
    .name       = "uart_Transmit_Angle",
    .cb_mem     = &uart_Transmit_AngleControlBlock,
    .cb_size    = sizeof(uart_Transmit_AngleControlBlock),
    .stack_mem  = &uart_Transmit_AngleBuffer[0],
    .stack_size = sizeof(uart_Transmit_AngleBuffer),
    .priority   = (osPriority_t)osPriorityNormal,
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
void uart_test(void *argument);
void motor_test(void *arguments);
void jointFollowAngle(void *arguments);
void uart_Transmit_Angle(void *arguments);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void IMU_TempCtrlTask(void *argument);
void Remoter_Task(void *argument);

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

  /* creation of Remoter */
  RemoterHandle = osThreadNew(Remoter_Task, NULL, &Remoter_attributes);

  
  // uartTestHandle = osThreadNew(uart_test, NULL, &uartTest_attributes);
  // motorTestHandle = osThreadNew(motor_test, NULL,&motorTest_attributes);
  jointFollowAngleHandle = osThreadNew(jointFollowAngle,NULL, &jointFollowAngle_attributes);
  uart_Transmit_AngleHandle = osThreadNew(uart_Transmit_Angle, NULL, &uart_Transmit_Angle_attributes);
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
  UNUSED(argument);
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END IMU_TempCtrlTask */
}

/* USER CODE BEGIN Header_Remoter_Task */
/**
* @brief Function implementing the Remoter thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Remoter_Task */
__weak void Remoter_Task(void *argument)
{
  /* USER CODE BEGIN Remoter_Task */
  UNUSED(argument);
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Remoter_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

__weak void uart_test(void *argument)
{
  UNUSED(argument);
  for (;;) {
    osDelay(1);
  }
}
__weak void motor_test(void *argument)
{
  UNUSED(argument);
  for (;;) {
    osDelay(1);
  }
}
__weak void jointFollowAngle(void *argument)
{
  UNUSED(argument);
  for (;;) {
    osDelay(1);
  }
}
__weak void uart_Transmit_Angle(void *argument)
{
  UNUSED(argument);
  for (;;) {
    osDelay(1);
  }
}
/* USER CODE END Application */

