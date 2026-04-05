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
#include <string.h>

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
/* Definitions for SerialPlot */
osThreadId_t SerialPlotHandle;
uint32_t SerialPlotBuffer[512];
osStaticThreadDef_t SerialPlotControlBlock;
const osThreadAttr_t SerialPlot_attributes = {
    .name       = "SerialPlot",
    .cb_mem     = &SerialPlotControlBlock,
    .cb_size    = sizeof(SerialPlotControlBlock),
    .stack_mem  = &SerialPlotBuffer[0],
    .stack_size = sizeof(SerialPlotBuffer),
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

 /* Definitions for Arm_State_Machine_Task */
 osThreadId_t Arm_State_Machine_TaskHandle;
 uint32_t Arm_State_Machine_TaskBuffer[512];
 osStaticThreadDef_t Arm_State_Machine_TaskControlBlock;
 const osThreadAttr_t Arm_State_Machine_Task_attributes = {
     .name = "Arm_State_Machine_Task",
     .cb_mem = &Arm_State_Machine_TaskControlBlock,
     .cb_size = sizeof(Arm_State_Machine_TaskControlBlock),
     .stack_mem = &Arm_State_Machine_TaskBuffer[0],
     .stack_size = sizeof(Arm_State_Machine_TaskBuffer),
     .priority = (osPriority_t) osPriorityNormal,
 };
 /* Definitions for SerialPort */
 osThreadId_t SerialPortHandle;
 uint32_t SerialPortBuffer[256];
 osStaticThreadDef_t SerialPortControlBlock;
 const osThreadAttr_t SerialPort_attributes = {
     .name = "SerialPort",
     .cb_mem = &SerialPortControlBlock,
     .cb_size = sizeof(SerialPortControlBlock),
     .stack_mem = &SerialPortBuffer[0],
     .stack_size = sizeof(SerialPortBuffer),
     .priority = (osPriority_t) osPriorityNormal,
 };
uint32_t Chassis_TaskBuffer[512];  // 栈大小：128 * 4字节 = 512字节
osStaticThreadDef_t Chassis_TaskControlBlock;  // 静态任务控制块
osThreadId_t Chassis_TaskHandle;  // 任务句柄
const osThreadAttr_t Chassis_Task_attributes = {
  .name = "Chassis_Task",        // 任务名称（调试用）
  .cb_mem = &Chassis_TaskControlBlock,  // 控制块地址（静态创建）
  .cb_size = sizeof(Chassis_TaskControlBlock),  // 控制块大小
  .stack_mem = &Chassis_TaskBuffer[0],  // 栈空间地址
  .stack_size = sizeof(Chassis_TaskBuffer),  // 栈大小
  .priority = (osPriority_t)osPriorityNormal,  // 优先级（与默认任务相同）
};

uint32_t Referee_TaskBuffer[1024];  // 栈大小：1024 * 4字节 = 4096字节
osStaticThreadDef_t Referee_TaskControlBlock;  // 静态任务控制块
osThreadId_t Referee_TaskHandle;  // 任务句柄
const osThreadAttr_t Referee_Task_attributes = {
  .name = "Referee_Task",        // 任务名称（调试用）
  .cb_mem = &Referee_TaskControlBlock,  // 控制块地址（静态创建）
  .cb_size = sizeof(Referee_TaskControlBlock),  // 控制块大小
  .stack_mem = &Referee_TaskBuffer[0],  // 栈空间地址
  .stack_size = sizeof(Referee_TaskBuffer),  // 栈大小
  .priority = (osPriority_t)osPriorityNormal,  // 优先级（与默认任务相同）
};

uint32_t Watchdog_TaskBuffer[512];
osStaticThreadDef_t Watchdog_TaskControlBlock;
osThreadId_t Watchdog_TaskHandle;
const osThreadAttr_t Watchdog_Task_attributes = {
  .name = "Watchdog_Task",
  .cb_mem = &Watchdog_TaskControlBlock,
  .cb_size = sizeof(Watchdog_TaskControlBlock),
  .stack_mem = &Watchdog_TaskBuffer[0],
  .stack_size = sizeof(Watchdog_TaskBuffer),
  .priority = (osPriority_t)osPriorityLow,
};

uint32_t IMU_TaskBuffer[512];
osStaticThreadDef_t IMU_TaskControlBlock;
osThreadId_t IMU_TaskHandle;
const osThreadAttr_t IMU_Task_attributes = {
  .name = "IMU_Task",
  .cb_mem = &IMU_TaskControlBlock,
  .cb_size = sizeof(IMU_TaskControlBlock),
  .stack_mem = &IMU_TaskBuffer[0],
  .stack_size = sizeof(IMU_TaskBuffer),
  .priority = (osPriority_t)osPriorityLow,
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

 /* Definitions for Trajectory_Publisher */
 osThreadId_t Trajectory_PublisherHandle;
 uint32_t Trajectory_PublisherBuffer[256];
 osStaticThreadDef_t Trajectory_PublisherControlBlock;
 const osThreadAttr_t Trajectory_Publisher_attributes = {
     .name = "Trajectory_Publisher",
     .cb_mem = &Trajectory_PublisherControlBlock,
     .cb_size = sizeof(Trajectory_PublisherControlBlock),
     .stack_mem = &Trajectory_PublisherBuffer[0],
     .stack_size = sizeof(Trajectory_PublisherBuffer),
     .priority = (osPriority_t) osPriorityNormal,
 };
//  /* Definitions for test_CAN3_ARM */
//  osThreadId_t test_CAN3_ARMHandle;
//  uint32_t test_CAN3_ARMBuffer[256];
//  osStaticThreadDef_t test_CAN3_ARMControlBlock;
//  const osThreadAttr_t test_CAN3_ARM_attributes = {
//      .name = "test_CAN3_ARM",
//      .cb_mem = &test_CAN3_ARMControlBlock,
//      .cb_size = sizeof(test_CAN3_ARMControlBlock),
//      .stack_mem = &test_CAN3_ARMBuffer[0],
//      .stack_size = sizeof(test_CAN3_ARMBuffer),
//      .priority = (osPriority_t) osPriorityNormal,
//  };

 /* Definitions for Joint1_Move_Task */
 osThreadId_t Joint1_Move_TaskHandle;
 uint32_t Joint1_Move_TaskBuffer[128];
 osStaticThreadDef_t Joint1_Move_TaskControlBlock;
 const osThreadAttr_t Joint1_Move_Task_attributes = {
     .name = "Joint1_Move_Task",
     .cb_mem = &Joint1_Move_TaskControlBlock,
     .cb_size = sizeof(Joint1_Move_TaskControlBlock),
     .stack_mem = &Joint1_Move_TaskBuffer[0],
     .stack_size = sizeof(Joint1_Move_TaskBuffer),
     .priority = (osPriority_t) osPriorityNormal,
 };
 /* Definitions for Joint2_Move_Task  */
 osThreadId_t Joint2_Move_TaskHandle;
 uint32_t Joint2_Move_TaskBuffer[128];
 osStaticThreadDef_t Joint2_Move_TaskControlBlock;
 const osThreadAttr_t Joint2_Move_Task_attributes = {
     .name = "Joint2_Move_Task ",
     .cb_mem = &Joint2_Move_TaskControlBlock,
     .cb_size = sizeof(Joint2_Move_TaskControlBlock),
     .stack_mem = &Joint2_Move_TaskBuffer[0],
     .stack_size = sizeof(Joint2_Move_TaskBuffer),
     .priority = (osPriority_t) osPriorityNormal,
 };
 /* Definitions for Joint3_Move_Task */
 osThreadId_t Joint3_Move_TaskHandle;
 uint32_t Joint3_Move_TaskBuffer[128];
 osStaticThreadDef_t Joint3_Move_TaskControlBlock;
 const osThreadAttr_t Joint3_Move_Task_attributes = {
     .name = "Joint3_Move_Task",
     .cb_mem = &Joint3_Move_TaskControlBlock,
     .cb_size = sizeof(Joint3_Move_TaskControlBlock),
     .stack_mem = &Joint3_Move_TaskBuffer[0],
     .stack_size = sizeof(Joint3_Move_TaskBuffer),
     .priority = (osPriority_t) osPriorityNormal,
 };
 /* Definitions for Joint4_Move_Task */
 osThreadId_t Joint4_Move_TaskHandle;
 uint32_t Joint4_Move_TaskBuffer[128];
 osStaticThreadDef_t Joint4_Move_TaskControlBlock;
 const osThreadAttr_t Joint4_Move_Task_attributes = {
     .name = "Joint4_Move_Task",
     .cb_mem = &Joint4_Move_TaskControlBlock,
     .cb_size = sizeof(Joint4_Move_TaskControlBlock),
     .stack_mem = &Joint4_Move_TaskBuffer[0],
     .stack_size = sizeof(Joint4_Move_TaskBuffer),
     .priority = (osPriority_t) osPriorityNormal,
 };
 /* Definitions for Joint5_Move_Task */
 osThreadId_t Joint5_Move_TaskHandle;
 uint32_t Joint5_Move_TaskBuffer[128];
 osStaticThreadDef_t Joint5_Move_TaskControlBlock;
 const osThreadAttr_t Joint5_Move_Task_attributes = {
     .name = "Joint5_Move_Task",
     .cb_mem = &Joint5_Move_TaskControlBlock,
     .cb_size = sizeof(Joint5_Move_TaskControlBlock),
     .stack_mem = &Joint5_Move_TaskBuffer[0],
     .stack_size = sizeof(Joint5_Move_TaskBuffer),
     .priority = (osPriority_t) osPriorityAboveNormal,
 };
 /* Definitions for Joint6_Move_Task */
 osThreadId_t Joint6_Move_TaskHandle;
 uint32_t Joint6_Move_TaskBuffer[128];
 osStaticThreadDef_t Joint6_Move_TaskControlBlock;
 const osThreadAttr_t Joint6_Move_Task_attributes = {
     .name = "Joint6_Move_Task",
     .cb_mem = &Joint6_Move_TaskControlBlock,
     .cb_size = sizeof(Joint6_Move_TaskControlBlock),
     .stack_mem = &Joint6_Move_TaskBuffer[0],
     .stack_size = sizeof(Joint6_Move_TaskBuffer),
     .priority = (osPriority_t) osPriorityAboveNormal,
 };

 /* Definitions for View_Gimbal_Task */
 osThreadId_t View_Gimbal_TaskHandle;
 uint32_t View_Gimbal_TaskBuffer[128];
 osStaticThreadDef_t View_Gimbal_TaskControlBlock;
 const osThreadAttr_t View_Gimbal_Task_attributes = {
     .name = "View_Gimbal_Task",
     .cb_mem = &View_Gimbal_TaskControlBlock,
     .cb_size = sizeof(View_Gimbal_TaskControlBlock),
     .stack_mem = &View_Gimbal_TaskBuffer[0],
     .stack_size = sizeof(View_Gimbal_TaskBuffer),
     .priority = (osPriority_t) osPriorityNormal,
 };
/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void uart_test(void *argument);
void motor_test(void *arguments);
void jointFollowAngle(void *arguments);
void uart_Transmit_Angle(void *arguments);
void Chassis_Task(void *arguments);
void Referee_Task(void *arguments);
void Watchdog_Task(void *arguments);
void SerialPlot(void *argument);
void IMU_Task(void *argument);
// void test_CAN3_ARM(void *argument);
void Joint1_Move_Task(void *argument);
void Joint2_Move_Task(void *argument);
void Joint3_Move_Task(void *argument);
void Joint4_Move_Task(void *argument);
void Joint5_Move_Task(void *argument);
void Joint6_Move_Task(void *argument);
/* USER CODE END FunctionPrototypes */

/* USER CODE END FunctionPrototypes */
void View_Gimbal_Task(void *argument);
void Trajectory_Timer_Init(void);
void StartDefaultTask(void *argument);
void IMU_TempCtrlTask(void *argument);
void Remoter_Task(void *argument);
void Trajectory_Publisher_Task(void *argument) ;

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  Trajectory_Timer_Init();
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

  // Joint1_Move_TaskHandle = osThreadNew(Joint1_Move_Task, NULL, &Joint1_Move_Task_attributes);
  // Joint2_Move_TaskHandle = osThreadNew(Joint2_Move_Task, NULL, &Joint2_Move_Task_attributes);
  // Joint3_Move_TaskHandle = osThreadNew(Joint3_Move_Task, NULL, &Joint3_Move_Task_attributes);
  // Joint4_Move_TaskHandle = osThreadNew(Joint4_Move_Task, NULL, &Joint4_Move_Task_attributes);
  // Joint5_Move_TaskHandle = osThreadNew(Joint5_Move_Task, NULL, &Joint5_Move_Task_attributes);
  // Joint6_Move_TaskHandle = osThreadNew(Joint6_Move_Task, NULL, &Joint6_Move_Task_attributes);
  //uart_Transmit_AngleHandle = osThreadNew(uart_Transmit_Angle, NULL, &uart_Transmit_Angle_attributes);
  Chassis_TaskHandle = osThreadNew(Chassis_Task, NULL, &Chassis_Task_attributes);
  // SerialPortHandle = osThreadNew(SerialPlot, NULL, &SerialPort_attributes);
  Referee_TaskHandle = osThreadNew(Referee_Task, NULL, &Referee_Task_attributes);
  IMU_TaskHandle = osThreadNew(IMU_Task, NULL, &IMU_Task_attributes);
  Trajectory_PublisherHandle = osThreadNew(Trajectory_Publisher_Task, NULL, &Trajectory_Publisher_attributes);
  View_Gimbal_TaskHandle = osThreadNew(View_Gimbal_Task, NULL, &View_Gimbal_Task_attributes);
  // SerialPlotHandle = osThreadNew(SerialPlot, NULL, &SerialPlot_attributes);
  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  Watchdog_TaskHandle = osThreadNew(Watchdog_Task, NULL, &Watchdog_Task_attributes);
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

__weak void Chassis_Task(void *argument)
{
  /* USER CODE Chassis_Task */
  UNUSED(argument);
  /* Infinite loop */
  for(;;)
  {
  
    osDelay(2);
  }
  /* USER CODE END Chassis_Task */
}

__weak void Referee_Task(void *argument)
{
  UNUSED(argument);
  for(;;)
  {
    
    osDelay(1);
  }
}

__weak void SerialPort(void *argument)
{
  UNUSED(argument);
  for(;;)
  {
    
    osDelay(1);
  }
}

__weak void Trajectory_Publisher_Task (void *argument){
  UNUSED(argument);
  for(;;){
    osDelay(1);
  }
}

__weak void IMU_Task(void *argument)
{
  UNUSED(argument);
  for(;;){
    osDelay(1);
  }
}
/* USER CODE END Application */


__weak void SerialPlot(void *argument)
{
  UNUSED(argument);
  for(;;)
  {
    
    osDelay(1);
  }
}
