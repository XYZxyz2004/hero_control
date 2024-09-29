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
#include "shoot.h"
#include "tripod_head.h"
#include "task_init.h"
#include "chassis_m.h"
#include "bsp_motor.h"
#include "bsp_can.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
osThreadId startTaskHandle;
osThreadId tripod_headTaskHandle;
osThreadId chassisTaskHandle;
osThreadId shootTaskHandle;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/*void can_send_control(void const*argument)
{
	while(1)
	{
	CAN_Send_Data(&hcan1,0x1ff,CAN1_0x1ff_Tx_Data,8);
			osDelay(3);
	}
	
}*/
void shoot_task_control(void const*argument)
{
	
	 
	while(1)
	{shoot_task(&shoot_control);
		osDelay(2);
	}
}

void tripod_task_control(void const*argument)
{

	while(1)
	{tripod_task(&tripod_head_control);
osDelay(2);
	}
}

void chassis_task_control(void const*argument)
{  
	while(1)
	{chassism_task(&chassism_control);
	osDelay(2);
	}
}

void Start_Task(void const*argument)
{
		taskENTER_CRITICAL();	
	
	
	osThreadDef(shootTask,shoot_task_control, osPriorityNormal, 0, 128);
  shootTaskHandle = osThreadCreate(osThread(shootTask), NULL);
	
	 osThreadDef(tripod_headTask,tripod_task_control, osPriorityNormal, 0, 128);
  tripod_headTaskHandle = osThreadCreate(osThread(tripod_headTask), NULL);
	
	osThreadDef(chassisTask,chassis_task_control, osPriorityNormal, 0, 128);
  chassisTaskHandle  = osThreadCreate(osThread(chassisTask), NULL);
	
//osThreadDef(cansendTask,can_send_control, osPriorityNormal, 0, 128);
  //can_sendTaskHandle  = osThreadCreate(osThread(cansendTask), NULL);
	
	osThreadTerminate(startTaskHandle);
	taskEXIT_CRITICAL();

}
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

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
   osThreadDef(startTask, Start_Task, osPriorityNormal, 0, 128);
  startTaskHandle = osThreadCreate(osThread(startTask), NULL);
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
