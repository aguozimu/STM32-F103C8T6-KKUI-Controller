/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "Mydef.h"
#include "controller_config.h"
#include "handset_link.h"
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
TaskHandle_t xOLEDTask_Handle;
volatile UBaseType_t gOLEDTaskStackHighWaterWords;
volatile UBaseType_t gKEYTaskStackHighWaterWords;
volatile UBaseType_t gCTRLTaskStackHighWaterWords;
volatile UBaseType_t gFREQTaskStackHighWaterWords;
volatile UBaseType_t gJSTaskStackHighWaterWords;
volatile UBaseType_t gGYROTaskStackHighWaterWords;
volatile UBaseType_t gHandsetLinkTaskStackHighWaterWords;
volatile UBaseType_t gTimerServiceStackHighWaterWords;
volatile size_t gFreeHeapBytes;
volatile size_t gMinimumFreeHeapBytes;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
TaskHandle_t xKEYTask_Handle;
TaskHandle_t xCTRLTask_Handle;
TaskHandle_t xFREQ2_4GTask_Handle;
TaskHandle_t xJSTask_Handle;
TaskHandle_t xGYROTask_Handle;
TaskHandle_t xHandsetLinkTask_Handle;

xQueueHandle FREQ2_4G_DataQueueHandle;

EventGroupHandle_t KEY_StartEventGroupHandle;

TimerHandle_t KEY_LOCKTimerHandle;

static void TASK_UpdateStackHighWaterMarks(void);


signed char PCWriteBUFF[200];

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
	/* 调度器启动前初始化OLED并清屏；初始化阶段使用阻塞I2C发送。 */
	OLED_Init();

	
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */

	
	KEY_LOCKTimerHandle=xTimerCreate(
    "MyTimer",          // 定时器的名字，仅用于调试，随便取
		20,// 单次定时周期：20ms（系统节拍为1kHz）
    pdFALSE,             // 定时器模式：pdTRUE=周期性定时器，pdFALSE=单次定时器
    (void *)0,          // 用户自定义ID，用来区分多个定时器，不需要就填NULL或0
    KEY_LOCKTimerFun // 定时器到期后要执行的回调函数
);
	if (KEY_LOCKTimerHandle == NULL)
	{
		Error_Handler();
	}
	
	
	
	
	
	
	
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
	
/* 单槽队列只保留最新无线命令，避免控制任务积压后发出过时动作。 */
/* 无线控制帧使用单槽最新值队列；摇杆和姿态通过短临界区快照共享。 */
FREQ2_4G_DataQueueHandle=xQueueCreate(1,sizeof(FREQ2_4GQueueHandle_Data));

KEY_StartEventGroupHandle= xEventGroupCreate();
	if (FREQ2_4G_DataQueueHandle == NULL ||
		KEY_StartEventGroupHandle == NULL)
	{
		Error_Handler();
	}
	
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  
  /* 控制/采样优先级高于UI；链路任务只处理串口，不执行控制计算。 */
  if (xTaskCreate(KEYTASK,"KEY_Driver",128,NULL,CONTROLLER_TASK_PRIORITY_KEY,&xKEYTask_Handle) != pdPASS ||
	  xTaskCreate(CTRLTASK,"CTRL_Driver",512,NULL,CONTROLLER_TASK_PRIORITY_CONTROL,&xCTRLTask_Handle) != pdPASS ||
	  xTaskCreate(FREQ2_4GTask,"FREQ2_4G_Driver",256,NULL,CONTROLLER_TASK_PRIORITY_24G,&xFREQ2_4GTask_Handle) != pdPASS ||
	  xTaskCreate(JSTask,"JS_Driver",256,NULL,CONTROLLER_TASK_PRIORITY_SAMPLE,&xJSTask_Handle) != pdPASS ||
	  xTaskCreate(GYROTask,"GYRO_Driver",256,NULL,CONTROLLER_TASK_PRIORITY_SAMPLE,&xGYROTask_Handle) != pdPASS ||
	  xTaskCreate(HandsetLink_Task,"HANDSET_LINK",CONTROLLER_LINK_TASK_STACK_WORDS,NULL,
			  CONTROLLER_TASK_PRIORITY_LINK,&xHandsetLinkTask_Handle) != pdPASS)
  {
	Error_Handler();
  }
  
  
  
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
  TickType_t lastStackCheck;
  /* 显示动画允许被控制和链路任务抢占；CMSIS 默认创建优先级仍为 24。 */
  vTaskPrioritySet(NULL, CONTROLLER_TASK_PRIORITY_OLED);
  /* Infinite loop */

	OLED_Ctrl();	
	xOLEDTask_Handle = xTaskGetCurrentTaskHandle();
	lastStackCheck = xTaskGetTickCount();
	for (;;)
	{
		/* 消费最新UI状态；页面层级变化时在此完成滑动动画和屏幕发送。 */
		OLED_UIProcess();
		if ((TickType_t)(xTaskGetTickCount() - lastStackCheck) >= pdMS_TO_TICKS(1000U))
		{
			lastStackCheck = xTaskGetTickCount();
			TASK_UpdateStackHighWaterMarks();
		}
		/* 没有待处理状态时，10ms后再检查一次。 */
		vTaskDelay(10U);
	}
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

static void TASK_UpdateStackHighWaterMarks(void)
{
	/* 这些计数只提供调试观察值，需在实际任务压力下确认栈/堆余量。 */
	/* FreeRTOS返回剩余栈空间，单位为StackType_t字数。 */
	gOLEDTaskStackHighWaterWords = uxTaskGetStackHighWaterMark(xOLEDTask_Handle);
	gKEYTaskStackHighWaterWords = uxTaskGetStackHighWaterMark(xKEYTask_Handle);
	gCTRLTaskStackHighWaterWords = uxTaskGetStackHighWaterMark(xCTRLTask_Handle);
	gFREQTaskStackHighWaterWords = uxTaskGetStackHighWaterMark(xFREQ2_4GTask_Handle);
	gJSTaskStackHighWaterWords = uxTaskGetStackHighWaterMark(xJSTask_Handle);
	gGYROTaskStackHighWaterWords = uxTaskGetStackHighWaterMark(xGYROTask_Handle);
	gHandsetLinkTaskStackHighWaterWords = uxTaskGetStackHighWaterMark(xHandsetLinkTask_Handle);
	{
		TaskHandle_t timerTaskHandle = xTimerGetTimerDaemonTaskHandle();
		if (timerTaskHandle != NULL)
		{
			gTimerServiceStackHighWaterWords = uxTaskGetStackHighWaterMark(timerTaskHandle);
		}
	}
	gFreeHeapBytes = xPortGetFreeHeapSize();
	gMinimumFreeHeapBytes = xPortGetMinimumEverFreeHeapSize();
}

void vApplicationStackOverflowHook(TaskHandle_t taskHandle, char *taskName)
{
	(void)taskHandle;
	(void)taskName;
	taskDISABLE_INTERRUPTS();
	for (;;)
	{
	}
}

void vApplicationMallocFailedHook(void)
{
	taskDISABLE_INTERRUPTS();
	for (;;)
	{
	}
}

/* USER CODE END Application */

