#ifndef __MYDEF_H
#define __MYDEF_H



#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "Math.h"
#include "stm32f1xx_hal.h"
#include "FreeRTOS.h"
#include "event_groups.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"
#include "main.h"
#include "cmsis_os.h"
#include "timers.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"



#include  "Delay.h"
#include "OLED.h"






#include "fun.h"
#include "TASK.h"
#include "FREQ2_4G.h"
#include "HLKAs201.h"


#define SPI_CSN(x)		HAL_GPIO_WritePin(SPI_CSN_GPIO_Port, SPI_CSN_Pin, (GPIO_PinState)(x))


typedef struct {

	/* 这是原2.4G控制帧的本机队列数据，不是USART3双手柄协议的线上结构体。 */
	uint8_t Data_1;	
	uint8_t Data_2;
	uint8_t Data_3;
	uint8_t Data_4;
	uint8_t Data_5;
	uint8_t Data_6;
	uint8_t Data_7;
	uint8_t Data_8;
	uint8_t Data_9;
	uint8_t Data_10;
	uint8_t Data_11;
	uint8_t Data_12;
	

	}FREQ2_4GQueueHandle_Data;



extern TaskHandle_t xOLEDTask_Handle;
extern TaskHandle_t xKEYTask_Handle;

/* 传感器输入改由TASK快照API共享；此队列仅承载待发送的无线控制命令。 */
extern xQueueHandle FREQ2_4G_DataQueueHandle;
	
extern EventGroupHandle_t KEY_StartEventGroupHandle;	
	
extern TimerHandle_t KEY_LOCKTimerHandle;
	
extern signed char PCWriteBUFF[];

extern FREQ2_4GQueueHandle_Data TX_Data;




#endif
