#ifndef __OLED_H
#define __OLED_H

#include <stdint.h>

#define OLED_TIMING_HISTORY_LENGTH 5U

/* 单个屏幕帧统计；按时间推进的动画每个实际处理帧分别记录。 */
typedef struct
{
	uint32_t draw_us;       /* 绘图、动画合成、差异比较及页缓存同步耗时 */
	uint32_t i2c_us;        /* I2C发送时间，DMA路径包含等待传输完成 */
	uint32_t frame_us;      /* 单帧绘制和发送总耗时，不含帧间动画延时 */
	uint32_t sent_bytes;    /* 成功发送字节数，含地址字节和SSD1306控制字节 */
	uint32_t dma_count;     /* DMA发送调用次数，包含启动失败的调用 */
	uint32_t failure_count; /* I2C发送失败次数 */
	uint32_t last_status;   /* 本帧状态，HAL_OK为0 */
	uint32_t frame_id;      /* 帧序号 */
} OLED_FrameTiming_t;

/* 直接字段保存最近一帧；history_write_index指向环形缓冲区下一个写入槽。 */
typedef struct
{
	uint32_t draw_us;
	uint32_t i2c_us;
	uint32_t frame_us;
	uint32_t sent_bytes;
	uint32_t dma_count;
	uint32_t failure_count;
	uint32_t last_status;
	uint32_t frame_count;
	uint32_t history_write_index;
	uint32_t timer_ready;   /* DWT周期计数器可用时为1 */
	OLED_FrameTiming_t history[OLED_TIMING_HISTORY_LENGTH];
} OLED_TimingStats_t;

extern volatile OLED_TimingStats_t OLED_TimingStats;


void OLED_Init(void);
void OLED_Clear(void);
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char);
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String);
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length);
void OLED_ShowSmoothSignedNum(uint8_t Line, uint8_t Column, float Number, uint8_t Length);
void OLED_ClearFeatureData(void);
void OLED_RenderControllerUI(uint8_t Root, uint8_t Submenu,
	uint8_t Level, uint8_t FeatureSelection, uint8_t ConnectionState,
	uint8_t LocalRole, uint8_t ButtonBitmap, uint8_t MwGyroState);
void OLED_UpdateFeatureValue(uint8_t Index, int32_t Value);
void OLED_UIProcess(void);
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_TakeMutex(void);
void OLED_GiveMutex(void);

extern 	xSemaphoreHandle xOLEDMutex;


#endif
