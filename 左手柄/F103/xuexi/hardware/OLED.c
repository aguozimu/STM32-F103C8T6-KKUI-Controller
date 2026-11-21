#include "OLED_Font.h"
#include "stm32f1xx_hal.h"
#include "Delay.h"
#include "Mydef.h"
#include "OLED.h"
#include "i2c.h"
#include "controller_config.h"
#include "controller_types.h"



#define OLED_WIDTH 128U
#define OLED_HEIGHT 64U
#define OLED_PAGE_COUNT 8U
#define OLED_FRAMEBUFFER_SIZE 1024U
#define OLED_UI_VALUE_COUNT 4U
#define OLED_UI_VALUE_PERIOD_MS 40U
/* 菜单选项滚动与左右页面滑动分别配置；动画进度都计入绘制和传输耗时。 */
#define OLED_MENU_SCROLL_DURATION_US 150000UL
#define OLED_PAGE_TRANSITION_DURATION_US 150000UL
/* 同级选项滚动结束后的小幅回弹；仅影响菜单正文。 */
	#define OLED_MENU_BOUNCE_DURATION_US 100000UL
	#define OLED_MENU_BOUNCE_MAX_PIXELS 1U
/* 左右页面滑动独立回弹参数，幅度按横向128像素范围设置。 */
#define OLED_PAGE_BOUNCE_DURATION_US 100000UL
#define OLED_PAGE_BOUNCE_MAX_PIXELS 2U
#define OLED_I2C_ADDRESS 0x78U
#define OLED_I2C_TRANSFER_TIMEOUT_MS 100U
/* 小于16字节的数据直接发送，避免短事务的DMA中断和任务唤醒开销。 */
#define OLED_I2C_DMA_MIN_DATA_BYTES 16U

#define OLED_UI_SCREEN_EMPTY 0U
#define OLED_UI_SCREEN_MAIN 1U
#define OLED_UI_SCREEN_MENU 2U
#define OLED_UI_SCREEN_FEATURE 3U
#define OLED_UI_SCREEN_LINK 4U
#define OLED_UI_DIRTY_SCREEN 0x01U
#define OLED_UI_DIRTY_VALUES 0x02U

typedef struct
{
	uint8_t Screen;
	uint8_t Root;
	uint8_t Submenu;
	uint8_t Selection;
	uint8_t Level;
	uint8_t ConnectionState;
	uint8_t LocalRole;
	uint8_t ButtonBitmap;
	uint8_t MwGyroState; /* 0=普通页，1=关闭，2=摇杆直控，3=设速，4=锁定恒速 */
	int32_t Values[OLED_UI_VALUE_COUNT];
} OLED_UIState_t;

	xSemaphoreHandle xOLEDMutex;
static SemaphoreHandle_t xOLEDTransferSemaphore;

/* 当前屏幕缓存：128x64像素，按8页保存，每页对应128字节。 */
static uint8_t OLED_FrameBuffer[OLED_FRAMEBUFFER_SIZE];
/* 两个单页缓冲分别保存合成后的过渡页和目标页。 */
static uint8_t OLED_NextPage[OLED_WIDTH];
static uint8_t OLED_TargetPage[OLED_WIDTH];
static uint8_t OLED_I2CTransferBuffer[OLED_WIDTH + 1U];
static const uint8_t OLED_ClearLine[OLED_WIDTH] = {0U};
static uint8_t OLED_RenderPageIndex = 0U;
/* RequestedUI是最新菜单状态；RenderedUI是目前已经显示的状态。 */
static OLED_UIState_t OLED_RequestedUI;
static OLED_UIState_t OLED_RenderedUI;
static volatile uint8_t OLED_UIDirtyFlags = OLED_UI_DIRTY_SCREEN;
static uint8_t OLED_UIInitialized = 0U;
static uint32_t OLED_LastValueRenderTick = 0U;
static volatile HAL_StatusTypeDef OLED_I2CAsyncStatus = HAL_OK;
static volatile uint8_t OLED_DMAEnabled = 0U;

volatile OLED_TimingStats_t OLED_TimingStats;
static uint8_t OLED_TimingAvailable = 0U;
static uint8_t OLED_FrameTimingActive = 0U;
static uint32_t OLED_FrameDrawCycles = 0U;
static uint32_t OLED_FrameI2CCycles = 0U;
static uint32_t OLED_FrameSentBytes = 0U;
static uint32_t OLED_FrameDMACount = 0U;
static uint32_t OLED_FrameFailureCount = 0U;

void OLED_SetCursor(uint8_t Y, uint8_t X);
	

static uint32_t OLED_TimingReadCycles(void)
{
	return OLED_TimingAvailable != 0U ? DWT->CYCCNT : 0U;
}

static uint32_t OLED_TimingCyclesToUs(uint32_t cycles)
{
	if (OLED_TimingAvailable == 0U || SystemCoreClock == 0U)
	{
		return 0U;
	}
	return (uint32_t)(((uint64_t)cycles * 1000000ULL) / SystemCoreClock);
}

static void OLED_TimingAddDrawCycles(uint32_t startCycles)
{
	if (OLED_FrameTimingActive != 0U && OLED_TimingAvailable != 0U)
	{
		OLED_FrameDrawCycles += (uint32_t)(DWT->CYCCNT - startCycles);
	}
}

static void OLED_TimingBeginFrame(uint32_t *startCycles)
{
	OLED_FrameDrawCycles = 0U;
	OLED_FrameI2CCycles = 0U;
	OLED_FrameSentBytes = 0U;
	OLED_FrameDMACount = 0U;
	OLED_FrameFailureCount = 0U;
	*startCycles = OLED_TimingReadCycles();
	OLED_FrameTimingActive = 1U;
}

static void OLED_TimingFinishFrame(uint32_t startCycles,
	HAL_StatusTypeDef status)
{
	uint32_t frameCycles = (uint32_t)(OLED_TimingReadCycles() - startCycles);
	uint32_t historyIndex = OLED_TimingStats.history_write_index;
	uint32_t frameId = OLED_TimingStats.frame_count + 1U;
	uint32_t drawUs = OLED_TimingCyclesToUs(OLED_FrameDrawCycles);
	uint32_t i2cUs = OLED_TimingCyclesToUs(OLED_FrameI2CCycles);
	uint32_t frameUs = OLED_TimingCyclesToUs(frameCycles);

	OLED_FrameTimingActive = 0U;
	OLED_TimingStats.draw_us = drawUs;
	OLED_TimingStats.i2c_us = i2cUs;
	OLED_TimingStats.frame_us = frameUs;
	OLED_TimingStats.sent_bytes = OLED_FrameSentBytes;
	OLED_TimingStats.dma_count = OLED_FrameDMACount;
	OLED_TimingStats.failure_count = OLED_FrameFailureCount;
	OLED_TimingStats.last_status = (uint32_t)status;
	OLED_TimingStats.history[historyIndex].draw_us = drawUs;
	OLED_TimingStats.history[historyIndex].i2c_us = i2cUs;
	OLED_TimingStats.history[historyIndex].frame_us = frameUs;
	OLED_TimingStats.history[historyIndex].sent_bytes = OLED_FrameSentBytes;
	OLED_TimingStats.history[historyIndex].dma_count = OLED_FrameDMACount;
	OLED_TimingStats.history[historyIndex].failure_count = OLED_FrameFailureCount;
	OLED_TimingStats.history[historyIndex].last_status = (uint32_t)status;
	OLED_TimingStats.history[historyIndex].frame_id = frameId;
	OLED_TimingStats.history_write_index =
		(historyIndex + 1U) % OLED_TIMING_HISTORY_LENGTH;
	OLED_TimingStats.frame_count = frameId;
}

static void OLED_TimingInit(void)
{
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0U;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
	OLED_TimingAvailable =
		((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0U &&
		 SystemCoreClock >= 1000000U) ? 1U : 0U;
	OLED_TimingStats.timer_ready = OLED_TimingAvailable;
}

	
void OLED_TakeMutex(void){//获取互斥

xSemaphoreTake(xOLEDMutex,portMAX_DELAY);

}
	
void OLED_GiveMutex(void){//释放互斥

xSemaphoreGive(xOLEDMutex);

}


static HAL_StatusTypeDef OLED_I2C_Transmit(uint8_t ControlByte,
	const uint8_t *Data, uint8_t Length)
{
	HAL_StatusTypeDef status = HAL_ERROR;
	uint16_t transferLength = 0U;
	uint32_t transferStartCycles = 0U;
	uint8_t transferStarted = 0U;

	if (Length > OLED_WIDTH || (Length > 0U && Data == NULL))
	{
		goto finished;
	}

	if (OLED_DMAEnabled != 0U && xOLEDTransferSemaphore != NULL &&
		HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
	{
		status = HAL_BUSY;
		goto finished;
	}

	/* DMA运行期间保持发送缓冲区有效，完成回调后才允许复用。 */
	OLED_I2CTransferBuffer[0] = ControlByte;
	if (Length > 0U)
	{
		memcpy(&OLED_I2CTransferBuffer[1], Data, Length);
	}
	transferLength = (uint16_t)Length + 1U;

	if (OLED_DMAEnabled == 0U || xOLEDTransferSemaphore == NULL ||
		ControlByte == 0x00U || Length < OLED_I2C_DMA_MIN_DATA_BYTES)
	{
		/* 初始化、短命令和小块数据轮询发送；像素长块仍由DMA搬运。
		 * 不关闭中断、不暂停调度，控制任务仍可正常抢占。 */
		transferStartCycles = OLED_TimingReadCycles();
		transferStarted = 1U;
		status = HAL_I2C_Master_Transmit(&hi2c1, OLED_I2C_ADDRESS,
			OLED_I2CTransferBuffer, transferLength,
			OLED_I2C_TRANSFER_TIMEOUT_MS);
		goto finished;
	}

	(void)xSemaphoreTake(xOLEDTransferSemaphore, 0U);
	OLED_I2CAsyncStatus = HAL_BUSY;
	if (OLED_FrameTimingActive != 0U)
	{
		OLED_FrameDMACount++;
	}
	transferStartCycles = OLED_TimingReadCycles();
	transferStarted = 1U;
	status = HAL_I2C_Master_Transmit_DMA(&hi2c1, OLED_I2C_ADDRESS,
		OLED_I2CTransferBuffer, transferLength);
	if (status != HAL_OK)
	{
		goto finished;
	}

	if (xSemaphoreTake(xOLEDTransferSemaphore,
		pdMS_TO_TICKS(OLED_I2C_TRANSFER_TIMEOUT_MS)) != pdTRUE)
	{
		status = HAL_I2C_Master_Abort_IT(&hi2c1, OLED_I2C_ADDRESS);
		if (status == HAL_OK)
		{
			if (xSemaphoreTake(xOLEDTransferSemaphore,
				pdMS_TO_TICKS(OLED_I2C_TRANSFER_TIMEOUT_MS)) != pdTRUE)
			{
				status = HAL_TIMEOUT;
				goto finished;
			}
		}
		else
		{
			goto finished;
		}
	}

	status = OLED_I2CAsyncStatus;

finished:
	if (OLED_FrameTimingActive != 0U)
	{
		if (transferStarted != 0U)
		{
			OLED_FrameI2CCycles += (uint32_t)(
				OLED_TimingReadCycles() - transferStartCycles);
			if (status == HAL_OK)
			{
				/* 每次成功传输含1个地址字节、控制字节及有效载荷。 */
				OLED_FrameSentBytes += (uint32_t)transferLength + 1U;
			}
		}
		if (status != HAL_OK)
		{
			OLED_FrameFailureCount++;
		}
	}
	return status;
}

static void OLED_NotifyI2CTransferFromISR(I2C_HandleTypeDef *hi2c,
	HAL_StatusTypeDef status)
{
	BaseType_t higherPriorityTaskWoken = pdFALSE;

	if (hi2c == NULL || hi2c->Instance != I2C1 ||
		OLED_DMAEnabled == 0U || xOLEDTransferSemaphore == NULL)
	{
		return;
	}

	OLED_I2CAsyncStatus = status;
	(void)xSemaphoreGiveFromISR(xOLEDTransferSemaphore,
		&higherPriorityTaskWoken);
	portYIELD_FROM_ISR(higherPriorityTaskWoken);
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	OLED_NotifyI2CTransferFromISR(hi2c, HAL_OK);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
	OLED_NotifyI2CTransferFromISR(hi2c, HAL_ERROR);
}

void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef *hi2c)
{
	OLED_NotifyI2CTransferFromISR(hi2c, HAL_ERROR);
}
/**
  * @brief  OLED写命令
  * @param  Command 要写入的命令
  * @retval 无
  */
void OLED_WriteCommand(uint8_t Command)
{
	(void)OLED_I2C_Transmit(0x00U, &Command, 1U);
}

/**
  * @brief  OLED写数据
  * @param  Data 要写入的数据
  * @retval 无
  */
void OLED_WriteData(uint8_t Data)
{
	(void)OLED_I2C_Transmit(0x40U, &Data, 1U);
}

static HAL_StatusTypeDef OLED_WriteDataBuffer(const uint8_t *Data, uint8_t Length)
{
	return OLED_I2C_Transmit(0x40U, Data, Length);
}

static uint8_t OLED_IsFeatureDataCell(uint8_t Line, uint8_t Column)
{
	if (Line == 1U)
	{
		return (Column >= 8U && Column <= 10U) ? 1U : 0U;
	}

	if ((Line == 2U || Line == 3U) && Column >= 7U && Column <= 16U)
	{
		return 1U;
	}

	return 0U;
}

static uint8_t OLED_CanDrawFeatureData(void)
{
	task_menu_snapshot_t menuState;
	TASK_GetMenuSnapshot(&menuState);
	return (menuState.FloorSwitch == 2 &&
		(((menuState.Floor1 == 0 || menuState.Floor1 == 1) &&
		  (menuState.Floor2 == 0 || menuState.Floor2 == 3)) ||
		 (menuState.Floor1 == 2 && menuState.Floor2 == 0)) &&
		OLED_FeatureClearPending == 0U) ? 1U : 0U;
}

static void OLED_ResetFeatureSmoothFilters(void);

void OLED_ClearFeatureData(void)
{
	uint8_t i;

	taskENTER_CRITICAL();
	for (i = 0U; i < OLED_UI_VALUE_COUNT; i++)
	{
		OLED_RequestedUI.Values[i] = 0;
	}
	OLED_UIDirtyFlags |= OLED_UI_DIRTY_VALUES;
	taskEXIT_CRITICAL();
	OLED_ResetFeatureSmoothFilters();
}

/**
  * @brief  OLED设置光标位置
  * @param  Y 以左上角为原点，向下方向的坐标，范围：0~7
  * @param  X 以左上角为原点，向右方向的坐标，范围：0~127
  * @retval 无
  */
static HAL_StatusTypeDef OLED_SetCursorChecked(uint8_t Y, uint8_t X)
{
	uint8_t commands[3];

	/* SSD1306允许连续发送多条命令，合并为一次I2C传输。 */
	commands[0] = (uint8_t)(0xB0U | Y);
	commands[1] = (uint8_t)(0x10U | ((X & 0xF0U) >> 4U));
	commands[2] = (uint8_t)(0x00U | (X & 0x0FU));
	return OLED_I2C_Transmit(0x00U, commands, (uint8_t)sizeof(commands));
}

void OLED_SetCursor(uint8_t Y, uint8_t X)
{
	(void)OLED_SetCursorChecked(Y, X);
}

/**
  * @brief  OLED清屏
  * @param  无
  * @retval 无
  */
void OLED_Clear(void)
{
	uint8_t page;

	OLED_TakeMutex();
	for (page = 0U; page < OLED_PAGE_COUNT; page++)
	{
		OLED_SetCursor(page, 0U);
		OLED_WriteDataBuffer(OLED_ClearLine, OLED_WIDTH);
	}
	OLED_GiveMutex();
}

static void OLED_WriteCharNoMutex(uint8_t Line, uint8_t Column, char Char)
{
	uint8_t i;
	uint8_t characterData[8];

	OLED_SetCursor((Line - 1) * 2, (Column - 1) * 8);
	for (i = 0; i < 8; i++)
	{
		characterData[i] = OLED_F8x16[Char - ' '][i];
	}
	OLED_WriteDataBuffer(characterData, sizeof(characterData));

	OLED_SetCursor((Line - 1) * 2 + 1, (Column - 1) * 8);
	for (i = 0; i < 8; i++)
	{
		characterData[i] = OLED_F8x16[Char - ' '][i + 8];
	}
	OLED_WriteDataBuffer(characterData, sizeof(characterData));
}

/**
  * @brief  OLED显示一个字符
  * @param  Line 行位置，范围：1~4
  * @param  Column 列位置，范围：1~16
  * @param  Char 要显示的一个字符，范围：ASCII可见字符
  * @retval 无
  */
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char)
{
	OLED_TakeMutex();
	if (OLED_IsFeatureDataCell(Line, Column) != 0U &&
		OLED_CanDrawFeatureData() == 0U)
	{
		OLED_GiveMutex();
		return;
	}
	OLED_WriteCharNoMutex(Line, Column, Char);
	OLED_GiveMutex();
}

/**
  * @brief  OLED显示字符串
  * @param  Line 起始行位置，范围：1~4
  * @param  Column 起始列位置，范围：1~16
  * @param  String 要显示的字符串，范围：ASCII可见字符
  * @retval 无
  */
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String)
{

	uint8_t i;
	for (i = 0; String[i] != '\0'; i++)
	{
		OLED_ShowChar(Line, Column + i, String[i]);
	}

}

/**
  * @brief  OLED次方函数
  * @retval 返回值等于X的Y次方
  */
uint32_t OLED_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;
	while (Y--)
	{
		Result *= X;
	}
	return Result;
}

/**
  * @brief  OLED显示数字（十进制，正数）
  * @param  Line 起始行位置，范围：1~4
  * @param  Column 起始列位置，范围：1~16
  * @param  Number 要显示的数字，范围：0~4294967295
  * @param  Length 要显示数字的长度，范围：1~10
  * @retval 无
  */
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{

	uint8_t i;
	for (i = 0; i < Length; i++)							
	{
		OLED_ShowChar(Line, Column + i, Number / OLED_Pow(10, Length - i - 1) % 10 + '0');
	}

}

/**
  * @brief  OLED显示数字（十进制，带符号数）
  * @param  Line 起始行位置，范围：1~4
  * @param  Column 起始列位置，范围：1~16
  * @param  Number 要显示的数字，范围：-2147483648~2147483647
  * @param  Length 要显示数字的长度，范围：1~10
  * @retval 无
  */
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length)
{

	uint8_t i;
	uint32_t Number1;
	if (Number >= 0)
	{
		OLED_ShowChar(Line, Column, '+');
		Number1 = Number;
	}
	else
	{
		OLED_ShowChar(Line, Column, '-');
		Number1 = -Number;
	}
	for (i = 0; i < Length; i++)							
	{
		OLED_ShowChar(Line, Column + i + 1, Number1 / OLED_Pow(10, Length - i - 1) % 10 + '0');
	}

}

typedef struct
{
	uint8_t Line;
	uint8_t Column;
	uint8_t Initialized;
	float FilteredValue;
	int32_t LastDisplayedValue;
	uint32_t LastUpdateTick;
} OLED_SmoothFilter_t;

/* 控制值和无线回传值各自维护显示滤波状态。 */
static OLED_SmoothFilter_t OLED_SmoothFilters[4] =
{
	{2U, 7U, 0U, 0.0f, 0, 0U},
	{3U, 7U, 0U, 0.0f, 0, 0U},
	{2U, 12U, 0U, 0.0f, 0, 0U},
	{3U, 12U, 0U, 0.0f, 0, 0U}
};

static void OLED_ResetFeatureSmoothFilters(void)
{
	uint8_t i;
	taskENTER_CRITICAL();
	for (i = 0U; i < 4U; i++)
	{
		OLED_SmoothFilters[i].Initialized = 0U;
	}
	taskEXIT_CRITICAL();
}

/**
  * @brief  平滑显示带符号数值，不改变传入的控制数据。
  * @param  Number 原始显示值
  * @retval 无
  */
void OLED_ShowSmoothSignedNum(uint8_t Line, uint8_t Column, float Number, uint8_t Length)
{
	uint8_t i;
	uint32_t elapsed;
	uint32_t currentTick;
	int32_t displayedValue;
	float alpha;
	uint8_t forceDraw = 0U;
	uint8_t updateNeeded = 1U;
	uint8_t valueIndex;
	OLED_SmoothFilter_t *filter = NULL;

	if (Number != Number)
	{
		return;
	}
	if (Number > 2000000000.0f)
	{
		Number = 2000000000.0f;
	}
	else if (Number < -2000000000.0f)
	{
		Number = -2000000000.0f;
	}

	for (i = 0; i < 4U; i++)
	{
		if (OLED_SmoothFilters[i].Line == Line && OLED_SmoothFilters[i].Column == Column)
		{
			filter = &OLED_SmoothFilters[i];
			break;
		}
	}

	if (filter == NULL)
	{
		OLED_ShowSignedNum(Line, Column, (int32_t)Number, Length);
		return;
	}

	if (OLED_IsFeatureDataCell(Line, Column) != 0U &&
		OLED_CanDrawFeatureData() == 0U)
	{
		taskENTER_CRITICAL();
		filter->Initialized = 0U;
		taskEXIT_CRITICAL();
		return;
	}

	currentTick = HAL_GetTick();
	taskENTER_CRITICAL();
	elapsed = currentTick - filter->LastUpdateTick;

	if (filter->Initialized == 0U || elapsed > 300U)
	{
		filter->FilteredValue = Number;
		filter->Initialized = 1U;
		forceDraw = 1U;
	}
	else
	{
		alpha = (float)elapsed / (80.0f + (float)elapsed);
		filter->FilteredValue += (Number - filter->FilteredValue) * alpha;
	}

	filter->LastUpdateTick = currentTick;
	displayedValue = (int32_t)filter->FilteredValue;
	if (forceDraw == 0U && displayedValue == filter->LastDisplayedValue)
	{
		updateNeeded = 0U;
	}
	else
	{
		filter->LastDisplayedValue = displayedValue;
	}
	taskEXIT_CRITICAL();
	if (updateNeeded == 0U)
	{
		return;
	}

	valueIndex = (uint8_t)((Line == 3U ? 1U : 0U) + (Column == 12U ? 2U : 0U));
	OLED_UpdateFeatureValue(valueIndex, displayedValue);
}

void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{

	uint8_t i, SingleNumber;
	for (i = 0; i < Length; i++)							
	{
		SingleNumber = Number / OLED_Pow(16, Length - i - 1) % 16;
		if (SingleNumber < 10)
		{
			OLED_ShowChar(Line, Column + i, SingleNumber + '0');
		}
		else
		{
			OLED_ShowChar(Line, Column + i, SingleNumber - 10 + 'A');
		}
	}

}

/**
  * @brief  OLED显示数字（二进制，正数）
  * @param  Line 起始行位置，范围：1~4
  * @param  Column 起始列位置，范围：1~16
  * @param  Number 要显示的数字，范围：0~1111 1111 1111 1111
  * @param  Length 要显示数字的长度，范围：1~16
  * @retval 无
  */
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{

	uint8_t i;
	for (i = 0; i < Length; i++)							
	{
		OLED_ShowChar(Line, Column + i, Number / OLED_Pow(2, Length - i - 1) % 2 + '0');
	}

}

static void OLED_ClearFrame(uint8_t *Frame)
{
	uint8_t i;
	for (i = 0U; i < OLED_WIDTH; i++)
	{
		Frame[i] = 0U;
	}
}

static void OLED_PutPixel(uint8_t *Frame, uint8_t X, uint8_t Y, uint8_t Value)
{
	uint8_t index;
	uint8_t mask;

	if (X >= OLED_WIDTH || Y >= OLED_HEIGHT || Y / 8U != OLED_RenderPageIndex)
	{
		return;
	}

	index = X;
	mask = (uint8_t)(1U << (Y & 7U));
	if (Value != 0U)
	{
		Frame[index] |= mask;
	}
	else
	{
		Frame[index] &= (uint8_t)~mask;
	}
}

static uint8_t OLED_GetGlyphPixel(char Character, uint8_t X, uint8_t Y)
{
	uint8_t glyphIndex;
	uint8_t fontX;
	uint8_t fontY;

	if (Character < ' ' || Character > '~' || X >= 8U || Y >= 16U)
	{
		return 0U;
	}

	glyphIndex = (uint8_t)(Character - ' ');
	fontX = X;
	fontY = Y;
	return (OLED_F8x16[glyphIndex][fontX + (fontY >= 8U ? 8U : 0U)] &
		(uint8_t)(1U << (fontY & 7U))) != 0U ? 1U : 0U;
}

static void OLED_DrawLargeChar(uint8_t *Frame, uint8_t X, uint8_t Y,
	char Character, uint8_t Foreground)
{
	uint8_t x;
	uint8_t y;
	uint8_t glyph;
	uint8_t pixel;

	for (x = 0U; x < 8U; x++)
	{
		for (y = 0U; y < 16U; y++)
		{
			glyph = OLED_GetGlyphPixel(Character, x, y);
			pixel = glyph != 0U ? Foreground : (uint8_t)!Foreground;
			OLED_PutPixel(Frame, (uint8_t)(X + x), (uint8_t)(Y + y), pixel);
		}
	}
}

static void OLED_DrawLargeText(uint8_t *Frame, uint8_t X, uint8_t Y,
	const char *Text, uint8_t Foreground)
{
	uint8_t index;
	uint16_t x;
	for (index = 0U; Text[index] != '\0'; index++)
	{
		x = (uint16_t)X + (uint16_t)index * 8U;
		if (x + 8U <= OLED_WIDTH)
		{
			OLED_DrawLargeChar(Frame, (uint8_t)x, Y, Text[index], Foreground);
		}
	}
}

/* 从现有8x16 ASCII字模压缩绘制6x8提示，避免额外占用字库常量空间。 */
static void OLED_DrawCompactChar(uint8_t *Frame, uint8_t X, uint8_t Y,
	char Character, uint8_t Foreground)
{
	uint8_t x;
	uint8_t y;
	uint8_t sourceX;
	uint8_t sourceXEnd;
	uint8_t sourceY;
	uint8_t glyph;
	uint8_t pixel;

	for (x = 0U; x < 6U; x++)
	{
		sourceX = (uint8_t)(((uint16_t)x * 8U) / 6U);
		sourceXEnd = (uint8_t)(((uint16_t)(x + 1U) * 8U) / 6U);
		for (y = 0U; y < 8U; y++)
		{
			sourceY = (uint8_t)(y * 2U);
			glyph = OLED_GetGlyphPixel(Character, sourceX, sourceY) |
				OLED_GetGlyphPixel(Character, sourceX, (uint8_t)(sourceY + 1U));
			if (sourceXEnd > (uint8_t)(sourceX + 1U))
			{
				glyph |= OLED_GetGlyphPixel(Character,
					(uint8_t)(sourceX + 1U), sourceY) |
					OLED_GetGlyphPixel(Character, (uint8_t)(sourceX + 1U),
						(uint8_t)(sourceY + 1U));
			}
			pixel = glyph != 0U ? Foreground : (uint8_t)!Foreground;
			OLED_PutPixel(Frame, (uint8_t)(X + x), (uint8_t)(Y + y), pixel);
		}
	}
}

static void OLED_DrawCompactText(uint8_t *Frame, uint8_t X, uint8_t Y,
	const char *Text, uint8_t Foreground)
{
	uint8_t index;
	uint16_t x;

	for (index = 0U; Text[index] != '\0'; index++)
	{
		x = (uint16_t)X + (uint16_t)index * 6U;
		if (x + 6U <= OLED_WIDTH)
		{
			OLED_DrawCompactChar(Frame, (uint8_t)x, Y, Text[index], Foreground);
		}
	}
}

static void OLED_FillRect(uint8_t *Frame, uint8_t X, uint8_t Y,
	uint8_t Width, uint8_t Height, uint8_t Value)
{
	uint8_t x;
	uint8_t y;
	for (y = 0U; y < Height; y++)
	{
		for (x = 0U; x < Width; x++)
		{
			OLED_PutPixel(Frame, (uint8_t)(X + x), (uint8_t)(Y + y), Value);
		}
	}
}

static void OLED_DrawRoundRect(uint8_t *Frame, uint8_t X, uint8_t Y,
	uint8_t Width, uint8_t Height, uint8_t Radius, uint8_t Value)
{
	uint8_t x;
	uint8_t y;
	uint8_t dx;
	uint8_t dy;
	uint8_t draw;
	uint16_t distanceSquared;
	uint16_t radiusSquared;

	radiusSquared = (uint16_t)Radius * Radius;
	for (y = 0U; y < Height; y++)
	{
		for (x = 0U; x < Width; x++)
		{
			draw = 1U;
			if (x < Radius && y < Radius)
			{
				dx = (uint8_t)(Radius - x);
				dy = (uint8_t)(Radius - y);
				distanceSquared = (uint16_t)dx * dx + (uint16_t)dy * dy;
				draw = distanceSquared <= radiusSquared ? 1U : 0U;
			}
			else if (x >= Width - Radius && y < Radius)
			{
				dx = (uint8_t)(x - (Width - Radius - 1U));
				dy = (uint8_t)(Radius - y);
				distanceSquared = (uint16_t)dx * dx + (uint16_t)dy * dy;
				draw = distanceSquared <= radiusSquared ? 1U : 0U;
			}
			else if (x < Radius && y >= Height - Radius)
			{
				dx = (uint8_t)(Radius - x);
				dy = (uint8_t)(y - (Height - Radius - 1U));
				distanceSquared = (uint16_t)dx * dx + (uint16_t)dy * dy;
				draw = distanceSquared <= radiusSquared ? 1U : 0U;
			}
			else if (x >= Width - Radius && y >= Height - Radius)
			{
				dx = (uint8_t)(x - (Width - Radius - 1U));
				dy = (uint8_t)(y - (Height - Radius - 1U));
				distanceSquared = (uint16_t)dx * dx + (uint16_t)dy * dy;
				draw = distanceSquared <= radiusSquared ? 1U : 0U;
			}
			if (draw != 0U)
			{
				OLED_PutPixel(Frame, (uint8_t)(X + x), (uint8_t)(Y + y), Value);
			}
		}
	}
}

static void OLED_FormatUnsigned(uint32_t Value, char *Buffer)
{
	char reversed[10];
	uint8_t count = 0U;
	uint8_t i;
	do
	{
		reversed[count++] = (char)('0' + Value % 10U);
		Value /= 10U;
	} while (Value != 0U && count < sizeof(reversed));

	for (i = 0U; i < count; i++)
	{
		Buffer[i] = reversed[count - i - 1U];
	}
	Buffer[count] = '\0';
}

static void OLED_FormatSigned(int32_t Value, uint8_t AlwaysSign, char *Buffer)
{
	uint32_t magnitude;
	uint8_t offset = 0U;
	if (Value < 0)
	{
		Buffer[offset++] = '-';
		magnitude = (uint32_t)(-(int64_t)Value);
	}
	else
	{
		magnitude = (uint32_t)Value;
		if (AlwaysSign != 0U && Value > 0)
		{
			Buffer[offset++] = '+';
		}
	}
	OLED_FormatUnsigned(magnitude, &Buffer[offset]);
}

static void OLED_DrawHeader(uint8_t *Frame, const OLED_UIState_t *State)
{
	const char *title;
	const char *status;
	uint8_t connectionState;
	static const char *const leftStatus[4] =
		{"SOLO-L", "WAIT-L", "DUAL-L", "FAULT-L"};
	static const char *const rightStatus[4] =
		{"SOLO-R", "WAIT-R", "DUAL-R", "FAULT-R"};

	title = "MODE";
	if (State->Screen == OLED_UI_SCREEN_LINK)
	{
		title = "LINK";
	}
	else if (State->Screen == OLED_UI_SCREEN_FEATURE)
	{
		if (State->Root == 1U)
		{
			title = State->Submenu == 3U ? "RA/GYRO" : "RA/JS";
		}
		else if (State->Root == 2U)
		{
			title = State->Submenu == 3U ? "MW/GYRO" : "MW/JS";
		}
		else
		{
			title = State->Submenu == 3U ? "SP/GYRO" : "SP/JS";
		}
	}
	else if (State->Level == 1U)
	{
		if (State->Root == 0U)
		{
			title = "SP/MODE";
		}
		else if (State->Root == 1U)
		{
			title = "RA/MODE";
		}
		else
		{
			title = "MW/MODE";
		}
	}
	else if (State->Level == 2U && State->Submenu == 1U)
	{
		title = "SET/ID";
	}
	else if (State->Level == 2U)
	{
		title = "CONTROL";
	}

	OLED_DrawLargeText(Frame, 4U, 0U, title, 1U);
	connectionState = State->ConnectionState;
	if (connectionState > CONTROLLER_LINK_STATE_FAULT)
	{
		connectionState = CONTROLLER_LINK_STATE_FAULT;
	}
	status = State->LocalRole == CONTROLLER_ROLE_RIGHT ?
		rightStatus[connectionState] : leftStatus[connectionState];
	OLED_DrawRoundRect(Frame, 64U, 0U, 64U, 16U, 3U, 1U);
	OLED_DrawLargeText(Frame, 68U, 0U, status, 0U);
	OLED_FillRect(Frame, 4U, 15U, 121U, 1U, 1U);
}

static uint8_t OLED_GetRootSelection(uint8_t Root)
{
	if (Root == 0U)
	{
		return 0U;
	}
	return Root == 2U ? 1U : 2U;
}

static uint8_t OLED_GetSubmenuSelection(uint8_t Submenu)
{
	if (Submenu == 0U)
	{
		return 0U;
	}
	if (Submenu == 3U)
	{
		return 1U;
	}
	if (Submenu == 2U)
	{
		return 2U;
	}
	return 3U;
}

static void OLED_DrawMenu(uint8_t *Frame, const char *const *Items,
	uint8_t Count, uint8_t Selection)
{
	uint8_t visible;
	uint8_t firstItem = 0U;
	uint8_t row;
	uint8_t itemIndex;
	uint8_t rowY;
	uint8_t selectionRow;
	uint8_t thumbHeight;
	uint8_t thumbY;

	if (Count == 0U)
	{
		return;
	}
	if (Selection >= Count)
	{
		Selection = (uint8_t)(Count - 1U);
	}
	visible = Count > 3U ? 3U : Count;
	if (Selection >= visible)
	{
		firstItem = (uint8_t)(Selection - visible + 1U);
	}
	selectionRow = (uint8_t)(Selection - firstItem);

	for (row = 0U; row < visible; row++)
	{
		itemIndex = (uint8_t)(firstItem + row);
		rowY = (uint8_t)(16U + row * 16U);
		if (row == selectionRow)
		{
			OLED_DrawRoundRect(Frame, 4U, rowY, 116U, 16U, 3U, 1U);
			OLED_DrawLargeChar(Frame, 8U, rowY, '>', 0U);
			OLED_DrawLargeText(Frame, 21U, rowY, Items[itemIndex], 0U);
		}
		else
		{
			OLED_DrawLargeText(Frame, 13U, rowY, Items[itemIndex], 1U);
		}
	}

	if (Count > 3U)
	{
		OLED_FillRect(Frame, 125U, 18U, 1U, 44U, 1U);
		thumbHeight = 12U;
		thumbY = (uint8_t)(18U + (firstItem * (44U - thumbHeight)) / (Count - 3U));
		OLED_DrawRoundRect(Frame, 122U, thumbY, 5U, thumbHeight, 2U, 1U);
	}
}

/* 主菜单和二级菜单固定高亮在中间行，前后选项按循环顺序排列。 */
static void OLED_DrawRollingMenu(uint8_t *Frame, const char *const *Items,
	uint8_t Count, uint8_t Selection)
{
	uint8_t row;
	uint8_t itemIndex;
	uint8_t rowY;
	uint8_t thumbHeight;
	uint8_t thumbY;

	if (Count == 0U)
	{
		return;
	}
	if (Selection >= Count)
	{
		Selection = (uint8_t)(Count - 1U);
	}

	for (row = 0U; row < 3U; row++)
	{
		if (row == 0U)
		{
			itemIndex = Selection == 0U ? (uint8_t)(Count - 1U) :
				(uint8_t)(Selection - 1U);
		}
		else if (row == 1U)
		{
			itemIndex = Selection;
		}
		else
		{
			itemIndex = Selection + 1U < Count ?
				(uint8_t)(Selection + 1U) : 0U;
		}

		rowY = (uint8_t)(16U + row * 16U);
		if (row == 1U)
		{
			OLED_DrawRoundRect(Frame, 4U, rowY, 116U, 16U, 3U, 1U);
			OLED_DrawLargeChar(Frame, 8U, rowY, '>', 0U);
			OLED_DrawLargeText(Frame, 21U, rowY, Items[itemIndex], 0U);
		}
		else
		{
			OLED_DrawLargeText(Frame, 21U, rowY, Items[itemIndex], 1U);
		}
	}

	if (Count > 3U)
	{
		OLED_FillRect(Frame, 125U, 18U, 1U, 44U, 1U);
		thumbHeight = 12U;
		thumbY = (uint8_t)(18U +
			(Selection * (44U - thumbHeight)) / (Count - 1U));
		OLED_DrawRoundRect(Frame, 122U, thumbY, 5U, thumbHeight, 2U, 1U);
	}
}

static void OLED_DrawCompactValueBar(uint8_t *Frame, uint8_t Y,
	int32_t Value, uint16_t Range, uint8_t Unipolar)
{
	uint8_t width;
	uint8_t center = 108U;
	uint32_t magnitude;

	if (Range == 0U)
	{
		return;
	}
	OLED_DrawRoundRect(Frame, 92U, (uint8_t)(Y + 5U), 32U, 5U, 2U, 1U);
	if (Unipolar != 0U)
	{
		if (Value < 0)
		{
			Value = 0;
		}
		if ((uint32_t)Value > Range)
		{
			Value = (int32_t)Range;
		}
		width = (uint8_t)(((uint32_t)Value * 28U) / Range);
		OLED_FillRect(Frame, 94U, (uint8_t)(Y + 7U), width, 1U, 1U);
	}
	else
	{
		if (Value > (int32_t)Range)
		{
			Value = (int32_t)Range;
		}
		else if (Value < -(int32_t)Range)
		{
			Value = -(int32_t)Range;
		}
		magnitude = Value < 0 ? (uint32_t)(-(int64_t)Value) : (uint32_t)Value;
		width = (uint8_t)((magnitude * 13U) / Range);
		OLED_FillRect(Frame, center, (uint8_t)(Y + 6U), 1U, 3U, 1U);
		if (width != 0U)
		{
			OLED_FillRect(Frame,
				(uint8_t)(center - (Value < 0 ? width : 0U) + (Value > 0 ? 1U : 0U)),
				(uint8_t)(Y + 7U), width, 1U, 1U);
		}
	}
}

static void OLED_DrawFeatureValueRow(uint8_t *Frame, uint8_t Y,
	const char *Label, int32_t Value, int32_t Secondary,
	uint8_t HasSecondary, uint8_t Unipolar, uint16_t Range, uint8_t ShowBar)
{
	char valueText[12];
	char secondaryText[12];
	uint8_t index;
	uint16_t x;

	if (Unipolar != 0U)
	{
		OLED_FormatUnsigned(Value < 0 ? 0U : (uint32_t)Value, valueText);
	}
	else
	{
		OLED_FormatSigned(Value, 1U, valueText);
	}
	OLED_DrawLargeText(Frame, 4U, Y, Label, 1U);
	index = 0U;
	while (Label[index] != '\0')
	{
		index++;
	}
	x = (uint16_t)4U + (uint16_t)index * 8U + 8U;
	if (x < OLED_WIDTH)
	{
		OLED_DrawLargeText(Frame, (uint8_t)x, Y, valueText, 1U);
	}
	index = 0U;
	while (valueText[index] != '\0')
	{
		index++;
	}
	x += (uint16_t)index * 8U + 8U;
	if (HasSecondary != 0U)
	{
		OLED_FormatSigned(Secondary, 1U, secondaryText);
		if (x < OLED_WIDTH)
		{
			OLED_DrawLargeText(Frame, (uint8_t)x, Y, "V:", 1U);
		}
		x += 16U;
		if (x < OLED_WIDTH)
		{
			OLED_DrawLargeText(Frame, (uint8_t)x, Y, secondaryText, 1U);
		}
	}
	if (ShowBar != 0U)
	{
		OLED_DrawCompactValueBar(Frame, Y, Value, Range, Unipolar);
	}
}

static void OLED_DrawFeature(uint8_t *Frame, const OLED_UIState_t *State)
{
	uint8_t raJoystick;
	uint8_t isGyro;
	const char *controlText;

	if (State->Root == 2U && State->MwGyroState != 0U)
	{
		if (State->Selection == 0U)
		{
			controlText = "CONTROL OFF";
		}
		else if (State->MwGyroState == 2U)
		{
			controlText = "CTL ON SPIN LIVE";
		}
		else if (State->MwGyroState == 3U)
		{
			controlText = "CTL ON SPIN SET";
		}
		else if (State->MwGyroState == 4U)
		{
			controlText = "CTL ON SPIN LOCK";
		}
		else
		{
			controlText = "CTL ON SPIN OFF";
		}
	}
	else
	{
		controlText = State->Selection != 0U ? "CONTROL ON" : "CONTROL OFF";
	}
	OLED_DrawLargeText(Frame, 4U, 16U, controlText, 1U);

	raJoystick = State->Root == 1U && State->Submenu == 0U ? 1U : 0U;
	isGyro = State->Submenu == 3U ? 1U : 0U;
	if (raJoystick != 0U)
	{
		OLED_DrawFeatureValueRow(Frame, 32U, "HGT", State->Values[0],
			State->Values[2], 1U, 1U, 128U, 0U);
		OLED_DrawFeatureValueRow(Frame, 48U, "ANG", State->Values[1],
			State->Values[3], 1U, 0U, 180U, 0U);
	}
	else
	{
		if (isGyro != 0U)
		{
			OLED_DrawFeatureValueRow(Frame, 32U, "ROLL", State->Values[0],
				0, 0U, 0U, 180U, 1U);
			OLED_DrawFeatureValueRow(Frame, 48U, "PITCH", State->Values[1],
				0, 0U, 0U, 180U, 1U);
		}
		else
		{
			OLED_DrawFeatureValueRow(Frame, 32U, "X", State->Values[0],
				0, 0U, 0U, 100U, 1U);
			OLED_DrawFeatureValueRow(Frame, 48U, "Y", State->Values[1],
				0, 0U, 0U, 100U, 1U);
		}
	}
}

/* 将一个UI状态绘制成指定屏幕页的128字节图像，供刷新和动画共用。 */
static void OLED_RenderPage(const OLED_UIState_t *State, uint8_t Page, uint8_t *Frame)
{
	static const char *const mainItems[3] = {"SP", "MW", "RA"};
	static const char *const submenuItems[4] = {"JS", "GYRO", "MIX", "SET"};
	static const char *const frequencyItems[2] = {"FREQ", "ARR(ID)"};
	static const char *const enableItems[2] = {"OFF", "ON"};

	OLED_RenderPageIndex = Page;
	OLED_ClearFrame(Frame);
	OLED_DrawHeader(Frame, State);
		switch (State->Screen)
		{
		case OLED_UI_SCREEN_MAIN:
			OLED_DrawRollingMenu(Frame, mainItems, 3U,
				OLED_GetRootSelection(State->Root));
			break;

		case OLED_UI_SCREEN_MENU:
			if (State->Level == 1U)
			{
				OLED_DrawRollingMenu(Frame, submenuItems, 4U,
					OLED_GetSubmenuSelection(State->Submenu));
			}
			else if (State->Level == 2U && State->Submenu == 1U)
			{
				OLED_DrawMenu(Frame, frequencyItems, 2U, State->Selection);
			}
			else if (State->Level == 2U)
			{
				OLED_DrawMenu(Frame, enableItems, 2U, State->Selection);
			}
			break;

		case OLED_UI_SCREEN_FEATURE:
			OLED_DrawFeature(Frame, State);
			break;

		case OLED_UI_SCREEN_LINK:
			if (State->ConnectionState == CONTROLLER_LINK_STATE_FAULT)
			{
				OLED_DrawLargeText(Frame, 28U, 24U, "LINK LOST", 1U);
				OLED_DrawLargeText(Frame, 28U, 40U, "SAFE STOP", 1U);
			}
			else
			{
				OLED_DrawLargeText(Frame, 24U, 24U, "PAIRING...", 1U);
				OLED_DrawLargeText(Frame, 20U, 40U, "CONTROL OFF", 1U);
			}
			break;

		default:
			break;
	}
}

static HAL_StatusTypeDef OLED_FlushPage(uint8_t Page, const uint8_t *Data)
{
	HAL_StatusTypeDef status;

	OLED_TakeMutex();
	status = OLED_SetCursorChecked(Page, 0U);
	if (status == HAL_OK)
	{
		status = OLED_WriteDataBuffer(Data, OLED_WIDTH);
	}
	OLED_GiveMutex();
	return status;
}

static HAL_StatusTypeDef OLED_FlushPageDiff(uint8_t Page, const uint8_t *Data)
{
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t x;
	uint8_t first;
	uint8_t last;
	uint8_t length;
	uint16_t base;
	uint32_t drawStartCycles;

	drawStartCycles = OLED_TimingReadCycles();
	base = (uint16_t)Page * OLED_WIDTH;
	first = OLED_WIDTH;
	last = 0U;
	for (x = 0U; x < OLED_WIDTH; x++)
	{
		if (OLED_FrameBuffer[base + x] != Data[x])
		{
			if (first == OLED_WIDTH)
			{
				first = x;
			}
			last = x;
		}
	}
	OLED_TimingAddDrawCycles(drawStartCycles);
	if (first != OLED_WIDTH)
	{
		length = (uint8_t)(last - first + 1U);
		OLED_TakeMutex();
		status = OLED_SetCursorChecked(Page, first);
		if (status == HAL_OK)
		{
			status = OLED_WriteDataBuffer(&Data[first], length);
		}
		OLED_GiveMutex();
	}
	return status;
}

static void OLED_CopyPageToFrame(uint8_t Page, const uint8_t *Data)
{
	uint8_t x;
	uint16_t base = (uint16_t)Page * OLED_WIDTH;
	uint32_t drawStartCycles = OLED_TimingReadCycles();
	for (x = 0U; x < OLED_WIDTH; x++)
	{
		OLED_FrameBuffer[base + x] = Data[x];
	}
	OLED_TimingAddDrawCycles(drawStartCycles);
}

static HAL_StatusTypeDef OLED_RenderState(const OLED_UIState_t *State, uint8_t FullRefresh)
{
	HAL_StatusTypeDef status;
	uint8_t page;
	uint32_t drawStartCycles;
	uint32_t frameStartCycles;

	OLED_TimingBeginFrame(&frameStartCycles);
	for (page = 0U; page < OLED_PAGE_COUNT; page++)
	{
		drawStartCycles = OLED_TimingReadCycles();
		OLED_RenderPage(State, page, OLED_TargetPage);
		OLED_TimingAddDrawCycles(drawStartCycles);
		if (FullRefresh != 0U)
		{
			status = OLED_FlushPage(page, OLED_TargetPage);
		}
		else
		{
			status = OLED_FlushPageDiff(page, OLED_TargetPage);
		}
		if (status != HAL_OK)
		{
			OLED_TimingFinishFrame(frameStartCycles, status);
			return status;
		}
		/* 只有整页传输成功，缓存才代表屏幕上实际显示的内容。 */
		OLED_CopyPageToFrame(page, OLED_TargetPage);
	}
	OLED_TimingFinishFrame(frameStartCycles, HAL_OK);
	return HAL_OK;
}

/* 从上一帧已显示缓存平移Delta列，仅补入这次新进入的目标页列。 */
static void OLED_ComposeTransitionPage(uint8_t Page, uint8_t Offset,
	uint8_t Delta, uint8_t Entering)
{
	uint8_t x;
	uint8_t sourceX;
	uint16_t base = (uint16_t)Page * OLED_WIDTH;

	for (x = 0U; x < OLED_WIDTH; x++)
	{
		if (Entering != 0U)
		{
			/* 进入方向保持左移，右侧补入目标页[Offset-Delta, Offset)。 */
			if (x < OLED_WIDTH - Delta)
			{
				OLED_NextPage[x] = OLED_FrameBuffer[base + x + Delta];
			}
			else
			{
				sourceX = (uint8_t)(x - (OLED_WIDTH - Offset));
				OLED_NextPage[x] = OLED_TargetPage[sourceX];
			}
		}
		else if (x < Delta)
		{
			/* 退出方向保持右移，左侧补入目标页的新列。 */
			sourceX = (uint8_t)(x + OLED_WIDTH - Offset);
			OLED_NextPage[x] = OLED_TargetPage[sourceX];
		}
		else
		{
			OLED_NextPage[x] = OLED_FrameBuffer[base + x - Delta];
		}
	}
}

/* 功能页标题在动画中固定；末帧只补刷未参与滑动的页。 */
static HAL_StatusTypeDef OLED_RenderFixedPages(const OLED_UIState_t *TargetState,
	uint8_t FirstPage)
{
	uint8_t page;
	uint32_t drawStartCycles;
	HAL_StatusTypeDef status;

	for (page = 0U; page < FirstPage; page++)
	{
		drawStartCycles = OLED_TimingReadCycles();
		OLED_RenderPage(TargetState, page, OLED_TargetPage);
		OLED_TimingAddDrawCycles(drawStartCycles);
		status = OLED_FlushPageDiff(page, OLED_TargetPage);
		if (status != HAL_OK)
		{
			return status;
		}
		OLED_CopyPageToFrame(page, OLED_TargetPage);
	}
	return HAL_OK;
}

static uint8_t OLED_ReadMenuBodyPixel(uint8_t X, uint8_t Y)
{
	uint8_t screenY = (uint8_t)(16U + Y);
	uint16_t base = (uint16_t)(screenY / 8U) * OLED_WIDTH;

	return (uint8_t)((OLED_FrameBuffer[base + X] >> (screenY % 8U)) & 1U);
}

static uint8_t OLED_IsMenuSelectionPixel(uint8_t X, uint8_t Y)
{
	uint8_t localX;
	uint8_t localY;
	uint8_t dx;
	uint8_t dy;
	uint16_t distanceSquared;

	if (X < 4U || X >= 120U || Y < 16U || Y >= 32U)
	{
		return 0U;
	}
	localX = (uint8_t)(X - 4U);
	localY = (uint8_t)(Y - 16U);
	if ((localX < 3U && localY < 3U) ||
		(localX >= 113U && localY < 3U) ||
		(localX < 3U && localY >= 13U) ||
		(localX >= 113U && localY >= 13U))
	{
		if (localX < 3U && localY < 3U)
		{
			dx = (uint8_t)(3U - localX);
			dy = (uint8_t)(3U - localY);
		}
		else if (localX >= 113U && localY < 3U)
		{
			dx = (uint8_t)(localX - 112U);
			dy = (uint8_t)(3U - localY);
		}
		else if (localX < 3U)
		{
			dx = (uint8_t)(3U - localX);
			dy = (uint8_t)(localY - 12U);
		}
		else
		{
			dx = (uint8_t)(localX - 112U);
			dy = (uint8_t)(localY - 12U);
		}
		distanceSquared = (uint16_t)dx * dx + (uint16_t)dy * dy;
		return distanceSquared <= 9U ? 1U : 0U;
	}
	return 1U;
}

/* 按帧间差值移动缓存，边缘新像素按总偏移从目标页补入。 */
static void OLED_ComposeMenuScrollPage(const OLED_UIState_t *TargetState,
	uint8_t Page, uint8_t Delta, uint8_t Offset, uint8_t ScrollUp)
{
	uint8_t x;
	uint8_t bit;
	uint8_t destinationY;
	uint8_t sourceY;
	uint8_t targetScreenY;
	uint8_t targetPage;
	uint8_t targetBit;
	uint8_t loadedTargetPage = Page;
	uint8_t sourceIsTarget;
	uint8_t pixel;
	uint8_t targetRail[6U];

	for (x = 0U; x < OLED_WIDTH; x++)
	{
		OLED_NextPage[x] = 0U;
		if (x >= 122U)
		{
			targetRail[x - 122U] = OLED_TargetPage[x];
		}
	}

	for (bit = 0U; bit < 8U; bit++)
	{
		destinationY = (uint8_t)((Page - 2U) * 8U + bit);
		sourceIsTarget = 0U;
		if (ScrollUp != 0U)
		{
			sourceY = (uint8_t)(destinationY + Delta);
			if (sourceY >= 48U)
			{
				/* 原菜单下方的新行随总偏移进入屏幕。 */
				targetScreenY = (uint8_t)(destinationY + Offset);
				sourceIsTarget = 1U;
			}
		}
		else if (destinationY < Delta)
		{
			/* 原菜单上方的新行随总偏移进入屏幕。 */
			targetScreenY = (uint8_t)(destinationY + 32U - Offset);
			sourceIsTarget = 1U;
		}

		if (sourceIsTarget != 0U)
		{
			targetPage = (uint8_t)(targetScreenY / 8U);
			targetBit = (uint8_t)(targetScreenY % 8U);
			if (loadedTargetPage != targetPage)
			{
				OLED_RenderPage(TargetState, targetPage, OLED_TargetPage);
				loadedTargetPage = targetPage;
			}
		}
		else if (ScrollUp == 0U)
		{
			sourceY = (uint8_t)(destinationY - Delta);
		}

		for (x = 0U; x < OLED_WIDTH; x++)
		{
			if (x >= 122U)
			{
				pixel = (uint8_t)((targetRail[x - 122U] >> bit) & 1U);
			}
			else if (sourceIsTarget != 0U)
			{
				pixel = (uint8_t)((OLED_TargetPage[x] >> targetBit) & 1U);
			}
			else
			{
				pixel = OLED_ReadMenuBodyPixel(x, sourceY);
				if (OLED_IsMenuSelectionPixel(x, sourceY) != 0U)
				{
					if (sourceY >= 16U && sourceY < 32U &&
						x >= 8U && x < 16U)
					{
						/* 旧固定光标离开时恢复为普通黑底。 */
						pixel = 0U;
					}
					else
					{
						pixel = (uint8_t)!pixel;
					}
				}
			}

			if (OLED_IsMenuSelectionPixel(x, destinationY) != 0U)
			{
				pixel = (uint8_t)!pixel;
			}
			if (pixel != 0U)
			{
				OLED_NextPage[x] |= (uint8_t)(1U << bit);
			}
		}
	}

	OLED_RenderPageIndex = Page;
	if (Page == 4U || Page == 5U)
	{
		/* 中央选中框固定，只有选项文字在框下滚动。 */
		OLED_DrawLargeChar(OLED_NextPage, 8U, 32U, '>', 0U);
	}
}

static HAL_StatusTypeDef OLED_UpdateMenuScrollPage(const OLED_UIState_t *TargetState,
	uint8_t Page, uint8_t Delta, uint8_t Offset, uint8_t ScrollUp)
{
	HAL_StatusTypeDef status;
	uint32_t drawStartCycles;

	drawStartCycles = OLED_TimingReadCycles();
	OLED_RenderPage(TargetState, Page, OLED_TargetPage);
	OLED_TimingAddDrawCycles(drawStartCycles);
	drawStartCycles = OLED_TimingReadCycles();
	OLED_ComposeMenuScrollPage(TargetState, Page, Delta, Offset, ScrollUp);
	OLED_TimingAddDrawCycles(drawStartCycles);
	status = OLED_FlushPageDiff(Page, OLED_NextPage);
	if (status == HAL_OK)
	{
		/* 下一页可能引用这一页的旧像素，成功刷新后再更新缓存。 */
		OLED_CopyPageToFrame(Page, OLED_NextPage);
	}
	return status;
}

/* 从目标菜单绝对合成轻微回弹帧，避免反向移动时引用已移出屏幕的像素。 */
static void OLED_ComposeMenuBouncePage(const OLED_UIState_t *TargetState,
	uint8_t Page, uint8_t Offset, uint8_t ScrollUp)
{
	uint8_t x;
	uint8_t bit;
	uint8_t destinationY;
	uint8_t sourceY;
	uint8_t sourceScreenY;
	uint8_t sourcePage;
	uint8_t sourceBit;
	uint8_t sourceValid;
	uint8_t loadedSourcePage;
	uint8_t pixel;
	uint8_t targetRail[6U];

	OLED_RenderPage(TargetState, Page, OLED_TargetPage);
	for (x = 0U; x < OLED_WIDTH; x++)
	{
		OLED_NextPage[x] = 0U;
		if (x >= 122U)
		{
			targetRail[x - 122U] = OLED_TargetPage[x];
		}
	}
	loadedSourcePage = Page;

	for (bit = 0U; bit < 8U; bit++)
	{
		destinationY = (uint8_t)((Page - 2U) * 8U + bit);
		sourceValid = 1U;
		if (ScrollUp != 0U)
		{
			sourceY = (uint8_t)(destinationY + Offset);
			if (sourceY >= 48U)
			{
				sourceValid = 0U;
			}
		}
		else if (destinationY < Offset)
		{
			sourceValid = 0U;
		}
		else
		{
			sourceY = (uint8_t)(destinationY - Offset);
		}

		if (sourceValid != 0U)
		{
			sourceScreenY = (uint8_t)(sourceY + 16U);
			sourcePage = (uint8_t)(sourceScreenY / 8U);
			sourceBit = (uint8_t)(sourceScreenY % 8U);
			if (loadedSourcePage != sourcePage)
			{
				OLED_RenderPage(TargetState, sourcePage, OLED_TargetPage);
				loadedSourcePage = sourcePage;
			}
		}

		for (x = 0U; x < OLED_WIDTH; x++)
		{
			if (x >= 122U)
			{
				pixel = (uint8_t)((targetRail[x - 122U] >> bit) & 1U);
			}
			else if (sourceValid != 0U)
			{
				pixel = (uint8_t)((OLED_TargetPage[x] >> sourceBit) & 1U);
				if (OLED_IsMenuSelectionPixel(x, sourceY) != 0U)
				{
					if (sourceY >= 16U && sourceY < 32U &&
						x >= 8U && x < 16U)
					{
						pixel = 0U;
					}
					else
					{
						pixel = (uint8_t)!pixel;
					}
				}
			}
			else
			{
				pixel = 0U;
			}

			if (OLED_IsMenuSelectionPixel(x, destinationY) != 0U)
			{
				pixel = (uint8_t)!pixel;
			}
			if (pixel != 0U)
			{
				OLED_NextPage[x] |= (uint8_t)(1U << bit);
			}
		}
	}

	OLED_RenderPageIndex = Page;
	if (Page == 4U || Page == 5U)
	{
		/* 菜单选项回弹时仍固定中央高亮箭头。 */
		OLED_DrawLargeChar(OLED_NextPage, 8U, 32U, '>', 0U);
	}
}

static HAL_StatusTypeDef OLED_UpdateMenuBouncePage(const OLED_UIState_t *TargetState,
	uint8_t Page, uint8_t Offset, uint8_t ScrollUp)
{
	HAL_StatusTypeDef status;
	uint32_t drawStartCycles;

	drawStartCycles = OLED_TimingReadCycles();
	OLED_ComposeMenuBouncePage(TargetState, Page, Offset, ScrollUp);
	OLED_TimingAddDrawCycles(drawStartCycles);
	status = OLED_FlushPageDiff(Page, OLED_NextPage);
	if (status == HAL_OK)
	{
		OLED_CopyPageToFrame(Page, OLED_NextPage);
	}
	return status;
}

static uint8_t OLED_GetBounceOffset(uint32_t ElapsedUs,
	uint32_t DurationUs, uint8_t MaxOffset)
{
	uint8_t phase;

	if (ElapsedUs >= DurationUs)
	{
		return 0U;
	}
	phase = (uint8_t)((ElapsedUs * 4U) / DurationUs);
	if (phase == 1U)
	{
		return MaxOffset;
	}
	if (phase == 0U || phase == 2U)
	{
		return (uint8_t)((MaxOffset + 1U) / 2U);
	}
	return 0U;
}

static HAL_StatusTypeDef OLED_AnimateMenuBounce(const OLED_UIState_t *TargetState,
	uint8_t ScrollUp)
{
	uint8_t page;
	uint8_t offset;
	uint8_t lastOffset = 0U;
	HAL_StatusTypeDef status;
	uint32_t frameStartCycles;
	uint32_t startCycles = OLED_TimingReadCycles();
	uint32_t startTick = HAL_GetTick();
	uint32_t elapsedCycles;
	uint32_t elapsedUs;
	uint32_t cyclesPerUs = SystemCoreClock / 1000000U;
	for (;;)
	{
		if (OLED_TimingAvailable != 0U && cyclesPerUs != 0U)
		{
			elapsedCycles = (uint32_t)(OLED_TimingReadCycles() - startCycles);
			elapsedUs = elapsedCycles / cyclesPerUs;
		}
		else
		{
			elapsedUs = (uint32_t)(HAL_GetTick() - startTick) * 1000U;
		}

		offset = OLED_GetBounceOffset(elapsedUs,
			OLED_MENU_BOUNCE_DURATION_US, OLED_MENU_BOUNCE_MAX_PIXELS);

		if (offset == lastOffset)
		{
			if (elapsedUs >= OLED_MENU_BOUNCE_DURATION_US)
			{
				return HAL_OK;
			}
			vTaskDelay(1U);
			continue;
		}

		OLED_TimingBeginFrame(&frameStartCycles);
		status = HAL_OK;
		if (ScrollUp != 0U)
		{
			for (page = 2U; page < OLED_PAGE_COUNT; page++)
			{
				status = OLED_UpdateMenuBouncePage(TargetState, page,
					offset, ScrollUp);
				if (status != HAL_OK)
				{
					break;
				}
			}
		}
		else
		{
			page = OLED_PAGE_COUNT;
			while (page > 2U)
			{
				page--;
				status = OLED_UpdateMenuBouncePage(TargetState, page,
					offset, ScrollUp);
				if (status != HAL_OK)
				{
					break;
				}
			}
		}
		OLED_TimingFinishFrame(frameStartCycles, status);
		if (status != HAL_OK)
		{
			return status;
		}

		lastOffset = offset;
		if (elapsedUs >= OLED_MENU_BOUNCE_DURATION_US)
		{
			return HAL_OK;
		}
	}
}

static HAL_StatusTypeDef OLED_UpdatePageBouncePage(const OLED_UIState_t *TargetState,
	uint8_t Page, uint8_t Offset, uint8_t Entering)
{
	HAL_StatusTypeDef status;
	uint8_t x;
	uint16_t sourceX;
	uint32_t drawStartCycles;

	drawStartCycles = OLED_TimingReadCycles();
	OLED_RenderPage(TargetState, Page, OLED_TargetPage);
	for (x = 0U; x < OLED_WIDTH; x++)
	{
		if (Entering != 0U)
		{
			/* 进入页再向左越过少量像素，右边缘裁切。 */
			sourceX = (uint16_t)x + Offset;
			OLED_NextPage[x] = sourceX < OLED_WIDTH ?
				OLED_TargetPage[sourceX] : 0U;
		}
		else if (x < Offset)
		{
			/* 退出后页面再向右越过少量像素，左边缘裁切。 */
			OLED_NextPage[x] = 0U;
		}
		else
		{
			OLED_NextPage[x] = OLED_TargetPage[x - Offset];
		}
	}
	OLED_TimingAddDrawCycles(drawStartCycles);
	status = OLED_FlushPageDiff(Page, OLED_NextPage);
	if (status == HAL_OK)
	{
		OLED_CopyPageToFrame(Page, OLED_NextPage);
	}
	return status;
}

static HAL_StatusTypeDef OLED_AnimatePageBounce(const OLED_UIState_t *TargetState,
	uint8_t Entering, uint8_t FirstPage)
{
	uint8_t page;
	uint8_t offset;
	uint8_t lastOffset = 0U;
	HAL_StatusTypeDef status;
	uint32_t frameStartCycles;
	uint32_t startCycles = OLED_TimingReadCycles();
	uint32_t startTick = HAL_GetTick();
	uint32_t elapsedCycles;
	uint32_t elapsedUs;
	uint32_t cyclesPerUs = SystemCoreClock / 1000000U;

	for (;;)
	{
		if (OLED_TimingAvailable != 0U && cyclesPerUs != 0U)
		{
			elapsedCycles = (uint32_t)(OLED_TimingReadCycles() - startCycles);
			elapsedUs = elapsedCycles / cyclesPerUs;
		}
		else
		{
			elapsedUs = (uint32_t)(HAL_GetTick() - startTick) * 1000U;
		}
		offset = OLED_GetBounceOffset(elapsedUs,
			OLED_PAGE_BOUNCE_DURATION_US, OLED_PAGE_BOUNCE_MAX_PIXELS);

		if (offset == lastOffset)
		{
			if (elapsedUs >= OLED_PAGE_BOUNCE_DURATION_US)
			{
				return HAL_OK;
			}
			vTaskDelay(1U);
			continue;
		}

		OLED_TimingBeginFrame(&frameStartCycles);
		status = HAL_OK;
		for (page = FirstPage; page < OLED_PAGE_COUNT; page++)
		{
			status = OLED_UpdatePageBouncePage(TargetState, page,
				offset, Entering);
			if (status != HAL_OK)
			{
				break;
			}
		}
		OLED_TimingFinishFrame(frameStartCycles, status);
		if (status != HAL_OK)
		{
			return status;
		}

		lastOffset = offset;
		if (elapsedUs >= OLED_PAGE_BOUNCE_DURATION_US)
		{
			return HAL_OK;
		}
	}
}

static HAL_StatusTypeDef OLED_AnimateMenuScroll(const OLED_UIState_t *TargetState,
	uint8_t ScrollUp)
{
	uint8_t page;
	uint8_t offset;
	uint8_t lastOffset = 0U;
	uint8_t delta;
	HAL_StatusTypeDef status;
	uint32_t frameStartCycles;
	uint32_t startCycles = OLED_TimingReadCycles();
	uint32_t startTick = HAL_GetTick();
	uint32_t elapsedCycles;
	uint32_t elapsedUs;
	uint32_t cyclesPerUs = SystemCoreClock / 1000000U;

	for (;;)
	{
		if (OLED_TimingAvailable != 0U && cyclesPerUs != 0U)
		{
			elapsedCycles = (uint32_t)(OLED_TimingReadCycles() - startCycles);
			elapsedUs = elapsedCycles / cyclesPerUs;
		}
		else
		{
			elapsedUs = (uint32_t)(HAL_GetTick() - startTick) * 1000U;
		}

		if (elapsedUs >= OLED_MENU_SCROLL_DURATION_US)
		{
			offset = 16U;
		}
		else
		{
			offset = (uint8_t)((16U * elapsedUs) /
				OLED_MENU_SCROLL_DURATION_US);
		}
		if (offset == lastOffset)
		{
			vTaskDelay(1U);
			continue;
		}

		delta = (uint8_t)(offset - lastOffset);
		OLED_TimingBeginFrame(&frameStartCycles);
		status = HAL_OK;
		if (ScrollUp != 0U)
		{
			/* 上滚读取更高页的旧像素，从上到下处理以保留这些来源页。 */
			for (page = 2U; page < OLED_PAGE_COUNT; page++)
			{
				status = OLED_UpdateMenuScrollPage(TargetState, page, delta,
					offset, ScrollUp);
				if (status != HAL_OK)
				{
					break;
				}
			}
		}
		else
		{
			/* 下滚读取更低页的旧像素，从下到上处理以保留这些来源页。 */
			page = OLED_PAGE_COUNT;
			while (page > 2U)
			{
				page--;
				status = OLED_UpdateMenuScrollPage(TargetState, page, delta,
					offset, ScrollUp);
				if (status != HAL_OK)
				{
					break;
				}
			}
		}
		if (status != HAL_OK)
		{
			OLED_TimingFinishFrame(frameStartCycles, status);
			return status;
		}

		if (offset == 16U)
		{
			status = OLED_RenderFixedPages(TargetState, 2U);
			OLED_TimingFinishFrame(frameStartCycles, status);
			if (status != HAL_OK)
			{
				return status;
			}
			return OLED_AnimateMenuBounce(TargetState, ScrollUp);
		}

		OLED_TimingFinishFrame(frameStartCycles, HAL_OK);
		lastOffset = offset;
	}
}

static uint8_t OLED_GetMenuScrollDirection(const OLED_UIState_t *OldState,
	const OLED_UIState_t *NewState, uint8_t *ScrollUp)
{
	uint8_t count;
	uint8_t oldSelection;
	uint8_t newSelection;

	if (OldState->Screen == OLED_UI_SCREEN_MAIN &&
		NewState->Screen == OLED_UI_SCREEN_MAIN)
	{
		count = 3U;
		oldSelection = OLED_GetRootSelection(OldState->Root);
		newSelection = OLED_GetRootSelection(NewState->Root);
	}
	else if (OldState->Screen == OLED_UI_SCREEN_MENU &&
		NewState->Screen == OLED_UI_SCREEN_MENU &&
		OldState->Level == 1U && NewState->Level == 1U)
	{
		count = 4U;
		oldSelection = OLED_GetSubmenuSelection(OldState->Submenu);
		newSelection = OLED_GetSubmenuSelection(NewState->Submenu);
	}
	else
	{
		return 0U;
	}

	if (newSelection == (uint8_t)((oldSelection + 1U) % count))
	{
		/* 选择后一个选项时，列表内容向上滚过固定选中框。 */
		*ScrollUp = 1U;
		return 1U;
	}
	if (oldSelection == (uint8_t)((newSelection + 1U) % count))
	{
		*ScrollUp = 0U;
		return 1U;
	}
	return 0U;
}

static HAL_StatusTypeDef OLED_AnimatePageTransition(const OLED_UIState_t *TargetState,
	uint8_t Entering, uint8_t FirstPage)
{
	uint8_t page;
	uint8_t offset;
	uint8_t lastOffset = 0U;
	uint8_t delta;
	HAL_StatusTypeDef status;
	uint32_t drawStartCycles;
	uint32_t frameStartCycles;
	uint32_t startCycles = OLED_TimingReadCycles();
	uint32_t startTick = HAL_GetTick();
	uint32_t elapsedCycles;
	uint32_t elapsedUs;
	uint32_t cyclesPerUs = SystemCoreClock / 1000000U;

	for (;;)
	{
		if (OLED_TimingAvailable != 0U && cyclesPerUs != 0U)
		{
			/* DWT实时时钟包含绘制、I2C发送和DMA等待，慢帧会自动少走或跳帧。 */
			elapsedCycles = (uint32_t)(OLED_TimingReadCycles() - startCycles);
			elapsedUs = elapsedCycles / cyclesPerUs;
		}
		else
		{
			/* DWT不可用时退回RTOS毫秒时钟，动画仍按经过时间推进。 */
			elapsedUs = (uint32_t)(HAL_GetTick() - startTick) * 1000U;
		}
		if (elapsedUs >= OLED_PAGE_TRANSITION_DURATION_US)
		{
			offset = OLED_WIDTH;
		}
		else
		{
			offset = (uint8_t)(((uint32_t)OLED_WIDTH * elapsedUs) /
				OLED_PAGE_TRANSITION_DURATION_US);
		}
		if (offset == lastOffset)
		{
			/* 时间未推进到下一列才让出CPU，不给每个发送帧额外加延时。 */
			vTaskDelay(1U);
			continue;
		}
		delta = (uint8_t)(offset - lastOffset);
		OLED_TimingBeginFrame(&frameStartCycles);
		for (page = FirstPage; page < OLED_PAGE_COUNT; page++)
		{
			drawStartCycles = OLED_TimingReadCycles();
			OLED_RenderPage(TargetState, page, OLED_TargetPage);
			OLED_TimingAddDrawCycles(drawStartCycles);
			drawStartCycles = OLED_TimingReadCycles();
			OLED_ComposeTransitionPage(page, offset, delta, Entering);
			OLED_TimingAddDrawCycles(drawStartCycles);
			status = OLED_FlushPageDiff(page, OLED_NextPage);
			if (status != HAL_OK)
			{
				OLED_TimingFinishFrame(frameStartCycles, status);
				return status;
			}
			/* 页发送成功后才可作为下一帧平移的来源。 */
			OLED_CopyPageToFrame(page, OLED_NextPage);
		}
		if (offset == OLED_WIDTH)
		{
			status = OLED_RenderFixedPages(TargetState, FirstPage);
			OLED_TimingFinishFrame(frameStartCycles, status);
			if (status != HAL_OK)
			{
				return status;
			}
			return OLED_AnimatePageBounce(TargetState, Entering, FirstPage);
		}
		OLED_TimingFinishFrame(frameStartCycles, HAL_OK);
		lastOffset = offset;
	}
}

void OLED_RenderControllerUI(uint8_t Root, uint8_t Submenu,
	uint8_t Level, uint8_t FeatureSelection, uint8_t ConnectionState,
	uint8_t LocalRole, uint8_t ButtonBitmap, uint8_t MwGyroState)
{
	OLED_UIState_t nextState;
	uint8_t i;

	/* 按当前菜单变量构造目标状态，稍后由OLED_UIProcess统一绘制。 */
	nextState.Screen = OLED_UI_SCREEN_MENU;
	if (ConnectionState == CONTROLLER_LINK_STATE_CONNECTING ||
		ConnectionState == CONTROLLER_LINK_STATE_FAULT)
	{
		nextState.Screen = OLED_UI_SCREEN_LINK;
	}
	else if (Level == 0U)
	{
		nextState.Screen = OLED_UI_SCREEN_MAIN;
	}
	else if (Level == 2U &&
		(((Root == 0U || Root == 1U) && (Submenu == 0U || Submenu == 3U)) ||
		 (Root == 2U && Submenu == 0U)))
	{
		nextState.Screen = OLED_UI_SCREEN_FEATURE;
	}

	nextState.Root = Root;
	nextState.Submenu = Submenu;
	nextState.Level = Level;
	nextState.ConnectionState = ConnectionState;
	nextState.LocalRole = LocalRole;
	nextState.ButtonBitmap = ButtonBitmap;
	nextState.MwGyroState = MwGyroState;
	if (Level == 0U)
	{
		nextState.Selection = OLED_GetRootSelection(Root);
	}
	else if (Level == 1U)
	{
		nextState.Selection = OLED_GetSubmenuSelection(Submenu);
	}
	else
	{
		nextState.Selection = FeatureSelection;
	}

	taskENTER_CRITICAL();
	for (i = 0U; i < OLED_UI_VALUE_COUNT; i++)
	{
		if (nextState.Screen == OLED_UI_SCREEN_FEATURE &&
			OLED_RequestedUI.Screen == OLED_UI_SCREEN_FEATURE &&
			OLED_RequestedUI.Root == Root && OLED_RequestedUI.Submenu == Submenu)
		{
			nextState.Values[i] = OLED_RequestedUI.Values[i];
		}
		else
		{
			nextState.Values[i] = 0;
		}
	}
	OLED_RequestedUI = nextState;
	OLED_UIDirtyFlags |= OLED_UI_DIRTY_SCREEN;
	taskEXIT_CRITICAL();
}

void OLED_UpdateFeatureValue(uint8_t Index, int32_t Value)
{
	if (Index >= OLED_UI_VALUE_COUNT || OLED_CanDrawFeatureData() == 0U)
	{
		return;
	}

	taskENTER_CRITICAL();
	if (OLED_RequestedUI.Screen == OLED_UI_SCREEN_FEATURE &&
		(Index < 2U || (OLED_RequestedUI.Root == 1U &&
		 OLED_RequestedUI.Submenu == 0U)) &&
		OLED_RequestedUI.Values[Index] != Value)
	{
		OLED_RequestedUI.Values[Index] = Value;
		OLED_UIDirtyFlags |= OLED_UI_DIRTY_VALUES;
	}
	taskEXIT_CRITICAL();
}

void OLED_UIProcess(void)
{
	HAL_StatusTypeDef status = HAL_OK;
	OLED_UIState_t targetState;
	uint8_t dirtyFlags;
	uint8_t enteringFeature;
	uint8_t exitingFeature;
	uint8_t enteringSecondPage;
	uint8_t exitingSecondPage;
	uint8_t scrollUp;
	uint8_t animatedTransition = 0U;
	uint32_t currentTick;

	currentTick = HAL_GetTick();
	taskENTER_CRITICAL();
	dirtyFlags = OLED_UIDirtyFlags;
	/* 页面变化立即处理；纯数值变化按40ms节流，避免过度刷新。 */
	if (dirtyFlags == 0U ||
		((dirtyFlags & OLED_UI_DIRTY_SCREEN) == 0U &&
		(uint32_t)(currentTick - OLED_LastValueRenderTick) < OLED_UI_VALUE_PERIOD_MS))
	{
		taskEXIT_CRITICAL();
		return;
	}
	targetState = OLED_RequestedUI;
	OLED_UIDirtyFlags = 0U;
	taskEXIT_CRITICAL();

	if (OLED_UIInitialized == 0U)
	{
		status = OLED_RenderState(&targetState, 1U);
		OLED_UIInitialized = 1U;
	}
	else if (OLED_RenderedUI.ConnectionState != targetState.ConnectionState)
	{
		/* 链路变化直接重绘，让连接/停机状态不等待菜单滑动动画。 */
		status = OLED_RenderState(&targetState, 0U);
	}
	else
	{
		/* 判断新旧状态的页面类型，决定是否播放进入或退出动画。 */
		enteringFeature = targetState.Screen == OLED_UI_SCREEN_FEATURE &&
			(OLED_RenderedUI.Screen != OLED_UI_SCREEN_FEATURE ||
			 OLED_RenderedUI.Root != targetState.Root ||
			 OLED_RenderedUI.Submenu != targetState.Submenu) ? 1U : 0U;
		exitingFeature = OLED_RenderedUI.Screen == OLED_UI_SCREEN_FEATURE &&
			(targetState.Screen != OLED_UI_SCREEN_FEATURE ||
			 OLED_RenderedUI.Root != targetState.Root ||
			 OLED_RenderedUI.Submenu != targetState.Submenu) ? 1U : 0U;
		enteringSecondPage = targetState.Screen == OLED_UI_SCREEN_MENU &&
			targetState.Level == 1U &&
			OLED_RenderedUI.Screen == OLED_UI_SCREEN_MAIN ? 1U : 0U;
		exitingSecondPage = targetState.Screen == OLED_UI_SCREEN_MAIN &&
			OLED_RenderedUI.Screen == OLED_UI_SCREEN_MENU &&
			OLED_RenderedUI.Level == 1U ? 1U : 0U;
		if (enteringFeature != 0U || exitingFeature != 0U)
		{
			/* 功能页的标题区占前两页，动画只移动下方内容区。 */
			animatedTransition = 1U;
			status = OLED_AnimatePageTransition(&targetState, enteringFeature, 2U);
		}
		else if (enteringSecondPage != 0U || exitingSecondPage != 0U)
		{
			/* 主菜单和二级菜单切换只滑动正文，标题区在末尾补刷。 */
			animatedTransition = 1U;
			status = OLED_AnimatePageTransition(&targetState, enteringSecondPage, 2U);
		}
		else if (OLED_GetMenuScrollDirection(&OLED_RenderedUI,
			&targetState, &scrollUp) != 0U)
		{
			/* 同级上下选择时固定高亮行，只滚动菜单选项。 */
			animatedTransition = 1U;
			status = OLED_AnimateMenuScroll(&targetState, scrollUp);
		}
		/* 动画末帧已准确显示目标；普通刷新继续沿用原来的差异刷新路径。 */
		if (status == HAL_OK && animatedTransition == 0U)
		{
			status = OLED_RenderState(&targetState, 0U);
		}
	}

	if (status != HAL_OK)
	{
		/* 出错后停止当前动画，下轮按最新状态全刷，修复部分发送的页面。 */
		OLED_UIInitialized = 0U;
		taskENTER_CRITICAL();
		OLED_UIDirtyFlags |= OLED_UI_DIRTY_SCREEN;
		taskEXIT_CRITICAL();
		return;
	}

	OLED_RenderedUI = targetState;
	OLED_LastValueRenderTick = currentTick;
}

/**
  * @brief  OLED初始化
  * @param  无
  * @retval 无
  */
void OLED_Init(void)
{
	uint32_t i, j;
	OLED_TimingInit();
	/* 初始化时先关闭DMA，清屏结束后再启用任务态DMA发送。 */
	OLED_DMAEnabled = 0U;
	xOLEDTransferSemaphore = xSemaphoreCreateBinary();
	if (xOLEDTransferSemaphore == NULL)
	{
		Error_Handler();
	}
	
	for (i = 0; i < 1000; i++)			//上电延时
	{
		for (j = 0; j < 1000; j++);
	}
	

	
	OLED_WriteCommand(0xAE);	//关闭显示
	
	OLED_WriteCommand(0xD5);	//设置显示时钟分频比/振荡器频率
	OLED_WriteCommand(0x80);
	
	OLED_WriteCommand(0xA8);	//设置多路复用率
	OLED_WriteCommand(0x3F);
	
	OLED_WriteCommand(0xD3);	//设置显示偏移
	OLED_WriteCommand(0x00);
	
	OLED_WriteCommand(0x40);	//设置显示开始行
	
	OLED_WriteCommand(0xA1);	//设置左右方向，0xA1正常 0xA0左右反置
	
	OLED_WriteCommand(0xC8);	//设置上下方向，0xC8正常 0xC0上下反置

	OLED_WriteCommand(0xDA);	//设置COM引脚硬件配置
	OLED_WriteCommand(0x12);
	
	OLED_WriteCommand(0x81);	//设置对比度控制
	OLED_WriteCommand(0xCF);

	OLED_WriteCommand(0xD9);	//设置预充电周期
	OLED_WriteCommand(0xF1);

	OLED_WriteCommand(0xDB);	//设置VCOMH取消选择级别
	OLED_WriteCommand(0x30);

	OLED_WriteCommand(0xA4);	//设置整个显示打开/关闭

	OLED_WriteCommand(0xA6);	//设置正常/倒转显示

	OLED_WriteCommand(0x8D);	//设置充电泵
	OLED_WriteCommand(0x14);

	OLED_WriteCommand(0xAF);	//开启显示
	
	
	xOLEDMutex=xSemaphoreCreateMutex();//创建互斥锁
	if (xOLEDMutex == NULL)
	{
		Error_Handler();
	}
		
	OLED_Clear();				//OLED清屏
	OLED_DMAEnabled = 1U;
	

	
}
