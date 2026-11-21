#include "MYdef.h"
#include "controller_config.h"
#include "controller_mode.h"
#include "handset_link.h"
#include "fun.h"

/* TASK层把采样、按键、控制和链路任务通过快照/事件接起来。 */
#define CTRL_TASK_PERIOD_TICKS       pdMS_TO_TICKS(CONTROLLER_CONTROL_PERIOD_MS)
#define JOYSTICK_SAMPLE_PERIOD_TICKS pdMS_TO_TICKS(CONTROLLER_JOYSTICK_SAMPLE_PERIOD_MS)
#define GYRO_SAMPLE_PERIOD_TICKS     pdMS_TO_TICKS(CONTROLLER_GYRO_SAMPLE_PERIOD_MS)

/* 手柄到接收板再到小车的固定二进制帧，不带CRC。 */
#define RADIO_CONTROL_FRAME_SIZE       16U
#define RADIO_CONTROL_FRAME_START      0xFFU
#define RADIO_CONTROL_FRAME_END        0xFEU
#define RADIO_CONTROL_FRAME_VERSION    0x01U
#define RADIO_CONTROL_SOURCE_LEFT      0U
#define RADIO_CONTROL_SOURCE_RIGHT     1U
#define RADIO_CONTROL_MODE_STOP        0U
#define RADIO_CONTROL_MODE_SPEED       1U
#define RADIO_CONTROL_MODE_RA          2U
#define RADIO_CONTROL_MODE_MW           3U


/* 菜单状态：KEYTASK 更新，CTRLTASK 根据状态输出控制命令。 */
volatile uint8_t DoubleModle = 0;
volatile int8_t Floor1 = 0;
volatile int8_t Floor2 = 0;
volatile int8_t Floor3 = 0;
volatile int8_t Floor4 = 0;
volatile int8_t FloorSwitch = 0;

FREQ2_4GQueueHandle_Data TX_Data;
static controller_input_snapshot_t gLocalInputSnapshot;
static controller_peer_input_t gPeerInputSnapshot;
static uint16_t gJoystickSampleSequence;
static uint16_t gGyroSampleSequence;
static int16_t gDisplayValues[4];
static uint8_t gDisplayValueValidMask;
static volatile uint8_t gRequiresCenterBeforeEnable;
static volatile uint8_t gLocalJoystickCentered;
static volatile uint8_t gHasSeenMerge;

void TASK_GetMenuSnapshot(task_menu_snapshot_t *snapshot)
{
    if (snapshot == NULL)
    {
		return;
	}

	memset(snapshot, 0, sizeof(*snapshot));
	taskENTER_CRITICAL();
    snapshot->Floor1 = Floor1;
    snapshot->Floor2 = Floor2;
    snapshot->Floor3 = Floor3;
    snapshot->Floor4 = Floor4;
    snapshot->FloorSwitch = FloorSwitch;
    snapshot->DoubleModle = DoubleModle;
	snapshot->ControlEnabled = (FloorSwitch == 2 && Floor3 != 0 &&
		((Floor1 == 0 || Floor1 == 1) || (Floor1 == 2 && Floor2 == 0)) &&
		(Floor2 == 0 || Floor2 == 3) &&
		ControllerMode_CanControl() != 0U && gRequiresCenterBeforeEnable == 0U) ? 1U : 0U;
	snapshot->ConnectionState = (uint8_t)ControllerMode_GetState();
	snapshot->DisplayValueValidMask = gDisplayValueValidMask;
	memcpy(snapshot->DisplayValue, gDisplayValues, sizeof(gDisplayValues));
	taskEXIT_CRITICAL();
}

/* 采样任务发布最新值，不用队列逐个排队；控制任务总是读取最近一次样本。 */

void TASK_PublishJoystickSnapshot(float x_percent, float y_percent, uint32_t sample_tick)
{
	taskENTER_CRITICAL();
	gLocalInputSnapshot.joystick_x_percent = x_percent;
	gLocalInputSnapshot.joystick_y_percent = y_percent;
	gLocalInputSnapshot.joystick_sample_tick = sample_tick;
	gLocalInputSnapshot.joystick_sample_sequence = ++gJoystickSampleSequence;
	gLocalInputSnapshot.joystick_valid = 1U;
	taskEXIT_CRITICAL();
}

void TASK_PublishGyroSnapshot(float roll_deg, float pitch_deg, float yaw_deg,
	float height_m, uint32_t sample_tick)
{
	taskENTER_CRITICAL();
	gLocalInputSnapshot.gyro_roll_deg = roll_deg;
	gLocalInputSnapshot.gyro_pitch_deg = pitch_deg;
	gLocalInputSnapshot.gyro_yaw_deg = yaw_deg;
	gLocalInputSnapshot.gyro_height_m = height_m;
	gLocalInputSnapshot.gyro_sample_tick = sample_tick;
	gLocalInputSnapshot.gyro_sample_sequence = ++gGyroSampleSequence;
	gLocalInputSnapshot.gyro_valid = 1U;
	taskEXIT_CRITICAL();
}

void TASK_PublishButtonSnapshot(uint8_t button_bitmap, uint8_t event_key,
	uint16_t event_sequence)
{
	taskENTER_CRITICAL();
	gLocalInputSnapshot.button_bitmap = button_bitmap;
	if (event_key != CONTROLLER_KEY_NONE)
	{
		gLocalInputSnapshot.button_event_key = event_key;
		gLocalInputSnapshot.button_event_sequence = event_sequence;
	}
	taskEXIT_CRITICAL();
}

void TASK_ClearButtonEvent(void)
{
	taskENTER_CRITICAL();
	gLocalInputSnapshot.button_event_key = CONTROLLER_KEY_NONE;
	taskEXIT_CRITICAL();
}

uint8_t TASK_AcknowledgeButtonEvent(uint16_t event_sequence)
{
	uint8_t matched;
	taskENTER_CRITICAL();
	matched = (gLocalInputSnapshot.button_event_sequence == event_sequence) ? 1U : 0U;
	if (matched != 0U)
	{
		gLocalInputSnapshot.button_event_key = CONTROLLER_KEY_NONE;
	}
	taskEXIT_CRITICAL();
	return matched;
}

/* 读取本机采样快照；返回成功只表示拷贝完成，是否过期由调用方再判断。 */
uint8_t TASK_GetInputSnapshot(controller_input_snapshot_t *snapshot)
{
	if (snapshot == NULL)
	{
		return 0U;
	}

	taskENTER_CRITICAL();
	*snapshot = gLocalInputSnapshot;
	taskEXIT_CRITICAL();
	return 1U;
}

/* 读取串口收到的远端快照；当前KEYTASK读其中按键事件，控制函数读本机快照。 */
uint8_t TASK_GetPeerInputSnapshot(controller_peer_input_t *snapshot)
{
	TickType_t now;
	if (snapshot == NULL)
	{
		return 0U;
	}
	taskENTER_CRITICAL();
	*snapshot = gPeerInputSnapshot;
	taskEXIT_CRITICAL();
	now = xTaskGetTickCount();
	if ((TickType_t)(now - (TickType_t)snapshot->last_frame_tick) >=
		pdMS_TO_TICKS(CONTROLLER_INPUT_TIMEOUT_MS))
	{
		snapshot->input.joystick_valid = 0U;
		snapshot->input.gyro_valid = 0U;
		snapshot->input.button_bitmap = 0U;
		snapshot->input.button_event_key = CONTROLLER_KEY_NONE;
	}
	else
	{
		if ((TickType_t)(now - (TickType_t)snapshot->input.joystick_sample_tick) >=
			pdMS_TO_TICKS(CONTROLLER_INPUT_TIMEOUT_MS))
		{
			snapshot->input.joystick_valid = 0U;
		}
		if ((TickType_t)(now - (TickType_t)snapshot->input.gyro_sample_tick) >=
			pdMS_TO_TICKS(CONTROLLER_INPUT_TIMEOUT_MS))
		{
			snapshot->input.gyro_valid = 0U;
		}
	}
	return 1U;
}

void TASK_ResetPeerInput(void)
{
	taskENTER_CRITICAL();
	memset(&gPeerInputSnapshot, 0, sizeof(gPeerInputSnapshot));
	taskEXIT_CRITICAL();
}

/* LEFT保存RIGHT上传的数据；只有样本序号变化才刷新该输入的新鲜度时间。 */
void TASK_PublishPeerInput(const controller_input_snapshot_t *snapshot)
{
	TickType_t now = xTaskGetTickCount();
	uint8_t updated = 0U;
	if (snapshot == NULL)
	{
		return;
	}
	taskENTER_CRITICAL();
	gPeerInputSnapshot.last_frame_tick = (uint32_t)now;
	if (snapshot->joystick_valid == 0U)
	{
		gPeerInputSnapshot.input.joystick_valid = 0U;
	}
	else if (gPeerInputSnapshot.input.joystick_valid == 0U ||
		snapshot->joystick_sample_sequence !=
		gPeerInputSnapshot.input.joystick_sample_sequence)
	{
		gPeerInputSnapshot.input.joystick_x_percent = snapshot->joystick_x_percent;
		gPeerInputSnapshot.input.joystick_y_percent = snapshot->joystick_y_percent;
		gPeerInputSnapshot.input.joystick_sample_sequence = snapshot->joystick_sample_sequence;
		gPeerInputSnapshot.input.joystick_sample_tick = (uint32_t)now;
		gPeerInputSnapshot.input.joystick_valid = 1U;
		updated = 1U;
	}
	if (snapshot->gyro_valid == 0U)
	{
		gPeerInputSnapshot.input.gyro_valid = 0U;
	}
	else if (gPeerInputSnapshot.input.gyro_valid == 0U ||
		snapshot->gyro_sample_sequence != gPeerInputSnapshot.input.gyro_sample_sequence)
	{
		gPeerInputSnapshot.input.gyro_roll_deg = snapshot->gyro_roll_deg;
		gPeerInputSnapshot.input.gyro_pitch_deg = snapshot->gyro_pitch_deg;
		gPeerInputSnapshot.input.gyro_yaw_deg = snapshot->gyro_yaw_deg;
		gPeerInputSnapshot.input.gyro_height_m = snapshot->gyro_height_m;
		gPeerInputSnapshot.input.gyro_sample_sequence = snapshot->gyro_sample_sequence;
		gPeerInputSnapshot.input.gyro_sample_tick = (uint32_t)now;
		gPeerInputSnapshot.input.gyro_valid = 1U;
		updated = 1U;
	}
	gPeerInputSnapshot.input.button_bitmap = snapshot->button_bitmap;
	gPeerInputSnapshot.input.button_event_key = snapshot->button_event_key;
	gPeerInputSnapshot.input.button_event_sequence = snapshot->button_event_sequence;
	if (updated != 0U)
	{
		gPeerInputSnapshot.last_input_update_tick = (uint32_t)now;
	}
	taskEXIT_CRITICAL();
}

void TASK_PublishFeatureValue(uint8_t index, float value)
{
	int32_t rounded;
	if (index >= 4U || value != value)
	{
		return;
	}
	if (value > 32767.0f)
	{
		rounded = 32767;
	}
	else if (value < -32768.0f)
	{
		rounded = -32768;
	}
	else
	{
		rounded = (int32_t)(value >= 0.0f ? value + 0.5f : value - 0.5f);
	}
	taskENTER_CRITICAL();
	gDisplayValues[index] = (int16_t)rounded;
	gDisplayValueValidMask |= (uint8_t)(1U << index);
	taskEXIT_CRITICAL();
	OLED_UpdateFeatureValue(index, rounded);
}

void TASK_ClearFeatureValues(void)
{
	uint8_t index;
	taskENTER_CRITICAL();
	memset(gDisplayValues, 0, sizeof(gDisplayValues));
	gDisplayValueValidMask = 0U;
	taskEXIT_CRITICAL();
	for (index = 0U; index < 4U; index++)
	{
		OLED_UpdateFeatureValue(index, 0);
	}
}

/* 仅RIGHT在DUAL时应用LEFT发来的菜单/显示状态；LEFT始终是菜单状态主控端。 */
void TASK_SetRemoteMenuSnapshot(const task_menu_snapshot_t *snapshot)
{
	uint8_t menuChanged;
	uint8_t index;
	if (snapshot == NULL || CONTROLLER_LOCAL_ROLE != CONTROLLER_ROLE_RIGHT ||
		ControllerMode_GetState() != CONTROLLER_LINK_STATE_DUAL)
	{
		return;
	}
	taskENTER_CRITICAL();
	menuChanged = (Floor1 != snapshot->Floor1 || Floor2 != snapshot->Floor2 ||
		Floor3 != snapshot->Floor3 || Floor4 != snapshot->Floor4 ||
		FloorSwitch != snapshot->FloorSwitch || DoubleModle != snapshot->DoubleModle) ? 1U : 0U;
	Floor1 = snapshot->Floor1;
	Floor2 = snapshot->Floor2;
	Floor3 = snapshot->Floor3;
	Floor4 = snapshot->Floor4;
	FloorSwitch = snapshot->FloorSwitch;
	DoubleModle = snapshot->DoubleModle;
	memcpy(gDisplayValues, snapshot->DisplayValue, sizeof(gDisplayValues));
	gDisplayValueValidMask = snapshot->DisplayValueValidMask;
	taskEXIT_CRITICAL();
	if (menuChanged != 0U)
	{
		OLED_Ctrl();
	}
	for (index = 0U; index < 4U; index++)
	{
		OLED_UpdateFeatureValue(index,
			(snapshot->DisplayValueValidMask & (uint8_t)(1U << index)) != 0U ?
			snapshot->DisplayValue[index] : 0);
	}
}

static uint8_t OLED_IsFeatureScreenActive(void)
{
    task_menu_snapshot_t menuState;

    TASK_GetMenuSnapshot(&menuState);
    return ((menuState.DoubleModle == 0U ||
		menuState.ConnectionState == CONTROLLER_LINK_STATE_DUAL) &&
		menuState.FloorSwitch == 2 &&
        (menuState.Floor1 == 0 || menuState.Floor1 == 1) &&
        (menuState.Floor2 == 0 || menuState.Floor2 == 3) &&
        OLED_FeatureClearPending == 0U) ? 1U : 0U;
}

static void ApplyMenuKey(uint8_t key)
{
	uint8_t controlEnableAllowed = (ControllerMode_CanControl() != 0U &&
		(gRequiresCenterBeforeEnable == 0U || gLocalJoystickCentered != 0U)) ? 1U : 0U;
	taskENTER_CRITICAL();
	switch (key)
	{
		case CONTROLLER_KEY_U:
			switch (FloorSwitch)
			{
				case 0: Floor1 = (int8_t)((Floor1 + 1) % 3); break;
				case 1: Floor2 = (int8_t)((Floor2 + 1) % 4); break;
				case 2:
					if (controlEnableAllowed == 0U)
					{
						Floor3 = 0;
					}
					else
					{
						Floor3 = (int8_t)((Floor3 + 1) % 2);
						if (Floor3 != 0) gRequiresCenterBeforeEnable = 0U;
					}
					break;
				default: break;
			}
			break;
		case CONTROLLER_KEY_D:
			switch (FloorSwitch)
			{
				case 0: Floor1 = (Floor1 > 0) ? (int8_t)(Floor1 - 1) : 2; break;
				case 1: Floor2 = (Floor2 > 0) ? (int8_t)(Floor2 - 1) : 3; break;
				case 2:
					if (controlEnableAllowed == 0U)
					{
						Floor3 = 0;
					}
					else
					{
						Floor3 = (Floor3 > 0) ? (int8_t)(Floor3 - 1) : 1;
						if (Floor3 != 0) gRequiresCenterBeforeEnable = 0U;
					}
					break;
				default: break;
			}
			break;
		case CONTROLLER_KEY_L:
			if (FloorSwitch == 2)
			{
				Floor3 = 0;
			}
			else if (FloorSwitch == 1)
			{
				Floor2 = 0;
			}
			FloorSwitch = (FloorSwitch > 0) ? (int8_t)(FloorSwitch - 1) : 0;
			break;
		case CONTROLLER_KEY_R:
			FloorSwitch++;
			if (Floor2 == 1)
			{
				if (FloorSwitch > 3) FloorSwitch = 3;
			}
			else if (FloorSwitch > 2)
			{
				FloorSwitch = 2;
			}
			break;
		default:
			taskEXIT_CRITICAL();
			return;
	}
	taskEXIT_CRITICAL();
	OLED_Ctrl();
}

static uint8_t ReadButtonBitmap(void)
{
	uint8_t bitmap = 0U;
	/* PA8是独立速度设定键；R/D事件在RIGHT合体状态下映射为启停。 */
	if (HAL_GPIO_ReadPin(Key_UP_GPIO_Port, Key_UP_Pin) == GPIO_PIN_RESET) bitmap |= CONTROLLER_BUTTON_PA8;
	if (HAL_GPIO_ReadPin(Key_U_GPIO_Port, Key_U_Pin) == GPIO_PIN_RESET) bitmap |= CONTROLLER_BUTTON_U;
	if (HAL_GPIO_ReadPin(Key_D_GPIO_Port, Key_D_Pin) == GPIO_PIN_RESET) bitmap |= CONTROLLER_BUTTON_D;
	if (HAL_GPIO_ReadPin(Key_L_GPIO_Port, Key_L_Pin) == GPIO_PIN_RESET) bitmap |= CONTROLLER_BUTTON_L;
	if (HAL_GPIO_ReadPin(Key_R_GPIO_Port, Key_R_Pin) == GPIO_PIN_RESET) bitmap |= CONTROLLER_BUTTON_R;
	return bitmap;
}

static uint8_t FirstNewKey(uint8_t new_buttons)
{
	/* PA8边沿走可靠事件；RIGHT合体时R/D映射由KEYTASK按角色处理。 */
	if ((new_buttons & CONTROLLER_BUTTON_PA8) != 0U) return CONTROLLER_KEY_PA8;
	if ((new_buttons & CONTROLLER_BUTTON_U) != 0U) return CONTROLLER_KEY_U;
	if ((new_buttons & CONTROLLER_BUTTON_D) != 0U) return CONTROLLER_KEY_D;
	if ((new_buttons & CONTROLLER_BUTTON_L) != 0U) return CONTROLLER_KEY_L;
	if ((new_buttons & CONTROLLER_BUTTON_R) != 0U) return CONTROLLER_KEY_R;
	return CONTROLLER_KEY_NONE;
}

static void StartKeyDebounce(void)
{
	Key_Lock = 1U;
	(void)xTimerStart(KEY_LOCKTimerHandle, portMAX_DELAY);
}

/**
 * @brief 周期采集按键状态；右侧合体时只上报事件，不在本机改菜单。
 */
/* DUAL时RIGHT只上报按键；LEFT按事件序号去重并回ACK，重复帧只补发ACK。 */
void KEYTASK(void *argument)
{
	static uint8_t previous_buttons;
	static uint16_t local_event_sequence;
	static uint16_t last_peer_event_sequence;
	static uint32_t last_peer_ack_frame_tick;
	static uint8_t first_sample = 1U;
	uint8_t buttons;
	uint8_t newButtons;
	uint8_t new_key;
	controller_peer_input_t peerInput;
	controller_mode_snapshot_t modeSnapshot;
	controller_link_state_t linkState;
	(void)argument;

	for (;;)
	{
		(void)xEventGroupWaitBits(KEY_StartEventGroupHandle, (1U << 0), pdTRUE,
			pdTRUE, pdMS_TO_TICKS(CONTROLLER_BUTTON_POLL_PERIOD_MS));
		buttons = ReadButtonBitmap();
		if (first_sample != 0U)
		{
			previous_buttons = buttons;
			first_sample = 0U;
			TASK_PublishButtonSnapshot(buttons, CONTROLLER_KEY_NONE, local_event_sequence);
			continue;
		}
		newButtons = (uint8_t)(buttons & (uint8_t)~previous_buttons);
		linkState = ControllerMode_GetState();
		new_key = FirstNewKey(newButtons);
		if (linkState == CONTROLLER_LINK_STATE_DUAL &&
			CONTROLLER_LOCAL_ROLE == CONTROLLER_ROLE_RIGHT)
		{
			/* D取消自旋优先；R开启实时自旋；PA8进入/确认恒速设定。 */
		if ((buttons & CONTROLLER_BUTTON_D) != 0U)
			{
			new_key = (newButtons & CONTROLLER_BUTTON_D) != 0U ?
				CONTROLLER_KEY_SPIN_STOP : CONTROLLER_KEY_NONE;
			}
			else if ((newButtons & CONTROLLER_BUTTON_R) != 0U)
			{
				new_key = CONTROLLER_KEY_SPIN_START;
			}
			else if ((newButtons & CONTROLLER_BUTTON_PA8) != 0U)
			{
				new_key = CONTROLLER_KEY_PA8;
			}
		}
		previous_buttons = buttons;

		if (new_key != CONTROLLER_KEY_NONE && Key_Lock == 0U)
		{
			local_event_sequence++;
			TASK_PublishButtonSnapshot(buttons, new_key, local_event_sequence);
			if (linkState == CONTROLLER_LINK_STATE_SOLO ||
				(linkState == CONTROLLER_LINK_STATE_DUAL &&
				CONTROLLER_LOCAL_ROLE == CONTROLLER_ROLE_LEFT))
			{
				ApplyMenuKey(new_key);
			}
			StartKeyDebounce();
		}
		else
		{
			TASK_PublishButtonSnapshot(buttons, CONTROLLER_KEY_NONE, local_event_sequence);
		}
		if (linkState != CONTROLLER_LINK_STATE_DUAL ||
			CONTROLLER_LOCAL_ROLE != CONTROLLER_ROLE_LEFT)
		{
			if (TASK_GetPeerInputSnapshot(&peerInput) != 0U)
			{
				last_peer_event_sequence = peerInput.input.button_event_sequence;
			}
		}
		else if (TASK_GetPeerInputSnapshot(&peerInput) != 0U &&
			peerInput.input.button_event_key != CONTROLLER_KEY_NONE &&
			peerInput.input.button_event_sequence != last_peer_event_sequence &&
			Key_Lock == 0U)
		{
			last_peer_event_sequence = peerInput.input.button_event_sequence;
			ApplyMenuKey(peerInput.input.button_event_key);
			ControllerMode_GetSnapshot(&modeSnapshot);
			(void)HandsetLink_QueueKeyEventAck(peerInput.input.button_event_sequence,
				modeSnapshot.connection_id);
			last_peer_ack_frame_tick = peerInput.last_frame_tick;
			StartKeyDebounce();
		}
		else if (linkState == CONTROLLER_LINK_STATE_DUAL &&
			CONTROLLER_LOCAL_ROLE == CONTROLLER_ROLE_LEFT &&
			TASK_GetPeerInputSnapshot(&peerInput) != 0U &&
			peerInput.input.button_event_key != CONTROLLER_KEY_NONE &&
			peerInput.input.button_event_sequence == last_peer_event_sequence &&
			peerInput.last_frame_tick != last_peer_ack_frame_tick)
		{
			ControllerMode_GetSnapshot(&modeSnapshot);
			(void)HandsetLink_QueueKeyEventAck(peerInput.input.button_event_sequence,
				modeSnapshot.connection_id);
			last_peer_ack_frame_tick = peerInput.last_frame_tick;
		}
	}
}

/**
 * @brief 根据当前菜单选择更新控制通道状态。
 */
/* 状态切换先清无线旧命令和远端快照，再退出控制使能，避免旧数据跨模式生效。 */
void CTRLTASK(void *argument)
{
    TickType_t lastWakeTime = xTaskGetTickCount();
    task_menu_snapshot_t menuState;
    controller_mode_snapshot_t modeSnapshot;
    controller_link_state_t previousLinkState = CONTROLLER_LINK_STATE_CONNECTING;
    controller_link_state_t currentLinkState;
    uint8_t modeOrContactChanged;
    controller_input_snapshot_t localInput;

    while (1)
    {
        /* 控制周期固定为5ms；读取最新快照，不等待输入队列。 */
        vTaskDelayUntil(&lastWakeTime, CTRL_TASK_PERIOD_TICKS);
		modeOrContactChanged = ControllerMode_UpdateContact(
            HAL_GPIO_ReadPin(Key_MergeInput_GPIO_Port, Key_MergeInput_Pin) == GPIO_PIN_RESET,
            xTaskGetTickCount());
        ControllerMode_GetSnapshot(&modeSnapshot);
        currentLinkState = modeSnapshot.state;
		if (modeSnapshot.contact_active != 0U)
		{
			gHasSeenMerge = 1U;
		}
		else if (currentLinkState == CONTROLLER_LINK_STATE_SOLO &&
			previousLinkState != CONTROLLER_LINK_STATE_SOLO && gHasSeenMerge != 0U)
		{
			gRequiresCenterBeforeEnable = 1U;
			gLocalJoystickCentered = 0U;
			gHasSeenMerge = 0U;
		}
		if (TASK_GetInputSnapshot(&localInput) != 0U && localInput.joystick_valid != 0U)
		{
			gLocalJoystickCentered =
				(localInput.joystick_x_percent >= -5.0f && localInput.joystick_x_percent <= 5.0f &&
				 localInput.joystick_y_percent >= -5.0f && localInput.joystick_y_percent <= 5.0f) ? 1U : 0U;
		}
        taskENTER_CRITICAL();
        DoubleModle = modeSnapshot.contact_active;
        if (currentLinkState != previousLinkState)
        {
            /* 合体、断开和故障恢复均先退出旧控制选择，需重新明确使能。 */
            Floor3 = 0;
            Floor4 = 0;
        }
        taskEXIT_CRITICAL();
        if (currentLinkState != previousLinkState)
        {
            (void)xQueueReset(FREQ2_4G_DataQueueHandle);
            TASK_ResetPeerInput();
            TASK_ClearButtonEvent();
            TASK_ClearFeatureValues();
            OLED_Ctrl();
            previousLinkState = currentLinkState;
        }
		else if (modeOrContactChanged != 0U)
		{
			OLED_Ctrl();
		}
        TASK_GetMenuSnapshot(&menuState);

        if (ControllerMode_CanControl() != 0U)
        {
            switch (menuState.Floor1)
            {
                case 0:
                    if (menuState.Floor2 == 0 && menuState.FloorSwitch == 2)
                    {
                        if (menuState.Floor3 == 0) SP_JS_OFF(); else SP_JS_ON();
                    }
                    else if (menuState.Floor2 == 3 && menuState.FloorSwitch == 2)
                    {
                        if (menuState.Floor3 == 0) SP_GYRO_OFF(); else SP_GYRO_ON();
                    }
                    break;
                case 1:
                    if (menuState.Floor2 == 0 && menuState.FloorSwitch == 2)
                    {
                        if (menuState.Floor3 == 0) RA_JS_OFF(); else RA_JS_ON();
                    }
                    else if (menuState.Floor2 == 3 && menuState.FloorSwitch == 2)
                    {
                        if (menuState.Floor3 == 0) RA_GYRO_OFF(); else RA_GYRO_ON();
                    }
                    break;
                case 2:
                    if (menuState.Floor2 == 0 && menuState.FloorSwitch == 2)
                    {
                        if (menuState.Floor3 == 0) MW_JS_OFF(); else MW_JS_ON();
                    }
                    break;
                default:
                    break;
            }
        }

        if (OLED_FeatureClearPending != 0U)
        {
            OLED_ClearFeatureData();
            TASK_ClearFeatureValues();
            OLED_FeatureClearPending = 0U;
        }
    }
}

/**
 * @brief 处理无线回传数据并发送控制帧。
 */
/* 保持原2.4G帧长度；DUAL由LEFT独占发送，RIGHT失去许可时仅尝试一次停机帧。 */
/* 控制菜单明确启用后才标记速度、RA或MW模式；其余状态统一发停止模式。 */
static uint8_t GetRadioControlMode(void)
{
    task_menu_snapshot_t menuState;

    TASK_GetMenuSnapshot(&menuState);
    if (menuState.ControlEnabled == 0U)
    {
        return RADIO_CONTROL_MODE_STOP;
    }
    if (menuState.Floor1 == 0)
    {
        return RADIO_CONTROL_MODE_SPEED;
    }
    if (menuState.Floor1 == 1)
    {
        return RADIO_CONTROL_MODE_RA;
    }
    if (menuState.Floor1 == 2)
    {
        return RADIO_CONTROL_MODE_MW;
    }
    return RADIO_CONTROL_MODE_STOP;
}

/* 每次从零构帧，清掉非当前模式字段，避免旧模式数据残留到小车端。 */
static void BuildRadioControlFrame(uint8_t *frame,
    const FREQ2_4GQueueHandle_Data *command, uint8_t mode, uint8_t sequence)
{
    memset(frame, 0, RADIO_CONTROL_FRAME_SIZE);
    frame[0] = RADIO_CONTROL_FRAME_START;
    frame[1] = RADIO_CONTROL_FRAME_VERSION;
    frame[2] = RADIO_CONTROL_FRAME_SIZE;
    frame[3] = (CONTROLLER_LOCAL_ROLE == CONTROLLER_ROLE_LEFT) ?
        RADIO_CONTROL_SOURCE_LEFT : RADIO_CONTROL_SOURCE_RIGHT;
    frame[4] = sequence;
    frame[5] = mode;
    /* frame[6]保留为0；不同模式复用7~14，未使用字段保持为0。 */

    if (mode == RADIO_CONTROL_MODE_SPEED)
    {
        frame[7] = command->Data_1; /* 左侧速度，int8_t原始补码 */
        frame[8] = command->Data_2; /* 右侧速度，int8_t原始补码 */
    }
    else if (mode == RADIO_CONTROL_MODE_MW)
    {
        frame[7] = command->Data_1; /* MW横移 x，正值向右 */
        frame[8] = command->Data_2; /* MW前后 y，正值向前 */
        frame[14] = command->Data_10; /* MW自转 z，正值俯视逆时针 */
    }
    else if (mode == RADIO_CONTROL_MODE_RA)
    {
        frame[9] = command->Data_4;  /* 高度目标高字节 */
        frame[10] = command->Data_5; /* 高度目标低字节 */
        frame[11] = command->Data_6; /* 高度速度 */
        frame[12] = command->Data_8; /* 角度目标高字节 */
        frame[13] = command->Data_9; /* 角度目标低字节 */
        frame[14] = command->Data_10;/* 角度速度，int8_t原始补码 */
    }
    frame[15] = RADIO_CONTROL_FRAME_END;
}

void FREQ2_4GTask(void *argument)
{
    FREQ2_4G_SendInit();

    FREQ2_4GQueueHandle_Data RX_Data = {0};
    uint8_t TX[RADIO_CONTROL_FRAME_SIZE] = {0};
    uint8_t RX_BUFF[100];
    uint8_t RSSI;
    uint8_t lossPack;
    uint8_t radioSequence = 0U;
    uint8_t commandMode;
    uint8_t radioWasAllowed = 1U;
    SendClc_Data Data;

    Data.RXData = RX_BUFF;
    Data.TXData = TX;
    Data.TXLen = RADIO_CONTROL_FRAME_SIZE;

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(5U));
        if (ControllerMode_CanSendRadio() == 0U)
        {
            if (radioWasAllowed != 0U)
            {
                /* 失去无线发送许可时只发一次全字段清零的STOP帧。 */
                BuildRadioControlFrame(TX, &RX_Data, RADIO_CONTROL_MODE_STOP,
                    radioSequence++);
                if (FREQ2_4G_SendClc(&Data) == HAL_OK &&
					CONTROLLER_LOCAL_ROLE == CONTROLLER_ROLE_RIGHT)
				{
					ControllerMode_NotifyRadioStopped();
				}
                radioWasAllowed = 0U;
            }
            (void)xQueueReset(FREQ2_4G_DataQueueHandle);
            continue;
        }
        radioWasAllowed = 1U;
        if (xQueueReceive(
            FREQ2_4G_DataQueueHandle,
            &RX_Data,
            pdMS_TO_TICKS(5U)) != pdPASS)
        {
            continue;
        }
        /* 队列等待期间也检查接管状态，丢弃已排队的旧运动命令。 */
        if (ControllerMode_CanSendRadio() == 0U)
        {
            (void)xQueueReset(FREQ2_4G_DataQueueHandle);
            continue;
        }

        /* 限制前两个有符号控制量的范围。 */
        if ((int8_t)RX_Data.Data_1 > 100)
        {
            RX_Data.Data_1 = 100;
        }
        else if ((int8_t)RX_Data.Data_1 < -100)
        {
            RX_Data.Data_1 = (uint8_t)(int8_t)(-100);
        }

        if ((int8_t)RX_Data.Data_2 > 100)
        {
            RX_Data.Data_2 = 100;
        }
        else if ((int8_t)RX_Data.Data_2 < -100)
        {
            RX_Data.Data_2 = (uint8_t)(int8_t)(-100);
        }

        if (OLED_IsFeatureScreenActive() != 0U)
        {
            OLED_ShowSmoothSignedNum(2, 12, (int8_t)RX_Data.Data_6, 4);
            OLED_ShowSmoothSignedNum(3, 12, (int8_t)RX_Data.Data_10, 4);
            TASK_PublishFeatureValue(2U, (int8_t)RX_Data.Data_6);
            TASK_PublishFeatureValue(3U, (int8_t)RX_Data.Data_10);
        }

		commandMode = GetRadioControlMode();
		BuildRadioControlFrame(TX, &RX_Data, commandMode, radioSequence++);
		if (ControllerMode_CanSendRadio() == 0U)
		{
			BuildRadioControlFrame(TX, &RX_Data, RADIO_CONTROL_MODE_STOP,
				radioSequence++);
			if (FREQ2_4G_SendClc(&Data) == HAL_OK &&
				CONTROLLER_LOCAL_ROLE == CONTROLLER_ROLE_RIGHT)
			{
				ControllerMode_NotifyRadioStopped();
			}
			radioWasAllowed = 0U;
			(void)xQueueReset(FREQ2_4G_DataQueueHandle);
			continue;
		}

        FREQ2_4G_SendClc(&Data);
        FREQ2_4G_GetSignal(&RSSI);
        FREQ2_4G_GetlossPack(&lossPack);

        /* 调试时可打开以下状态显示。 */
        // OLED_ShowHexNum(2, 8, RSSI, 3);
        // OLED_ShowHexNum(1, 1, lossPack, 3);
    }
}

/**
 * @brief 对摇杆 ADC 数据采样并转换为百分比。
 */
/* 摇杆采样保留原十次平均，再发布带样本tick/序号的最新快照。 */
void JSTask(void *argument)
{
    uint16_t JS_DATA[2];
    TickType_t lastSampleWakeTime;
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)JS_DATA, 2);
    lastSampleWakeTime = xTaskGetTickCount();

    uint16_t VAL_X[10], VAL_Y[10];
    float JS_Val[2];
    float sumx = 0, sumy = 0;
    float X_Offset = 1, Y_Offset = 1;
    uint16_t X_LostPower_Offset = 30, Y_LostPower_Offset = 30;

    while (1)
    {
        /* 累计十次采样，保持原有采样间隔和换算方式。 */
        for (int i = 0; i < 10; i++)
        {
            VAL_X[i] = JS_DATA[0];
            VAL_Y[i] = JS_DATA[1];

            sumx += (200.0 / (4095.0 - X_LostPower_Offset)) * (VAL_X[i]) + (X_Offset);
            sumy += (200.0 / (4095.0 - Y_LostPower_Offset)) * (VAL_Y[i]) + (Y_Offset);
            vTaskDelayUntil(&lastSampleWakeTime, JOYSTICK_SAMPLE_PERIOD_TICKS);
        };

        JS_Val[0] = (float)((sumx / 10.0) - 100.0);
        JS_Val[1] = (float)((sumy / 10.0) - 100.0);

        sumx = 0;
        sumy = 0;

        TASK_PublishJoystickSnapshot(
            JS_Val[0],
            JS_Val[1],
            (uint32_t)xTaskGetTickCount());
        /* RIGHT OLED显示此X轴按MW规则换算后的Z角速度指令。 */
        MW_UpdateAngularJoystickDisplay(JS_Val[0]);
    }
}

/**
 * @brief 新完整帧到达后，沿用原五次累计平均计算并发布姿态快照。
 */
/* 只在新完整帧到达后发布姿态；重复读取缓存不能刷新100ms新鲜度。 */
void GYROTask(void *argument)
{
    TickType_t lastSampleWakeTime;
    uint32_t receivedFrameTick;
    float Euler_P_BUFF = 0;
    float Euler_R_BUFF = 0;
    float Euler_Y_BUFF = 0;
    volatile float SUM_R = 0, SUM_P = 0, SUM_Y = 0;

    HLKAs201_init();
    lastSampleWakeTime = xTaskGetTickCount();
    while (1)
    {
        if (HLKAs201_getData(&receivedFrameTick) == 0U)
        {
            /* 无新帧/半帧/坏帧时仅让出CPU，保留上次样本时间供控制任务判断超时。 */
            vTaskDelayUntil(&lastSampleWakeTime, GYRO_SAMPLE_PERIOD_TICKS);
            continue;
        }
        HLKAs201_getData_A();
        HLKAs201_getData_Euler();
        HLKAs201_getData_H();

        for (int i = 0; i < 5; i++)
        {
            SUM_P += atan2f(HLKAs201_Data_A[0], HLKAs201_Data_A[2]) * 180.0 / MyPi;
            SUM_R += HLKAs201_Data_Euler[0];
            SUM_Y += HLKAs201_Data_Euler[2];
            vTaskDelayUntil(&lastSampleWakeTime, GYRO_SAMPLE_PERIOD_TICKS);
        };

        Euler_P_BUFF = SUM_P / 5.0;
        Euler_R_BUFF = SUM_R / 5.0;
        Euler_Y_BUFF = SUM_Y / 5.0;

        TASK_PublishGyroSnapshot(
            Euler_R_BUFF,
            -Euler_P_BUFF,
            Euler_Y_BUFF,
            HLKAs201_Data_H[0],
            /* 时间记录在取得帧时，后面的五次平均等待不能延长其有效期。 */
            (uint32_t)receivedFrameTick);

        SUM_P = 0;
        SUM_R = 0;
        SUM_Y = 0;
    }
}
