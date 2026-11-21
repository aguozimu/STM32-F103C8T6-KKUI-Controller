#include "Mydef.h"
#include "controller_config.h"
#include "handset_link.h"
#include "controller_mode.h"

#if (CONTROLLER_LINK_CRC16_REFLECT_INPUT != 0U) || (CONTROLLER_LINK_CRC16_REFLECT_OUTPUT != 0U)
#error "This implementation uses CRC-16/CCITT-FALSE (non-reflected)"
#endif
#if CONTROLLER_LINK_CRC16_POLYNOMIAL != 0x1021U
#error "CRC lookup table only matches polynomial 0x1021"
#endif

typedef struct
{
    uint8_t message_type;
    uint8_t payload_length;
    uint32_t connection_id;
    uint8_t payload[CONTROLLER_LINK_MAX_PAYLOAD_SIZE];
} handset_link_tx_request_t;

static const uint16_t gCrc16Table[256] =
{
    0x0000U, 0x1021U, 0x2042U, 0x3063U, 0x4084U, 0x50A5U, 0x60C6U, 0x70E7U,
    0x8108U, 0x9129U, 0xA14AU, 0xB16BU, 0xC18CU, 0xD1ADU, 0xE1CEU, 0xF1EFU,
    0x1231U, 0x0210U, 0x3273U, 0x2252U, 0x52B5U, 0x4294U, 0x72F7U, 0x62D6U,
    0x9339U, 0x8318U, 0xB37BU, 0xA35AU, 0xD3BDU, 0xC39CU, 0xF3FFU, 0xE3DEU,
    0x2462U, 0x3443U, 0x0420U, 0x1401U, 0x64E6U, 0x74C7U, 0x44A4U, 0x5485U,
    0xA56AU, 0xB54BU, 0x8528U, 0x9509U, 0xE5EEU, 0xF5CFU, 0xC5ACU, 0xD58DU,
    0x3653U, 0x2672U, 0x1611U, 0x0630U, 0x76D7U, 0x66F6U, 0x5695U, 0x46B4U,
    0xB75BU, 0xA77AU, 0x9719U, 0x8738U, 0xF7DFU, 0xE7FEU, 0xD79DU, 0xC7BCU,
    0x48C4U, 0x58E5U, 0x6886U, 0x78A7U, 0x0840U, 0x1861U, 0x2802U, 0x3823U,
    0xC9CCU, 0xD9EDU, 0xE98EU, 0xF9AFU, 0x8948U, 0x9969U, 0xA90AU, 0xB92BU,
    0x5AF5U, 0x4AD4U, 0x7AB7U, 0x6A96U, 0x1A71U, 0x0A50U, 0x3A33U, 0x2A12U,
    0xDBFDU, 0xCBDCU, 0xFBBFU, 0xEB9EU, 0x9B79U, 0x8B58U, 0xBB3BU, 0xAB1AU,
    0x6CA6U, 0x7C87U, 0x4CE4U, 0x5CC5U, 0x2C22U, 0x3C03U, 0x0C60U, 0x1C41U,
    0xEDAEU, 0xFD8FU, 0xCDECU, 0xDDCDU, 0xAD2AU, 0xBD0BU, 0x8D68U, 0x9D49U,
    0x7E97U, 0x6EB6U, 0x5ED5U, 0x4EF4U, 0x3E13U, 0x2E32U, 0x1E51U, 0x0E70U,
    0xFF9FU, 0xEFBEU, 0xDFDDU, 0xCFFCU, 0xBF1BU, 0xAF3AU, 0x9F59U, 0x8F78U,
    0x9188U, 0x81A9U, 0xB1CAU, 0xA1EBU, 0xD10CU, 0xC12DU, 0xF14EU, 0xE16FU,
    0x1080U, 0x00A1U, 0x30C2U, 0x20E3U, 0x5004U, 0x4025U, 0x7046U, 0x6067U,
    0x83B9U, 0x9398U, 0xA3FBU, 0xB3DAU, 0xC33DU, 0xD31CU, 0xE37FU, 0xF35EU,
    0x02B1U, 0x1290U, 0x22F3U, 0x32D2U, 0x4235U, 0x5214U, 0x6277U, 0x7256U,
    0xB5EAU, 0xA5CBU, 0x95A8U, 0x8589U, 0xF56EU, 0xE54FU, 0xD52CU, 0xC50DU,
    0x34E2U, 0x24C3U, 0x14A0U, 0x0481U, 0x7466U, 0x6447U, 0x5424U, 0x4405U,
    0xA7DBU, 0xB7FAU, 0x8799U, 0x97B8U, 0xE75FU, 0xF77EU, 0xC71DU, 0xD73CU,
    0x26D3U, 0x36F2U, 0x0691U, 0x16B0U, 0x6657U, 0x7676U, 0x4615U, 0x5634U,
    0xD94CU, 0xC96DU, 0xF90EU, 0xE92FU, 0x99C8U, 0x89E9U, 0xB98AU, 0xA9ABU,
    0x5844U, 0x4865U, 0x7806U, 0x6827U, 0x18C0U, 0x08E1U, 0x3882U, 0x28A3U,
    0xCB7DU, 0xDB5CU, 0xEB3FU, 0xFB1EU, 0x8BF9U, 0x9BD8U, 0xABBBU, 0xBB9AU,
    0x4A75U, 0x5A54U, 0x6A37U, 0x7A16U, 0x0AF1U, 0x1AD0U, 0x2AB3U, 0x3A92U,
    0xFD2EU, 0xED0FU, 0xDD6CU, 0xCD4DU, 0xBDAAU, 0xAD8BU, 0x9DE8U, 0x8DC9U,
    0x7C26U, 0x6C07U, 0x5C64U, 0x4C45U, 0x3CA2U, 0x2C83U, 0x1CE0U, 0x0CC1U,
    0xEF1FU, 0xFF3EU, 0xCF5DU, 0xDF7CU, 0xAF9BU, 0xBFBAU, 0x8FD9U, 0x9FF8U,
    0x6E17U, 0x7E36U, 0x4E55U, 0x5E74U, 0x2E93U, 0x3EB2U, 0x0ED1U, 0x1EF0U
};

static volatile uint8_t gRxRing[CONTROLLER_LINK_RX_BUFFER_SIZE];
static volatile uint16_t gRxWriteIndex;
static volatile uint16_t gRxReadIndex;
static volatile uint16_t gRxResetGeneration;
static volatile uint8_t gRxByte;
static volatile uint8_t gRxArmed;
static volatile uint8_t gTxActive;
static TaskHandle_t volatile gLinkTaskHandle;
static uint8_t gTxFrame[CONTROLLER_LINK_MAX_FRAME_SIZE];
static handset_link_tx_request_t gTxMailbox;
static volatile uint8_t gTxMailboxPending;
static uint16_t gTxSequence;
static volatile handset_link_diagnostics_t gDiagnostics;
static controller_test_input_t gLastTestInput;
static volatile uint8_t gUiStatePending;
static volatile uint16_t gUiStateGeneration;

static uint8_t gParserFrame[CONTROLLER_LINK_MAX_FRAME_SIZE];
static uint8_t gParserLength;
static uint8_t gParserExpectedLength;
static TickType_t gParserLastByteTick;
static uint16_t gParserSeenResetGeneration;

/* USART3接收回调先把字节放入环形缓冲，再调用本函数唤醒任务；解析留在任务上下文。 */
static void NotifyLinkTaskFromISR(void)
{
    BaseType_t higherPriorityTaskWoken = pdFALSE;
    TaskHandle_t taskHandle = gLinkTaskHandle;
    if (taskHandle != NULL)
    {
        vTaskNotifyGiveFromISR(taskHandle, &higherPriorityTaskWoken);
    }
    portYIELD_FROM_ISR(higherPriorityTaskWoken);
}

uint16_t HandsetLink_Crc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = (uint16_t)CONTROLLER_LINK_CRC16_INITIAL;
    uint16_t index;
    if ((data == NULL) && (length != 0U))
    {
        return 0U;
    }
    for (index = 0U; index < length; index++)
    {
        uint8_t tableIndex = (uint8_t)((crc >> 8) ^ data[index]);
        crc = (uint16_t)((crc << 8) ^ gCrc16Table[tableIndex]);
    }
    return (uint16_t)(crc ^ (uint16_t)CONTROLLER_LINK_CRC16_XOR_OUT);
}

/* 逐字段组帧：同步字、版本、类型、长度、连接号、序号、负载，末尾附CRC16。 */
static uint8_t BuildFrame(uint8_t messageType, uint32_t connectionId, uint16_t sequence,
    const uint8_t *payload, uint8_t payloadLength, uint8_t *frame)
{
    uint16_t crc;
    if ((frame == NULL) || (payloadLength > CONTROLLER_LINK_MAX_PAYLOAD_SIZE) ||
        ((payloadLength != 0U) && (payload == NULL)))
    {
        return 0U;
    }
    frame[CONTROLLER_LINK_SYNC_OFFSET] = CONTROLLER_LINK_SYNC_BYTE_0;
    frame[CONTROLLER_LINK_SYNC_OFFSET + 1U] = CONTROLLER_LINK_SYNC_BYTE_1;
    frame[CONTROLLER_LINK_VERSION_OFFSET] = (uint8_t)CONTROLLER_PROTOCOL_VERSION;
    frame[CONTROLLER_LINK_MESSAGE_TYPE_OFFSET] = messageType;
    frame[CONTROLLER_LINK_PAYLOAD_LENGTH_OFFSET] = payloadLength;
    frame[CONTROLLER_LINK_CONNECTION_ID_OFFSET] = (uint8_t)(connectionId & 0xFFU);
    frame[CONTROLLER_LINK_CONNECTION_ID_OFFSET + 1U] = (uint8_t)((connectionId >> 8) & 0xFFU);
    frame[CONTROLLER_LINK_CONNECTION_ID_OFFSET + 2U] = (uint8_t)((connectionId >> 16) & 0xFFU);
    frame[CONTROLLER_LINK_CONNECTION_ID_OFFSET + 3U] = (uint8_t)((connectionId >> 24) & 0xFFU);
    frame[CONTROLLER_LINK_SEQUENCE_OFFSET] = (uint8_t)(sequence & 0xFFU);
    frame[CONTROLLER_LINK_SEQUENCE_OFFSET + 1U] = (uint8_t)(sequence >> 8);
    if (payloadLength != 0U)
    {
        memcpy(&frame[CONTROLLER_LINK_PAYLOAD_OFFSET], payload, payloadLength);
    }
    crc = HandsetLink_Crc16(&frame[CONTROLLER_LINK_SYNC_SIZE],
        (uint16_t)(CONTROLLER_LINK_HEADER_SIZE - CONTROLLER_LINK_SYNC_SIZE + payloadLength));
    frame[CONTROLLER_LINK_PAYLOAD_OFFSET + payloadLength] = (uint8_t)(crc & 0xFFU);
    frame[CONTROLLER_LINK_PAYLOAD_OFFSET + payloadLength + 1U] = (uint8_t)(crc >> 8);
    return (uint8_t)(CONTROLLER_LINK_HEADER_SIZE + payloadLength + CONTROLLER_LINK_CRC_SIZE);
}

uint8_t HandsetLink_ProtocolSelfTest(void)
{
    static const uint8_t checkText[] = "123456789";
    uint8_t frame[CONTROLLER_LINK_MAX_FRAME_SIZE];
    uint8_t frameLength;
    uint16_t storedCrc;
    if (HandsetLink_Crc16(checkText, 9U) != 0x29B1U)
    {
        return 0U;
    }
    frameLength = BuildFrame(HANDSET_LINK_MSG_PING, 0U, 0x1234U, NULL, 0U, frame);
    if (frameLength != (CONTROLLER_LINK_HEADER_SIZE + CONTROLLER_LINK_CRC_SIZE))
    {
        return 0U;
    }
    storedCrc = (uint16_t)frame[CONTROLLER_LINK_PAYLOAD_OFFSET] |
        ((uint16_t)frame[CONTROLLER_LINK_PAYLOAD_OFFSET + 1U] << 8);
    /* 固定帧向量：01 01 00 00 00 00 00 34 12 -> CRC 0x4FA0（线上低字节先发）。 */
    return (storedCrc == 0x4FA0U) ? 1U : 0U;
}

uint8_t HandsetLink_QueueMessage(uint8_t messageType, uint32_t connectionId,
    const uint8_t *payload, uint8_t payloadLength)
{
    TaskHandle_t taskHandle;
    if ((messageType == 0U) || (payloadLength > CONTROLLER_LINK_MAX_PAYLOAD_SIZE) ||
        ((payloadLength != 0U) && (payload == NULL)))
    {
        return 0U;
    }
    taskENTER_CRITICAL();
    taskHandle = gLinkTaskHandle;
    if ((taskHandle == NULL) || (gTxMailboxPending != 0U))
    {
        gDiagnostics.tx_mailbox_drop_count++;
        taskEXIT_CRITICAL();
        return 0U;
    }
    gTxMailbox.message_type = messageType;
    gTxMailbox.payload_length = payloadLength;
    gTxMailbox.connection_id = connectionId;
    if (payloadLength != 0U)
    {
        memcpy(gTxMailbox.payload, payload, payloadLength);
    }
    gTxMailboxPending = 1U;
    taskEXIT_CRITICAL();
    xTaskNotifyGive(taskHandle);
    return 1U;
}

uint8_t HandsetLink_GetDiagnostics(handset_link_diagnostics_t *diagnostics)
{
	controller_mode_snapshot_t modeSnapshot;
    if (diagnostics == NULL)
    {
        return 0U;
    }
    taskENTER_CRITICAL();
    diagnostics->rx_byte_count = gDiagnostics.rx_byte_count;
    diagnostics->valid_frame_count = gDiagnostics.valid_frame_count;
    diagnostics->crc_error_count = gDiagnostics.crc_error_count;
    diagnostics->length_error_count = gDiagnostics.length_error_count;
    diagnostics->version_error_count = gDiagnostics.version_error_count;
    diagnostics->message_error_count = gDiagnostics.message_error_count;
    diagnostics->unknown_message_count = gDiagnostics.unknown_message_count;
    diagnostics->rx_ring_overflow_count = gDiagnostics.rx_ring_overflow_count;
    diagnostics->uart_error_count = gDiagnostics.uart_error_count;
    diagnostics->rx_rearm_error_count = gDiagnostics.rx_rearm_error_count;
    diagnostics->rx_arm_retry_error_count = gDiagnostics.rx_arm_retry_error_count;
    diagnostics->tx_frame_count = gDiagnostics.tx_frame_count;
    diagnostics->tx_error_count = gDiagnostics.tx_error_count;
    diagnostics->tx_start_error_count = gDiagnostics.tx_start_error_count;
    diagnostics->tx_mailbox_drop_count = gDiagnostics.tx_mailbox_drop_count;
    diagnostics->ping_rx_count = gDiagnostics.ping_rx_count;
    diagnostics->pong_rx_count = gDiagnostics.pong_rx_count;
    diagnostics->test_input_rx_count = gDiagnostics.test_input_rx_count;
    diagnostics->input_rx_count = gDiagnostics.input_rx_count;
    diagnostics->ui_state_rx_count = gDiagnostics.ui_state_rx_count;
    diagnostics->last_rx_connection_id = gDiagnostics.last_rx_connection_id;
    diagnostics->rx_ring_high_water = gDiagnostics.rx_ring_high_water;
    diagnostics->last_rx_sequence = gDiagnostics.last_rx_sequence;
    diagnostics->last_pong_echo_sequence = gDiagnostics.last_pong_echo_sequence;
    diagnostics->protocol_self_test_passed = gDiagnostics.protocol_self_test_passed;
    diagnostics->uart_configuration_matches = gDiagnostics.uart_configuration_matches;
    diagnostics->rx_armed = gRxArmed;
    diagnostics->tx_active = gTxActive;
    taskEXIT_CRITICAL();
    ControllerMode_GetSnapshot(&modeSnapshot);
    diagnostics->local_role = modeSnapshot.local_role;
    diagnostics->peer_role = modeSnapshot.peer_role;
    diagnostics->connection_state = (uint8_t)modeSnapshot.state;
    return 1U;
}

uint8_t HandsetLink_GetLastTestInput(controller_test_input_t *input)
{
    if (input == NULL)
    {
        return 0U;
    }
    taskENTER_CRITICAL();
    *input = gLastTestInput;
    taskEXIT_CRITICAL();
    return 1U;
}

static uint32_t ReadLe32(const uint8_t *data)
{
    return ((uint32_t)data[0]) | ((uint32_t)data[1] << 8) |
        ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

static uint16_t ReadLe16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static void WriteLe16(uint8_t *data, uint16_t value)
{
	data[0] = (uint8_t)value;
	data[1] = (uint8_t)(value >> 8);
}

static int16_t ReadLeI16(const uint8_t *data)
{
    return (int16_t)ReadLe16(data);
}

static void WriteLeI16(uint8_t *data, int16_t value)
{
    uint16_t raw = (uint16_t)value;
    data[0] = (uint8_t)(raw & 0xFFU);
    data[1] = (uint8_t)(raw >> 8);
}

static int16_t FloatToCenti(float value)
{
    if (value != value)
    {
        return 0;
    }
    if (value > 327.67f)
    {
        return 32767;
    }
    if (value < -327.68f)
    {
        return (int16_t)-32768;
    }
    return (int16_t)(value * 100.0f);
}

static uint8_t LinkElapsed(TickType_t now, TickType_t since, uint32_t period_ms)
{
	return ((TickType_t)(now - since) >= pdMS_TO_TICKS(period_ms)) ? 1U : 0U;
}

void HandsetLink_NotifyUIChanged(void)
{
#if CONTROLLER_LOCAL_ROLE == CONTROLLER_ROLE_LEFT
	TaskHandle_t taskHandle;
	taskENTER_CRITICAL();
	gUiStatePending = 1U;
	gUiStateGeneration++;
	taskHandle = gLinkTaskHandle;
	taskEXIT_CRITICAL();
	if (taskHandle != NULL)
	{
		xTaskNotifyGive(taskHandle);
	}
#endif
}

uint8_t HandsetLink_QueueKeyEventAck(uint16_t event_sequence, uint32_t connection_id)
{
#if CONTROLLER_LOCAL_ROLE == CONTROLLER_ROLE_LEFT
	controller_mode_snapshot_t modeSnapshot;
	uint8_t payload[2];
	ControllerMode_GetSnapshot(&modeSnapshot);
	if (modeSnapshot.state != CONTROLLER_LINK_STATE_DUAL ||
		modeSnapshot.connection_id != connection_id)
	{
		return 0U;
	}
	WriteLe16(payload, event_sequence);
	return HandsetLink_QueueMessage(HANDSET_LINK_MSG_EVENT_ACK, connection_id,
		payload, sizeof(payload));
#else
	(void)event_sequence;
	(void)connection_id;
	return 0U;
#endif
}

static uint8_t RxRingIsEmpty(void)
{
    return (gRxReadIndex == gRxWriteIndex) ? 1U : 0U;
}

static uint8_t RxRingPop(uint8_t *byte)
{
    uint16_t readIndex = gRxReadIndex;
    uint16_t writeIndex = gRxWriteIndex;
    if ((byte == NULL) || (readIndex == writeIndex))
    {
        return 0U;
    }
    *byte = gRxRing[readIndex & (CONTROLLER_LINK_RX_BUFFER_SIZE - 1U)];
    gRxReadIndex = (uint16_t)(readIndex + 1U);
    return 1U;
}

static void ArmReceiver(void)
{
    if (gRxArmed == 0U)
    {
        if (HAL_UART_Receive_IT(&huart3, (uint8_t *)&gRxByte, 1U) == HAL_OK)
        {
            gRxArmed = 1U;
        }
        else
        {
            gDiagnostics.rx_arm_retry_error_count++;
        }
    }
}

static void StartReceiver(void)
{
    gDiagnostics.uart_configuration_matches =
        (huart3.Init.BaudRate == CONTROLLER_UART_BAUDRATE &&
         huart3.Init.WordLength == UART_WORDLENGTH_8B &&
         huart3.Init.StopBits == UART_STOPBITS_1 &&
         huart3.Init.Parity == UART_PARITY_NONE &&
         huart3.Init.Mode == UART_MODE_TX_RX) ? 1U : 0U;
    HAL_NVIC_SetPriority(USART3_IRQn, CONTROLLER_USART3_IRQ_PRIORITY, 0U);
    HAL_NVIC_ClearPendingIRQ(USART3_IRQn);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
    ArmReceiver();
}

void USART3_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart3);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint16_t writeIndex;
    uint16_t readIndex;
    uint16_t used;
    if ((huart == NULL) || (huart->Instance != USART3))
    {
        return;
    }
    gRxArmed = 0U;
    writeIndex = gRxWriteIndex;
    readIndex = gRxReadIndex;
    used = (uint16_t)(writeIndex - readIndex);
    if (used < CONTROLLER_LINK_RX_BUFFER_SIZE)
    {
        gRxRing[writeIndex & (CONTROLLER_LINK_RX_BUFFER_SIZE - 1U)] = gRxByte;
        gRxWriteIndex = (uint16_t)(writeIndex + 1U);
        gDiagnostics.rx_byte_count++;
        used++;
        if (used > gDiagnostics.rx_ring_high_water)
        {
            gDiagnostics.rx_ring_high_water = used;
        }
    }
    else
    {
        gDiagnostics.rx_ring_overflow_count++;
        gRxResetGeneration++;
    }
    if (HAL_UART_Receive_IT(&huart3, (uint8_t *)&gRxByte, 1U) == HAL_OK)
    {
        gRxArmed = 1U;
    }
    else
    {
        gDiagnostics.rx_rearm_error_count++;
    }
    NotifyLinkTaskFromISR();
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if ((huart == NULL) || (huart->Instance != USART3))
    {
        return;
    }
    gTxActive = 0U;
    gDiagnostics.tx_frame_count++;
    NotifyLinkTaskFromISR();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if ((huart == NULL) || (huart->Instance != USART3))
    {
        return;
    }
    gDiagnostics.uart_error_count++;
    /* 非阻塞FE/NE仍保持HAL接收；ORE等阻塞错误结束接收后由任务重新挂接。 */
    gRxArmed = (huart->RxState == HAL_UART_STATE_BUSY_RX) ? 1U : 0U;
    gRxResetGeneration++;
    if ((gTxActive != 0U) && (huart->gState == HAL_UART_STATE_READY))
    {
        gTxActive = 0U;
        gDiagnostics.tx_error_count++;
    }
    NotifyLinkTaskFromISR();
}

static uint8_t BuildMailboxFrame(uint8_t *frame)
{
    handset_link_tx_request_t request;
    uint8_t hasRequest = 0U;
    taskENTER_CRITICAL();
    if (gTxMailboxPending != 0U)
    {
        request = gTxMailbox;
        gTxMailboxPending = 0U;
        hasRequest = 1U;
    }
    taskEXIT_CRITICAL();
    if (hasRequest == 0U)
    {
        return 0U;
    }
    gTxSequence++;
    return BuildFrame(request.message_type, request.connection_id, gTxSequence,
        request.payload, request.payload_length, frame);
}

/* TX邮箱只缓存一帧；正在中断发送的帧不会被新请求覆盖。 */
static void ProcessPendingTransmit(void)
{
    uint8_t frameLength;
    if (gTxActive != 0U)
    {
        return;
    }
    frameLength = BuildMailboxFrame(gTxFrame);
    if (frameLength == 0U)
    {
        return;
    }
    gTxActive = 1U;
    if (HAL_UART_Transmit_IT(&huart3, gTxFrame, frameLength) != HAL_OK)
    {
        gTxActive = 0U;
        gDiagnostics.tx_start_error_count++;
    }
}

static void ParserReset(void)
{
    gParserLength = 0U;
    gParserExpectedLength = 0U;
}

static void ParserKeepPossibleSyncSuffix(void)
{
    uint8_t offset;
    for (offset = 1U; (uint8_t)(offset + 1U) < gParserLength; offset++)
    {
        if ((gParserFrame[offset] == CONTROLLER_LINK_SYNC_BYTE_0) &&
            (gParserFrame[offset + 1U] == CONTROLLER_LINK_SYNC_BYTE_1))
        {
            uint8_t remain = (uint8_t)(gParserLength - offset);
            memmove(gParserFrame, &gParserFrame[offset], remain);
            gParserLength = remain;
            gParserExpectedLength = (remain > CONTROLLER_LINK_PAYLOAD_LENGTH_OFFSET &&
                gParserFrame[CONTROLLER_LINK_PAYLOAD_LENGTH_OFFSET] <= CONTROLLER_LINK_MAX_PAYLOAD_SIZE) ?
                (uint8_t)(CONTROLLER_LINK_HEADER_SIZE +
                    gParserFrame[CONTROLLER_LINK_PAYLOAD_LENGTH_OFFSET] +
                    CONTROLLER_LINK_CRC_SIZE) : 0U;
            return;
        }
    }
    if ((gParserLength != 0U) &&
        (gParserFrame[gParserLength - 1U] == CONTROLLER_LINK_SYNC_BYTE_0))
    {
        gParserFrame[0] = CONTROLLER_LINK_SYNC_BYTE_0;
        gParserLength = 1U;
        gParserExpectedLength = 0U;
    }
    else
    {
        ParserReset();
    }
}

/* 只由LEFT接收RIGHT的输入载荷；校验状态/连接号后写入远端输入快照。 */
static uint8_t ProcessInputPayload(uint32_t connectionId, const uint8_t *payload,
	uint8_t payloadLength)
{
	controller_mode_snapshot_t modeSnapshot;
	controller_input_snapshot_t input;
	uint8_t validFlags;
	if (payloadLength != CONTROLLER_LINK_INPUT_PAYLOAD_SIZE ||
		CONTROLLER_LOCAL_ROLE != CONTROLLER_ROLE_LEFT)
	{
		return 0U;
	}
	ControllerMode_GetSnapshot(&modeSnapshot);
	if (modeSnapshot.state != CONTROLLER_LINK_STATE_DUAL ||
		modeSnapshot.connection_id != connectionId)
	{
		return 0U;
	}
	validFlags = payload[0];
	if ((validFlags & (uint8_t)~(CONTROLLER_TEST_INPUT_JOYSTICK_VALID |
		CONTROLLER_TEST_INPUT_GYRO_VALID)) != 0U ||
		(payload[1] & (uint8_t)~(CONTROLLER_BUTTON_U | CONTROLLER_BUTTON_D |
		CONTROLLER_BUTTON_L | CONTROLLER_BUTTON_R | CONTROLLER_BUTTON_PA8)) != 0U ||
		payload[2] > CONTROLLER_KEY_MAX)
	{
		return 0U;
	}
	memset(&input, 0, sizeof(input));
	input.joystick_valid = (validFlags & CONTROLLER_TEST_INPUT_JOYSTICK_VALID) != 0U ? 1U : 0U;
	input.gyro_valid = (validFlags & CONTROLLER_TEST_INPUT_GYRO_VALID) != 0U ? 1U : 0U;
	input.button_bitmap = payload[1];
	input.button_event_key = payload[2];
	input.button_event_sequence = ReadLe16(&payload[3]);
	input.joystick_sample_sequence = ReadLe16(&payload[5]);
	input.joystick_x_percent = (float)ReadLeI16(&payload[7]) / 100.0f;
	input.joystick_y_percent = (float)ReadLeI16(&payload[9]) / 100.0f;
	input.gyro_sample_sequence = ReadLe16(&payload[11]);
	input.gyro_roll_deg = (float)ReadLeI16(&payload[13]) / 100.0f;
	input.gyro_pitch_deg = (float)ReadLeI16(&payload[15]) / 100.0f;
	input.gyro_yaw_deg = (float)ReadLeI16(&payload[17]) / 100.0f;
	input.gyro_height_m = (float)ReadLeI16(&payload[19]) / 100.0f;
	TASK_PublishPeerInput(&input);
	return 1U;
}

/* 只由RIGHT接收LEFT的菜单与显示快照，用于镜像界面，不拥有菜单控制权。 */
static uint8_t ProcessUiStatePayload(uint32_t connectionId, const uint8_t *payload,
	uint8_t payloadLength)
{
	controller_mode_snapshot_t modeSnapshot;
	task_menu_snapshot_t snapshot;
	uint8_t index;
	if (payloadLength != CONTROLLER_LINK_UI_STATE_PAYLOAD_SIZE ||
		CONTROLLER_LOCAL_ROLE != CONTROLLER_ROLE_RIGHT)
	{
		return 0U;
	}
	ControllerMode_GetSnapshot(&modeSnapshot);
	if (modeSnapshot.state != CONTROLLER_LINK_STATE_DUAL ||
		modeSnapshot.connection_id != connectionId || payload[0] > 2U ||
		payload[1] > 3U || payload[2] > 3U || payload[3] > 3U ||
		payload[4] > 3U || payload[5] > 1U || payload[6] > 1U ||
		payload[7] != CONTROLLER_LINK_STATE_DUAL || (payload[8] & 0xF0U) != 0U)
	{
		return 0U;
	}
	memset(&snapshot, 0, sizeof(snapshot));
	snapshot.Floor1 = (int8_t)payload[0];
	snapshot.Floor2 = (int8_t)payload[1];
	snapshot.FloorSwitch = (int8_t)payload[2];
	snapshot.Floor3 = (int8_t)payload[3];
	snapshot.Floor4 = (int8_t)payload[4];
	snapshot.DoubleModle = payload[5];
	snapshot.ControlEnabled = payload[6];
	snapshot.ConnectionState = payload[7];
	snapshot.DisplayValueValidMask = payload[8];
	for (index = 0U; index < 4U; index++)
	{
		snapshot.DisplayValue[index] = ReadLeI16(&payload[9U + index * 2U]);
	}
	TASK_SetRemoteMenuSnapshot(&snapshot);
	return 1U;
}

/* RIGHT收到匹配序号的ACK后清除待重发按键，避免同一菜单动作重复执行。 */
static uint8_t ProcessKeyEventAck(uint32_t connectionId, const uint8_t *payload,
	uint8_t payloadLength)
{
	controller_mode_snapshot_t modeSnapshot;
	uint16_t eventSequence;
	if (payloadLength != 2U || CONTROLLER_LOCAL_ROLE != CONTROLLER_ROLE_RIGHT)
	{
		return 0U;
	}
	ControllerMode_GetSnapshot(&modeSnapshot);
	if (modeSnapshot.state != CONTROLLER_LINK_STATE_DUAL ||
		modeSnapshot.connection_id != connectionId)
	{
		return 0U;
	}
	eventSequence = ReadLe16(payload);
	if (TASK_AcknowledgeButtonEvent(eventSequence) == 0U)
	{
		return 0U;
	}
	gDiagnostics.last_key_event_ack_sequence = eventSequence;
	return 1U;
}

/* 外层帧已通过长度、版本和CRC校验；这里再按消息类型做业务校验和分发。 */
static void ProcessValidMessage(uint8_t messageType, uint32_t connectionId, uint16_t sequence,
    const uint8_t *payload, uint8_t payloadLength)
{
    uint8_t replyPayload[2];
    switch (messageType)
    {
        case HANDSET_LINK_MSG_PING:
            if (payloadLength != 0U)
            {
                gDiagnostics.message_error_count++;
                break;
            }
            gDiagnostics.ping_rx_count++;
            replyPayload[0] = (uint8_t)(sequence & 0xFFU);
            replyPayload[1] = (uint8_t)(sequence >> 8);
            (void)HandsetLink_QueueMessage(HANDSET_LINK_MSG_PONG, connectionId, replyPayload, 2U);
            break;
        case HANDSET_LINK_MSG_PONG:
            if (payloadLength != 2U)
            {
                gDiagnostics.message_error_count++;
                break;
            }
            gDiagnostics.pong_rx_count++;
            gDiagnostics.last_pong_echo_sequence = ReadLe16(payload);
            break;
        case HANDSET_LINK_MSG_TEST_INPUT:
            if (payloadLength != CONTROLLER_TEST_INPUT_PAYLOAD_SIZE)
            {
                gDiagnostics.message_error_count++;
                break;
            }
            taskENTER_CRITICAL();
            gLastTestInput.sequence = sequence;
            gLastTestInput.valid_flags = payload[CONTROLLER_TEST_INPUT_VALID_FLAGS_OFFSET];
            gLastTestInput.joystick_x_centi_percent =
                ReadLeI16(&payload[CONTROLLER_TEST_INPUT_JOYSTICK_X_OFFSET]);
            gLastTestInput.joystick_y_centi_percent =
                ReadLeI16(&payload[CONTROLLER_TEST_INPUT_JOYSTICK_Y_OFFSET]);
            gLastTestInput.gyro_roll_centi_deg =
                ReadLeI16(&payload[CONTROLLER_TEST_INPUT_GYRO_ROLL_OFFSET]);
            gLastTestInput.gyro_pitch_centi_deg =
                ReadLeI16(&payload[CONTROLLER_TEST_INPUT_GYRO_PITCH_OFFSET]);
            gLastTestInput.gyro_yaw_centi_deg =
                ReadLeI16(&payload[CONTROLLER_TEST_INPUT_GYRO_YAW_OFFSET]);
            taskEXIT_CRITICAL();
            gDiagnostics.test_input_rx_count++;
            break;
        case HANDSET_LINK_MSG_HELLO:
        case HANDSET_LINK_MSG_HELLO_ACK:
        case HANDSET_LINK_MSG_HEARTBEAT:
            if (ControllerMode_OnLinkMessage(messageType, connectionId, sequence,
                payload, payloadLength, xTaskGetTickCount()) == 0U)
            {
                gDiagnostics.message_error_count++;
            }
            break;
        case HANDSET_LINK_MSG_INPUT:
            if (ProcessInputPayload(connectionId, payload, payloadLength) == 0U)
            {
                gDiagnostics.message_error_count++;
            }
            else
            {
                gDiagnostics.input_rx_count++;
            }
            break;
        case HANDSET_LINK_MSG_UI_STATE:
            if (ProcessUiStatePayload(connectionId, payload, payloadLength) == 0U)
            {
                gDiagnostics.message_error_count++;
            }
            else
            {
                gDiagnostics.ui_state_rx_count++;
            }
            break;
        case HANDSET_LINK_MSG_EVENT_ACK:
            if (ProcessKeyEventAck(connectionId, payload, payloadLength) == 0U)
            {
                gDiagnostics.message_error_count++;
            }
            break;
        default:
            gDiagnostics.unknown_message_count++;
            break;
    }
}

static void ParserCompleteFrame(void)
{
    uint8_t payloadLength = gParserFrame[CONTROLLER_LINK_PAYLOAD_LENGTH_OFFSET];
    uint16_t receivedCrc = (uint16_t)gParserFrame[CONTROLLER_LINK_PAYLOAD_OFFSET + payloadLength] |
        ((uint16_t)gParserFrame[CONTROLLER_LINK_PAYLOAD_OFFSET + payloadLength + 1U] << 8);
    uint16_t calculatedCrc = HandsetLink_Crc16(&gParserFrame[CONTROLLER_LINK_VERSION_OFFSET],
        (uint16_t)(CONTROLLER_LINK_HEADER_SIZE - CONTROLLER_LINK_SYNC_SIZE + payloadLength));
    uint32_t connectionId;
    uint16_t sequence;
    if (receivedCrc != calculatedCrc)
    {
        gDiagnostics.crc_error_count++;
        ParserKeepPossibleSyncSuffix();
        return;
    }
    if (gParserFrame[CONTROLLER_LINK_VERSION_OFFSET] != CONTROLLER_PROTOCOL_VERSION)
    {
        gDiagnostics.version_error_count++;
        ParserReset();
        return;
    }
    connectionId = ReadLe32(&gParserFrame[CONTROLLER_LINK_CONNECTION_ID_OFFSET]);
    sequence = ReadLe16(&gParserFrame[CONTROLLER_LINK_SEQUENCE_OFFSET]);
    gDiagnostics.valid_frame_count++;
    gDiagnostics.last_rx_connection_id = connectionId;
    gDiagnostics.last_rx_sequence = sequence;
    ProcessValidMessage(gParserFrame[CONTROLLER_LINK_MESSAGE_TYPE_OFFSET], connectionId, sequence,
        &gParserFrame[CONTROLLER_LINK_PAYLOAD_OFFSET], payloadLength);
    ParserReset();
}

/* 串口可能一次只来半帧或连来多帧；逐字节解析并在坏帧后寻找下一个同步字。 */
static void ParserConsumeByte(uint8_t byte)
{
    if (gParserLength == 0U)
    {
        if (byte == CONTROLLER_LINK_SYNC_BYTE_0)
        {
            gParserFrame[0] = byte;
            gParserLength = 1U;
            gParserLastByteTick = xTaskGetTickCount();
        }
        return;
    }
    if (gParserLength == CONTROLLER_LINK_SYNC_SIZE - 1U)
    {
        if (byte == CONTROLLER_LINK_SYNC_BYTE_1)
        {
            gParserFrame[1] = byte;
            gParserLength = 2U;
        }
        else if (byte == CONTROLLER_LINK_SYNC_BYTE_0)
        {
            gParserFrame[0] = byte;
        }
        else
        {
            ParserReset();
        }
        gParserLastByteTick = xTaskGetTickCount();
        return;
    }
    if (gParserLength >= CONTROLLER_LINK_MAX_FRAME_SIZE)
    {
        gDiagnostics.length_error_count++;
        ParserKeepPossibleSyncSuffix();
        return;
    }
    gParserFrame[gParserLength++] = byte;
    gParserLastByteTick = xTaskGetTickCount();
    if (gParserLength == CONTROLLER_LINK_PAYLOAD_LENGTH_OFFSET + 1U)
    {
        if (gParserFrame[CONTROLLER_LINK_PAYLOAD_LENGTH_OFFSET] > CONTROLLER_LINK_MAX_PAYLOAD_SIZE)
        {
            gDiagnostics.length_error_count++;
            ParserKeepPossibleSyncSuffix();
            return;
        }
        gParserExpectedLength = (uint8_t)(CONTROLLER_LINK_HEADER_SIZE +
            gParserFrame[CONTROLLER_LINK_PAYLOAD_LENGTH_OFFSET] +
            CONTROLLER_LINK_CRC_SIZE);
    }
    if ((gParserExpectedLength != 0U) && (gParserLength == gParserExpectedLength))
    {
        ParserCompleteFrame();
    }
}

/* RIGHT把摇杆、姿态、按住位图和待确认按键逐字段转成定长负载。 */
static uint8_t BuildInputPayload(uint8_t *payload)
{
	controller_input_snapshot_t input;
	if (payload == NULL || CONTROLLER_LOCAL_ROLE != CONTROLLER_ROLE_RIGHT ||
		ControllerMode_GetState() != CONTROLLER_LINK_STATE_DUAL ||
		TASK_GetInputSnapshot(&input) == 0U)
	{
		return 0U;
	}
	memset(payload, 0, CONTROLLER_LINK_INPUT_PAYLOAD_SIZE);
	if (input.joystick_valid != 0U)
	{
		payload[0] |= CONTROLLER_TEST_INPUT_JOYSTICK_VALID;
	}
	if (input.gyro_valid != 0U)
	{
		payload[0] |= CONTROLLER_TEST_INPUT_GYRO_VALID;
	}
	payload[1] = input.button_bitmap;
	payload[2] = input.button_event_key;
	WriteLe16(&payload[3], input.button_event_sequence);
	WriteLe16(&payload[5], input.joystick_sample_sequence);
	WriteLeI16(&payload[7], FloatToCenti(input.joystick_x_percent));
	WriteLeI16(&payload[9], FloatToCenti(input.joystick_y_percent));
	WriteLe16(&payload[11], input.gyro_sample_sequence);
	WriteLeI16(&payload[13], FloatToCenti(input.gyro_roll_deg));
	WriteLeI16(&payload[15], FloatToCenti(input.gyro_pitch_deg));
	WriteLeI16(&payload[17], FloatToCenti(input.gyro_yaw_deg));
	WriteLeI16(&payload[19], FloatToCenti(input.gyro_height_m));
	return CONTROLLER_LINK_INPUT_PAYLOAD_SIZE;
}

/* LEFT把菜单和4个显示值逐字段编码；显示值以有符号整数传输。 */
static uint8_t BuildUiStatePayload(uint8_t *payload)
{
	task_menu_snapshot_t state;
	uint8_t index;
	if (payload == NULL || CONTROLLER_LOCAL_ROLE != CONTROLLER_ROLE_LEFT ||
		ControllerMode_GetState() != CONTROLLER_LINK_STATE_DUAL)
	{
		return 0U;
	}
	TASK_GetMenuSnapshot(&state);
	payload[0] = (uint8_t)state.Floor1;
	payload[1] = (uint8_t)state.Floor2;
	payload[2] = (uint8_t)state.FloorSwitch;
	payload[3] = (uint8_t)state.Floor3;
	payload[4] = (uint8_t)state.Floor4;
	payload[5] = state.DoubleModle;
	payload[6] = state.ControlEnabled;
	payload[7] = state.ConnectionState;
	payload[8] = state.DisplayValueValidMask;
	for (index = 0U; index < 4U; index++)
	{
		WriteLeI16(&payload[9U + index * 2U], state.DisplayValue[index]);
	}
	return CONTROLLER_LINK_UI_STATE_PAYLOAD_SIZE;
}

/* 优先处理握手/心跳；DUAL时RIGHT定期上传输入，LEFT按变化或周期同步UI。 */
static void PollApplicationMessages(TickType_t now, TickType_t *lastInputTick,
	TickType_t *lastUiTick)
{
	controller_mode_snapshot_t modeSnapshot;
	uint8_t payload[CONTROLLER_LINK_MAX_PAYLOAD_SIZE];
	uint8_t payloadLength;
	uint8_t uiPending;
	uint16_t uiGeneration;
	taskENTER_CRITICAL();
	if (gTxMailboxPending != 0U)
	{
		taskEXIT_CRITICAL();
		return;
	}
	taskEXIT_CRITICAL();
	if (ControllerMode_PollLink(now) != 0U)
	{
		return;
	}
	ControllerMode_GetSnapshot(&modeSnapshot);
	if (modeSnapshot.state != CONTROLLER_LINK_STATE_DUAL)
	{
		return;
	}
	if (CONTROLLER_LOCAL_ROLE == CONTROLLER_ROLE_RIGHT &&
		LinkElapsed(now, *lastInputTick, CONTROLLER_LINK_INPUT_PERIOD_MS) != 0U)
	{
		payloadLength = BuildInputPayload(payload);
		if (payloadLength != 0U && HandsetLink_QueueMessage(HANDSET_LINK_MSG_INPUT,
			modeSnapshot.connection_id, payload, payloadLength) != 0U)
		{
			*lastInputTick = now;
			return;
		}
	}
	if (CONTROLLER_LOCAL_ROLE == CONTROLLER_ROLE_LEFT)
	{
		taskENTER_CRITICAL();
		uiPending = gUiStatePending;
		uiGeneration = gUiStateGeneration;
		taskEXIT_CRITICAL();
		if (uiPending != 0U || LinkElapsed(now, *lastUiTick, CONTROLLER_LINK_UI_PERIOD_MS) != 0U)
		{
			payloadLength = BuildUiStatePayload(payload);
			if (payloadLength != 0U && HandsetLink_QueueMessage(HANDSET_LINK_MSG_UI_STATE,
				modeSnapshot.connection_id, payload, payloadLength) != 0U)
			{
				taskENTER_CRITICAL();
				if (uiGeneration == gUiStateGeneration)
				{
					gUiStatePending = 0U;
				}
				taskEXIT_CRITICAL();
				*lastUiTick = now;
			}
		}
	}
}

#if CONTROLLER_LINK_TEST_MODE
static void StoreTestInput(void)
{
    controller_input_snapshot_t snapshot;
    uint8_t payload[CONTROLLER_TEST_INPUT_PAYLOAD_SIZE] = {0U};
    if (TASK_GetInputSnapshot(&snapshot) == 0U)
    {
        return;
    }
    if (snapshot.joystick_valid != 0U)
    {
        payload[CONTROLLER_TEST_INPUT_VALID_FLAGS_OFFSET] |= CONTROLLER_TEST_INPUT_JOYSTICK_VALID;
        WriteLeI16(&payload[CONTROLLER_TEST_INPUT_JOYSTICK_X_OFFSET], FloatToCenti(snapshot.joystick_x_percent));
        WriteLeI16(&payload[CONTROLLER_TEST_INPUT_JOYSTICK_Y_OFFSET], FloatToCenti(snapshot.joystick_y_percent));
    }
    if (snapshot.gyro_valid != 0U)
    {
        payload[CONTROLLER_TEST_INPUT_VALID_FLAGS_OFFSET] |= CONTROLLER_TEST_INPUT_GYRO_VALID;
        WriteLeI16(&payload[CONTROLLER_TEST_INPUT_GYRO_ROLL_OFFSET], FloatToCenti(snapshot.gyro_roll_deg));
        WriteLeI16(&payload[CONTROLLER_TEST_INPUT_GYRO_PITCH_OFFSET], FloatToCenti(snapshot.gyro_pitch_deg));
        WriteLeI16(&payload[CONTROLLER_TEST_INPUT_GYRO_YAW_OFFSET], FloatToCenti(snapshot.gyro_yaw_deg));
    }
    (void)HandsetLink_QueueMessage(HANDSET_LINK_MSG_TEST_INPUT, 0U, payload,
        CONTROLLER_TEST_INPUT_PAYLOAD_SIZE);
}
#endif

/* 任务启动先跑协议自检并挂接接收；之后按批次解析，避免长时间占住CPU。 */
void HandsetLink_Task(void *argument)
{
    TickType_t lastPingTick;
    TickType_t lastTestInputTick;
    TickType_t lastInputTxTick;
    TickType_t lastUiTxTick;
    (void)argument;
    gLinkTaskHandle = xTaskGetCurrentTaskHandle();
    gDiagnostics.local_role = CONTROLLER_LOCAL_ROLE;
    gDiagnostics.protocol_self_test_passed = HandsetLink_ProtocolSelfTest();
    if (gDiagnostics.protocol_self_test_passed == 0U)
    {
        for (;;)
        {
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        }
    }
    StartReceiver();
    lastPingTick = xTaskGetTickCount();
    lastTestInputTick = lastPingTick;
    lastInputTxTick = lastPingTick;
    lastUiTxTick = lastPingTick;
    for (;;)
    {
        uint8_t byte;
        uint8_t processedBytes = 0U;
        TickType_t now = xTaskGetTickCount();
        uint16_t resetGeneration = gRxResetGeneration;
        if (resetGeneration != gParserSeenResetGeneration)
        {
            ParserReset();
            gParserSeenResetGeneration = resetGeneration;
        }
        if ((gParserLength != 0U) &&
            (TickType_t)(xTaskGetTickCount() - gParserLastByteTick) >=
            pdMS_TO_TICKS(CONTROLLER_LINK_FRAME_TIMEOUT_MS))
        {
            ParserReset();
        }
        while ((processedBytes < CONTROLLER_LINK_RX_BATCH_LIMIT) && RxRingPop(&byte))
        {
            ParserConsumeByte(byte);
            processedBytes++;
        }
        PollApplicationMessages(now, &lastInputTxTick, &lastUiTxTick);
        ProcessPendingTransmit();
#if CONTROLLER_LINK_TEST_MODE
        if ((TickType_t)(now - lastPingTick) >=
            pdMS_TO_TICKS(CONTROLLER_LINK_TEST_PING_PERIOD_MS))
        {
            lastPingTick = now;
            (void)HandsetLink_QueueMessage(HANDSET_LINK_MSG_PING, 0U, NULL, 0U);
        }
        if ((TickType_t)(now - lastTestInputTick) >=
            pdMS_TO_TICKS(CONTROLLER_LINK_TEST_INPUT_PERIOD_MS))
        {
            lastTestInputTick = now;
            StoreTestInput();
        }
        ProcessPendingTransmit();
#else
        (void)lastPingTick;
        (void)lastTestInputTick;
#endif
        if ((processedBytes >= CONTROLLER_LINK_RX_BATCH_LIMIT) && (RxRingIsEmpty() == 0U))
        {
            vTaskDelay(pdMS_TO_TICKS(CONTROLLER_LINK_RX_BACKLOG_YIELD_MS));
            continue;
        }
        if (gRxArmed == 0U)
        {
            ArmReceiver();
            if (gRxArmed == 0U)
            {
                vTaskDelay(pdMS_TO_TICKS(1U));
                continue;
            }
        }
#if CONTROLLER_LINK_TEST_MODE
        if (RxRingIsEmpty() != 0U)
        {
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10U));
        }
#else
        if (RxRingIsEmpty() != 0U)
        {
            if (gTxActive != 0U)
            {
                ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10U));
            }
            else
            {
                ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(5U));
            }
        }
#endif
    }
}
