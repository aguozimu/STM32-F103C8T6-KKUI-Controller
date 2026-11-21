#ifndef HANDSET_LINK_H
#define HANDSET_LINK_H

#include <stdint.h>
#include "controller_types.h"

/* 串口消息类型：0x10段用于握手/心跳，0x20段传输入，0x30段同步UI。 */
typedef enum
{
	HANDSET_LINK_MSG_PING = 0x01U,
	HANDSET_LINK_MSG_PONG = 0x02U,
	HANDSET_LINK_MSG_TEST_INPUT = 0x03U,
	HANDSET_LINK_MSG_HELLO = 0x10U,
	HANDSET_LINK_MSG_HELLO_ACK = 0x11U,
	HANDSET_LINK_MSG_HEARTBEAT = 0x12U,
	HANDSET_LINK_MSG_EVENT_ACK = 0x13U,
	HANDSET_LINK_MSG_INPUT = 0x20U,
	HANDSET_LINK_MSG_UI_STATE = 0x30U
} handset_link_message_type_t;

/* 调试诊断计数器；可在调试器观察CRC、长度、缓冲溢出和收发失败。 */
typedef struct
{
	uint32_t rx_byte_count;
	uint32_t valid_frame_count;
	uint32_t crc_error_count;
	uint32_t length_error_count;
	uint32_t version_error_count;
	uint32_t message_error_count;
	uint32_t unknown_message_count;
	uint32_t rx_ring_overflow_count;
	uint32_t uart_error_count;
	uint32_t rx_rearm_error_count;
	uint32_t rx_arm_retry_error_count;
	uint32_t tx_frame_count;
	uint32_t tx_error_count;
	uint32_t tx_start_error_count;
	uint32_t tx_mailbox_drop_count;
	uint32_t ping_rx_count;
	uint32_t pong_rx_count;
	uint32_t test_input_rx_count;
	uint32_t input_rx_count;
	uint32_t ui_state_rx_count;
	uint16_t last_key_event_ack_sequence;
	uint32_t last_rx_connection_id;
	uint16_t rx_ring_high_water;
	uint16_t last_rx_sequence;
	uint16_t last_pong_echo_sequence;
	uint8_t protocol_self_test_passed;
	uint8_t uart_configuration_matches;
	uint8_t local_role;
	uint8_t peer_role;
	uint8_t connection_state;
	uint8_t rx_armed;
	uint8_t tx_active;
} handset_link_diagnostics_t;

/* 唯一的USART3协议任务：接收解析、消息派发、周期发送都在这里串行处理。 */
void HandsetLink_Task(void *argument);
void HandsetLink_NotifyUIChanged(void);
uint8_t HandsetLink_QueueKeyEventAck(uint16_t event_sequence, uint32_t connection_id);

/* 只能从任务调用；单槽邮箱满时返回0并累计丢弃次数。 */
uint8_t HandsetLink_QueueMessage(uint8_t message_type, uint32_t connection_id,
	const uint8_t *payload, uint8_t payload_length);
uint8_t HandsetLink_GetDiagnostics(handset_link_diagnostics_t *diagnostics);
uint8_t HandsetLink_GetLastTestInput(controller_test_input_t *input);

uint16_t HandsetLink_Crc16(const uint8_t *data, uint16_t length);
uint8_t HandsetLink_ProtocolSelfTest(void);

#endif /* HANDSET_LINK_H */
