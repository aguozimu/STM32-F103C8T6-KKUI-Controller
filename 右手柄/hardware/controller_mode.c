#include "Mydef.h"
#include "controller_config.h"
#include "controller_mode.h"
#include "handset_link.h"

typedef struct
{
	controller_link_state_t state;
	uint32_t connection_id;
	uint32_t local_nonce;
	uint32_t peer_nonce;
	TickType_t candidate_since;
	TickType_t connection_since;
	TickType_t last_hello_tx_tick;
	TickType_t last_ack_tx_tick;
	TickType_t last_heartbeat_tx_tick;
	TickType_t last_peer_heartbeat_tick;
	uint16_t last_peer_heartbeat_sequence;
	uint8_t candidate_active;
	uint8_t candidate_initialized;
	uint8_t contact_ready;
	uint8_t contact_active;
	uint8_t hello_seen;
	uint8_t hello_ack_seen;
	uint8_t hello_ack_pending;
	uint8_t hello_ack_queued;
	uint8_t peer_heartbeat_seen;
	uint8_t peer_heartbeat_sequence_valid;
	uint8_t radio_stopped;
	uint8_t radio_handoff_ready;
} controller_mode_internal_t;

/* 两个任务会读写同一会话；访问状态字段时统一使用FreeRTOS临界区。 */
static controller_mode_internal_t gMode;

static uint8_t OtherRole(void)
{
	return (CONTROLLER_LOCAL_ROLE == CONTROLLER_ROLE_LEFT) ?
		CONTROLLER_ROLE_RIGHT : CONTROLLER_ROLE_LEFT;
}

static uint32_t ReadLe32(const uint8_t *data)
{
	return (uint32_t)data[0] | ((uint32_t)data[1] << 8) |
		((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

static void WriteLe32(uint8_t *data, uint32_t value)
{
	data[0] = (uint8_t)value;
	data[1] = (uint8_t)(value >> 8);
	data[2] = (uint8_t)(value >> 16);
	data[3] = (uint8_t)(value >> 24);
}

static uint32_t MakeLocalNonce(TickType_t now)
{
	uint32_t value = (uint32_t)now ^ (uint32_t)(uintptr_t)&gMode ^
		((uint32_t)CONTROLLER_LOCAL_ROLE << 24) ^ 0xA53C91E7UL;
	value = value * 1664525UL + 1013904223UL;
	return (value == 0U) ? 1U : value;
}

static uint8_t Elapsed(TickType_t now, TickType_t since, uint32_t period_ms)
{
	return ((TickType_t)(now - since) >= pdMS_TO_TICKS(period_ms)) ? 1U : 0U;
}

/* 触点正在消抖时先对外报告CONNECTING，避免旧连接状态继续放行控制。 */
static controller_link_state_t EffectiveStateLocked(void)
{
	if (gMode.contact_ready == 0U ||
		(gMode.candidate_initialized != 0U &&
		gMode.candidate_active != gMode.contact_active))
	{
		return CONTROLLER_LINK_STATE_CONNECTING;
	}
	return gMode.state;
}

/* 稳定合体或分开后重建会话，清掉旧握手/心跳信息，防止复用上一轮连接。 */
static void StartContactSessionLocked(TickType_t now, uint8_t active)
{
	uint8_t radioStoppedBeforeSession = gMode.radio_stopped;
	gMode.contact_active = active;
	gMode.connection_id = 0U;
	gMode.local_nonce = active != 0U ? MakeLocalNonce(now) : 0U;
	gMode.peer_nonce = 0U;
	gMode.connection_since = now;
	gMode.last_hello_tx_tick = 0U;
	gMode.last_ack_tx_tick = 0U;
	gMode.last_heartbeat_tx_tick = 0U;
	gMode.last_peer_heartbeat_tick = now;
	gMode.last_peer_heartbeat_sequence = 0U;
	gMode.hello_seen = 0U;
	gMode.hello_ack_seen = 0U;
	gMode.hello_ack_pending = 0U;
	gMode.hello_ack_queued = 0U;
	gMode.peer_heartbeat_seen = 0U;
	gMode.peer_heartbeat_sequence_valid = 0U;
	/* 触点正在合体消抖时已发出的停机帧仍有效，保留其确认标志。 */
	gMode.radio_stopped = active != 0U ? radioStoppedBeforeSession : 0U;
	gMode.radio_handoff_ready = 0U;
	gMode.state = active != 0U ? CONTROLLER_LINK_STATE_CONNECTING :
		CONTROLLER_LINK_STATE_SOLO;
}

/* CTRLTASK每5ms调用：先等待触点稳定，再维护连接超时和故障恢复。 */
uint8_t ControllerMode_UpdateContact(uint8_t contact_active, TickType_t now)
{
	uint8_t previous_state;
	uint8_t new_state;
	uint8_t previous_contact;
	uint8_t new_contact;
	uint8_t raw_active = contact_active != 0U ? 1U : 0U;

	taskENTER_CRITICAL();
	previous_state = (uint8_t)EffectiveStateLocked();
	previous_contact = gMode.contact_active;
	if (gMode.candidate_initialized == 0U)
	{
		gMode.candidate_initialized = 1U;
		gMode.candidate_active = raw_active;
		gMode.candidate_since = now;
	}
	else if (raw_active != gMode.candidate_active)
	{
		gMode.candidate_active = raw_active;
		gMode.candidate_since = now;
	}
	else if (gMode.contact_ready == 0U &&
		Elapsed(now, gMode.candidate_since, CONTROLLER_MERGE_DEBOUNCE_MS) != 0U)
	{
		gMode.contact_ready = 1U;
		StartContactSessionLocked(now, gMode.candidate_active);
	}
	else if (gMode.contact_ready != 0U &&
		gMode.contact_active != gMode.candidate_active &&
		Elapsed(now, gMode.candidate_since, CONTROLLER_MERGE_DEBOUNCE_MS) != 0U)
	{
		StartContactSessionLocked(now, gMode.candidate_active);
	}

	if (gMode.contact_ready != 0U && gMode.contact_active != 0U)
	{
		if (gMode.state == CONTROLLER_LINK_STATE_DUAL &&
			Elapsed(now, gMode.last_peer_heartbeat_tick, CONTROLLER_LINK_TIMEOUT_MS) != 0U)
		{
			gMode.state = CONTROLLER_LINK_STATE_FAULT;
			gMode.connection_since = now;
			gMode.peer_heartbeat_seen = 0U;
			gMode.radio_handoff_ready = 0U;
		}
		else if (gMode.state == CONTROLLER_LINK_STATE_CONNECTING &&
			Elapsed(now, gMode.connection_since, CONTROLLER_LINK_CONNECT_TIMEOUT_MS) != 0U)
		{
			gMode.state = CONTROLLER_LINK_STATE_FAULT;
		}
	}
	new_state = (uint8_t)EffectiveStateLocked();
	new_contact = gMode.contact_active;
	taskEXIT_CRITICAL();
	return (previous_state != new_state || previous_contact != new_contact) ? 1U : 0U;
}

void ControllerMode_GetSnapshot(controller_mode_snapshot_t *snapshot)
{
	if (snapshot == NULL)
	{
		return;
	}
	taskENTER_CRITICAL();
	snapshot->state = EffectiveStateLocked();
	snapshot->connection_id = gMode.connection_id;
	snapshot->local_nonce = gMode.local_nonce;
	snapshot->peer_nonce = gMode.peer_nonce;
	snapshot->last_peer_heartbeat_tick = gMode.last_peer_heartbeat_tick;
	snapshot->local_role = CONTROLLER_LOCAL_ROLE;
	snapshot->peer_role = gMode.hello_seen != 0U || gMode.hello_ack_seen != 0U ?
		OtherRole() : 0U;
	snapshot->contact_ready = gMode.contact_ready;
	snapshot->contact_active = gMode.contact_active;
	snapshot->radio_handoff_ready = gMode.radio_handoff_ready;
	taskEXIT_CRITICAL();
}

controller_link_state_t ControllerMode_GetState(void)
{
	controller_mode_snapshot_t snapshot;
	ControllerMode_GetSnapshot(&snapshot);
	return snapshot.state;
}

/* SOLO允许本机控制；DUAL只允许LEFT控制，且须先收到RIGHT停机发送完成标志。 */
uint8_t ControllerMode_CanControl(void)
{
	controller_mode_snapshot_t snapshot;
	ControllerMode_GetSnapshot(&snapshot);
	if (snapshot.state == CONTROLLER_LINK_STATE_SOLO)
	{
		return 1U;
	}
	return (snapshot.state == CONTROLLER_LINK_STATE_DUAL &&
		CONTROLLER_LOCAL_ROLE == CONTROLLER_ROLE_LEFT &&
		snapshot.radio_handoff_ready != 0U) ? 1U : 0U;
}

uint8_t ControllerMode_CanSendRadio(void)
{
	return ControllerMode_CanControl();
}

uint8_t ControllerMode_GetLocalRole(void)
{
	return CONTROLLER_LOCAL_ROLE;
}

void ControllerMode_NotifyRadioStopped(void)
{
#if CONTROLLER_LOCAL_ROLE == CONTROLLER_ROLE_RIGHT
	taskENTER_CRITICAL();
	gMode.radio_stopped = 1U;
	taskEXIT_CRITICAL();
#endif
}

/* 按固定左右身份、连接号和双方nonce核对消息，拒绝旧会话或错误角色的数据。 */
/* nonce用于区分连接轮次，不是加密认证。 */
uint8_t ControllerMode_OnLinkMessage(uint8_t message_type, uint32_t connection_id,
	uint16_t sequence, const uint8_t *payload, uint8_t payload_length, TickType_t now)
{
	uint8_t valid = 0U;
	uint8_t peer_role = OtherRole();
	uint32_t nonce_a;
	uint32_t nonce_b;

	if ((payload == NULL) || (connection_id == 0U))
	{
		return 0U;
	}

	taskENTER_CRITICAL();
	if (gMode.contact_ready == 0U || gMode.contact_active == 0U)
	{
		taskEXIT_CRITICAL();
		return 0U;
	}

	switch (message_type)
	{
		case HANDSET_LINK_MSG_HELLO:
			if (CONTROLLER_LOCAL_ROLE == CONTROLLER_ROLE_RIGHT && payload_length == 1U &&
				payload[0] == CONTROLLER_ROLE_LEFT)
			{
				if (gMode.connection_id != connection_id)
				{
					gMode.connection_id = connection_id;
					gMode.peer_nonce = connection_id;
					gMode.connection_since = now;
					gMode.hello_ack_seen = 0U;
					gMode.hello_ack_queued = 0U;
					gMode.peer_heartbeat_seen = 0U;
					gMode.peer_heartbeat_sequence_valid = 0U;
				}
				gMode.hello_seen = 1U;
				gMode.hello_ack_pending = 1U;
				gMode.last_ack_tx_tick = 0U;
				if (gMode.state == CONTROLLER_LINK_STATE_FAULT)
				{
					gMode.state = CONTROLLER_LINK_STATE_CONNECTING;
					gMode.connection_since = now;
				}
				valid = 1U;
			}
			break;

		case HANDSET_LINK_MSG_HELLO_ACK:
			if (CONTROLLER_LOCAL_ROLE == CONTROLLER_ROLE_LEFT && payload_length == 5U &&
				payload[0] == CONTROLLER_ROLE_RIGHT && connection_id == gMode.local_nonce)
			{
				nonce_a = ReadLe32(&payload[1]);
				if (nonce_a != 0U && (gMode.peer_nonce == 0U || gMode.peer_nonce == nonce_a))
				{
					gMode.connection_id = gMode.local_nonce;
					gMode.peer_nonce = nonce_a;
					gMode.hello_ack_seen = 1U;
					if (gMode.state == CONTROLLER_LINK_STATE_FAULT)
					{
						gMode.state = CONTROLLER_LINK_STATE_CONNECTING;
						gMode.connection_since = now;
					}
					valid = 1U;
				}
			}
			break;

		case HANDSET_LINK_MSG_HEARTBEAT:
			if (payload_length == CONTROLLER_LINK_HEARTBEAT_PAYLOAD_SIZE &&
				payload[0] == peer_role && (payload[9] & 0xFEU) == 0U &&
				connection_id == gMode.connection_id && connection_id != 0U)
			{
				nonce_a = ReadLe32(&payload[1]);
				nonce_b = ReadLe32(&payload[5]);
				if (CONTROLLER_LOCAL_ROLE == CONTROLLER_ROLE_LEFT)
				{
					valid = (gMode.hello_ack_seen != 0U && nonce_a == gMode.peer_nonce &&
						nonce_b == gMode.local_nonce) ? 1U : 0U;
				}
				else
				{
					valid = (gMode.hello_seen != 0U && gMode.hello_ack_queued != 0U &&
						nonce_a == gMode.peer_nonce && nonce_b == gMode.local_nonce) ? 1U : 0U;
				}
				if (valid != 0U)
				{
					if (gMode.peer_heartbeat_sequence_valid == 0U ||
						(int16_t)(sequence - gMode.last_peer_heartbeat_sequence) > 0)
					{
						gMode.last_peer_heartbeat_sequence = sequence;
						gMode.peer_heartbeat_sequence_valid = 1U;
						gMode.last_peer_heartbeat_tick = now;
						gMode.peer_heartbeat_seen = 1U;
						if (CONTROLLER_LOCAL_ROLE == CONTROLLER_ROLE_LEFT)
						{
							gMode.radio_handoff_ready = payload[9] & 0x01U;
						}
						gMode.state = CONTROLLER_LINK_STATE_DUAL;
					}
				}
			}
			break;

		default:
			break;
	}
	taskEXIT_CRITICAL();
	return valid;
}

/* LEFT发起HELLO，RIGHT回HELLO_ACK；握手后双方周期发送心跳和状态标志。 */
uint8_t ControllerMode_PollLink(TickType_t now)
{
	uint8_t payload[CONTROLLER_LINK_HEARTBEAT_PAYLOAD_SIZE];
	uint8_t message_type = 0U;
	uint8_t payload_length = 0U;
	uint32_t connection_id = 0U;
	uint32_t local_nonce = 0U;
	uint32_t peer_nonce = 0U;
	uint8_t contact_active;
	uint8_t state;
	uint8_t ack_pending;
	uint8_t ack_seen;
	uint8_t peer_heartbeat_seen;
	uint8_t hello_seen;
	uint8_t radio_stopped;

	taskENTER_CRITICAL();
	contact_active = (gMode.contact_ready != 0U && gMode.contact_active != 0U) ? 1U : 0U;
	state = (uint8_t)gMode.state;
	connection_id = gMode.connection_id;
	local_nonce = gMode.local_nonce;
	peer_nonce = gMode.peer_nonce;
	ack_pending = gMode.hello_ack_pending;
	ack_seen = gMode.hello_ack_seen;
	peer_heartbeat_seen = gMode.peer_heartbeat_seen;
	hello_seen = gMode.hello_seen;
	radio_stopped = gMode.radio_stopped;
	if (contact_active != 0U && CONTROLLER_LOCAL_ROLE == CONTROLLER_ROLE_LEFT &&
		state != CONTROLLER_LINK_STATE_DUAL &&
		Elapsed(now, gMode.last_hello_tx_tick, CONTROLLER_LINK_HELLO_PERIOD_MS) != 0U)
	{
		message_type = HANDSET_LINK_MSG_HELLO;
		payload[0] = CONTROLLER_ROLE_LEFT;
		payload_length = 1U;
		connection_id = local_nonce;
	}
	else if (contact_active != 0U && CONTROLLER_LOCAL_ROLE == CONTROLLER_ROLE_RIGHT &&
		hello_seen != 0U &&
		(ack_pending != 0U || (peer_heartbeat_seen == 0U &&
		Elapsed(now, gMode.last_ack_tx_tick, CONTROLLER_LINK_HELLO_PERIOD_MS) != 0U)))
	{
		message_type = HANDSET_LINK_MSG_HELLO_ACK;
		payload[0] = CONTROLLER_ROLE_RIGHT;
		WriteLe32(&payload[1], local_nonce);
		payload_length = 5U;
		connection_id = gMode.connection_id;
	}
	else if (contact_active != 0U && connection_id != 0U &&
		((CONTROLLER_LOCAL_ROLE == CONTROLLER_ROLE_LEFT && ack_seen != 0U) ||
		 (CONTROLLER_LOCAL_ROLE == CONTROLLER_ROLE_RIGHT && hello_seen != 0U)) &&
		Elapsed(now, gMode.last_heartbeat_tx_tick, CONTROLLER_LINK_HEARTBEAT_PERIOD_MS) != 0U)
	{
		message_type = HANDSET_LINK_MSG_HEARTBEAT;
		payload[0] = CONTROLLER_LOCAL_ROLE;
		WriteLe32(&payload[1], local_nonce);
		WriteLe32(&payload[5], peer_nonce);
		payload[9] = radio_stopped;
		payload_length = CONTROLLER_LINK_HEARTBEAT_PAYLOAD_SIZE;
	}
	taskEXIT_CRITICAL();

	if (message_type == 0U)
	{
		return 0U;
	}
	if (HandsetLink_QueueMessage(message_type, connection_id, payload, payload_length) == 0U)
	{
		return 0U;
	}

	taskENTER_CRITICAL();
	if (message_type == HANDSET_LINK_MSG_HELLO)
	{
		gMode.last_hello_tx_tick = now;
	}
	else if (message_type == HANDSET_LINK_MSG_HELLO_ACK)
	{
		gMode.last_ack_tx_tick = now;
		gMode.hello_ack_pending = 0U;
		gMode.hello_ack_queued = 1U;
	}
	else if (message_type == HANDSET_LINK_MSG_HEARTBEAT)
	{
		gMode.last_heartbeat_tx_tick = now;
	}
	taskEXIT_CRITICAL();
	return 1U;
}
