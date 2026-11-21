#ifndef CONTROLLER_MODE_H
#define CONTROLLER_MODE_H

#include <stdint.h>
#include "FreeRTOS.h"
#include "controller_types.h"

/* 连接状态副本；radio_handoff_ready表示本机停机发送调用完成，不代表对端已收到。 */
typedef struct
{
	controller_link_state_t state;
	uint32_t connection_id;
	uint32_t local_nonce;
	uint32_t peer_nonce;
	TickType_t last_peer_heartbeat_tick;
	uint8_t local_role;
	uint8_t peer_role;
	uint8_t contact_ready;
	uint8_t contact_active;
	uint8_t radio_handoff_ready;
} controller_mode_snapshot_t;

/* 由 5 ms 控制任务调用，完成触点消抖、连接超时和故障状态维护。 */
uint8_t ControllerMode_UpdateContact(uint8_t contact_active, TickType_t now);
void ControllerMode_GetSnapshot(controller_mode_snapshot_t *snapshot);
controller_link_state_t ControllerMode_GetState(void);
uint8_t ControllerMode_CanControl(void);
uint8_t ControllerMode_CanSendRadio(void);
void ControllerMode_NotifyRadioStopped(void);
uint8_t ControllerMode_GetLocalRole(void);

/* 链路任务调用；只接受对端固定身份、当前连接编号和合法载荷。 */
uint8_t ControllerMode_OnLinkMessage(uint8_t message_type, uint32_t connection_id,
	uint16_t sequence, const uint8_t *payload, uint8_t payload_length, TickType_t now);

/* 链路任务调用；有握手、心跳待发时返回1，否则返回0。 */
uint8_t ControllerMode_PollLink(TickType_t now);

#endif
