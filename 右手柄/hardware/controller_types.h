#ifndef CONTROLLER_TYPES_H
#define CONTROLLER_TYPES_H

#include <stdint.h>

/* SOLO=分体单机；CONNECTING=触点消抖或握手中；DUAL=双机链路有效；FAULT=等待恢复。 */
typedef enum
{
	CONTROLLER_LINK_STATE_SOLO = 0U,
	CONTROLLER_LINK_STATE_CONNECTING,
	CONTROLLER_LINK_STATE_DUAL,
	CONTROLLER_LINK_STATE_FAULT
} controller_link_state_t;

/* 业务显示快照，不是线上的序列化格式；发送时必须逐字段编码。 */
typedef struct
{
	int8_t Floor1;
	int8_t Floor2;
	int8_t Floor3;
	int8_t Floor4; /* 双手柄自旋状态：0=关、1=直控、2=设速、3=锁速。 */
	int8_t FloorSwitch;
	uint8_t DoubleModle;
	uint8_t ControlEnabled;
	uint8_t ConnectionState;
	uint8_t DisplayValueValidMask;
	int16_t DisplayValue[4];
} controller_ui_state_t;

typedef controller_ui_state_t task_menu_snapshot_t;

/* 本机最新采样快照；多字段读写由 TASK API 在短临界区保护。 */
typedef struct
{
/* 最新摇杆/姿态样本；有效位说明数据是否存在，序号区分新样本。 */
	float joystick_x_percent;
	float joystick_y_percent;
	float gyro_roll_deg;
	float gyro_pitch_deg;
	float gyro_yaw_deg;
	float gyro_height_m;
/* 采样或接收新样本时的本机RTOS tick，用于输入过期保护。 */
	uint32_t joystick_sample_tick;
	uint32_t gyro_sample_tick;
	uint16_t joystick_sample_sequence;
	uint16_t gyro_sample_sequence;
	/* 位图表示按住状态；按键事件带独立序号，确认后才清除待处理事件。 */
	uint16_t button_event_sequence;
	uint8_t button_bitmap;
	uint8_t button_event_key;
	uint8_t joystick_valid;
	uint8_t gyro_valid;
} controller_input_snapshot_t;

typedef struct
{
	controller_input_snapshot_t input;
/* 收帧时间与样本更新时间分开记录：链路有帧不代表控制输入仍然新鲜。 */
	uint32_t last_frame_tick;
	uint32_t last_input_update_tick;
} controller_peer_input_t;

/* 共同测试消息的定点数载荷；接收后可在调试器中直接观察。 */
typedef struct
{
	uint16_t sequence;
	uint8_t valid_flags;
	int16_t joystick_x_centi_percent;
	int16_t joystick_y_centi_percent;
	int16_t gyro_roll_centi_deg;
	int16_t gyro_pitch_centi_deg;
	int16_t gyro_yaw_centi_deg;
} controller_test_input_t;

#endif /* CONTROLLER_TYPES_H */
