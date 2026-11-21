#ifndef __TASK_H
#define __TASK_H

#include <stdint.h>
#include "controller_types.h"







void KEYTASK (void *argument);
void CTRLTASK (void *argument);
void FREQ2_4GTask (void *argument);
void JSTask (void *argument);
void GYROTask (void *argument);

/* 任务间共享数据统一走快照API；读写多个字段时由实现用短临界区保护。 */
void TASK_GetMenuSnapshot(task_menu_snapshot_t *snapshot);
void TASK_PublishJoystickSnapshot(float x_percent, float y_percent, uint32_t sample_tick);
void TASK_PublishGyroSnapshot(float roll_deg, float pitch_deg, float yaw_deg,
	float height_m, uint32_t sample_tick);
void TASK_PublishButtonSnapshot(uint8_t button_bitmap, uint8_t event_key,
	uint16_t event_sequence);
void TASK_ClearButtonEvent(void);
uint8_t TASK_AcknowledgeButtonEvent(uint16_t event_sequence);
uint8_t TASK_GetInputSnapshot(controller_input_snapshot_t *snapshot);
uint8_t TASK_GetPeerInputSnapshot(controller_peer_input_t *snapshot);
void TASK_PublishPeerInput(const controller_input_snapshot_t *snapshot);
void TASK_ResetPeerInput(void);
void TASK_PublishFeatureValue(uint8_t index, float value);
void TASK_ClearFeatureValues(void);
void TASK_SetRemoteMenuSnapshot(const task_menu_snapshot_t *snapshot);
/* Floor4同步自旋状态；自旋速度由LEFT锁存并由MW/JS控制任务读取。 */
void TASK_SetMwSpinState(uint8_t state);
void TASK_SetMwSpinSpeed(int8_t speed_percent);
int8_t TASK_GetMwSpinSpeed(void);







extern volatile uint8_t DoubleModle;
extern volatile int8_t Floor1;
extern volatile int8_t Floor2;
extern volatile int8_t Floor3;
extern volatile int8_t Floor4;
extern volatile int8_t FloorSwitch;







#endif

