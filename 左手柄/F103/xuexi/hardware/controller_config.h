#ifndef CONTROLLER_CONFIG_H
#define CONTROLLER_CONFIG_H

/* 双手柄共同阶段的周期与超时配置，所有时间单位均为 ms。 */
#define CONTROLLER_CONTROL_PERIOD_MS                 5U
#define CONTROLLER_JOYSTICK_SAMPLE_PERIOD_MS         5U
#define CONTROLLER_GYRO_SAMPLE_PERIOD_MS             1U
#define CONTROLLER_INPUT_TIMEOUT_MS                  100U
#define CONTROLLER_UI_SYNC_PERIOD_MS                 50U
#define CONTROLLER_MERGE_DEBOUNCE_MS                 30U
#define CONTROLLER_LINK_HEARTBEAT_PERIOD_MS          20U
#define CONTROLLER_LINK_TIMEOUT_MS                   100U
#define CONTROLLER_LINK_HELLO_PERIOD_MS              50U
#define CONTROLLER_LINK_CONNECT_TIMEOUT_MS           500U
#define CONTROLLER_BUTTON_POLL_PERIOD_MS             10U
#define CONTROLLER_LINK_INPUT_PERIOD_MS              10U
#define CONTROLLER_LINK_UI_PERIOD_MS                 50U
#define CONTROLLER_MW_SPIN_DEADZONE_PERCENT          5U
#define CONTROLLER_MW_SPIN_LIMIT_PERCENT             100U
/** 右手柄方向键按住时叠加到麦轮平移命令的速度百分比。 */
#define CONTROLLER_MW_BUTTON_STEP_PERCENT            20U

#define CONTROLLER_ROLE_LEFT                         1U
#define CONTROLLER_ROLE_RIGHT                        2U
/* 每个副本固定一个身份：左工程选 LEFT，右工程选 RIGHT；其余协议参数保持一致。 */
#define CONTROLLER_LOCAL_ROLE                        CONTROLLER_ROLE_LEFT
#if (CONTROLLER_LOCAL_ROLE != CONTROLLER_ROLE_LEFT) && \
	(CONTROLLER_LOCAL_ROLE != CONTROLLER_ROLE_RIGHT)
#error "CONTROLLER_LOCAL_ROLE must be LEFT or RIGHT"
#endif

#define CONTROLLER_BUTTON_U                          0x01U
#define CONTROLLER_BUTTON_D                          0x02U
#define CONTROLLER_BUTTON_L                          0x04U
#define CONTROLLER_BUTTON_R                          0x08U
#define CONTROLLER_BUTTON_PA8                        0x10U
#define CONTROLLER_KEY_NONE                          0U
#define CONTROLLER_KEY_U                             1U
#define CONTROLLER_KEY_D                             2U
#define CONTROLLER_KEY_L                             3U
#define CONTROLLER_KEY_R                             4U
/* R启用实时自旋控制，D停止，PA8用于设定/锁存右摇杆X速度。 */
#define CONTROLLER_KEY_PA8                           5U
#define CONTROLLER_KEY_SPIN_START                    6U
#define CONTROLLER_KEY_SPIN_STOP                     7U
#define CONTROLLER_KEY_MAX                           CONTROLLER_KEY_SPIN_STOP
/* Floor4经现有状态帧同步：关闭、摇杆直控、设速、锁定恒速。 */
#define CONTROLLER_MW_SPIN_STATE_OFF                 0U
#define CONTROLLER_MW_SPIN_STATE_LIVE                1U
#define CONTROLLER_MW_SPIN_STATE_SETUP               2U
#define CONTROLLER_MW_SPIN_STATE_LOCKED              3U

/* 下面的偏移量描述 USART3 线上帧；本机结构体不能直接 memcpy 到串口。 */
#define CONTROLLER_PROTOCOL_VERSION                  1U
#define CONTROLLER_UART_BAUDRATE                      115200U
#define CONTROLLER_LINK_RX_BUFFER_SIZE               128U
#define CONTROLLER_LINK_RX_BATCH_LIMIT               32U
#define CONTROLLER_LINK_RX_BACKLOG_YIELD_MS           1U
#define CONTROLLER_LINK_MAX_PAYLOAD_SIZE              32U
#define CONTROLLER_LINK_INPUT_PAYLOAD_SIZE            21U
/* 菜单/状态9字节 + 4个int16显示值8字节，最后一个值也必须完整传输。 */
#define CONTROLLER_LINK_UI_STATE_PAYLOAD_SIZE         17U
#define CONTROLLER_LINK_HEARTBEAT_PAYLOAD_SIZE        10U
#define CONTROLLER_LINK_TASK_STACK_WORDS              256U
#define CONTROLLER_LINK_SYNC_SIZE                     2U
#define CONTROLLER_LINK_HEADER_SIZE                   11U
#define CONTROLLER_LINK_CRC_SIZE                      2U
#define CONTROLLER_LINK_SYNC_OFFSET                   0U
#define CONTROLLER_LINK_VERSION_OFFSET                2U
#define CONTROLLER_LINK_MESSAGE_TYPE_OFFSET            3U
#define CONTROLLER_LINK_PAYLOAD_LENGTH_OFFSET          4U
#define CONTROLLER_LINK_CONNECTION_ID_OFFSET           5U
#define CONTROLLER_LINK_SEQUENCE_OFFSET                9U
#define CONTROLLER_LINK_PAYLOAD_OFFSET                 11U
#define CONTROLLER_LINK_MAX_FRAME_SIZE \
	(CONTROLLER_LINK_HEADER_SIZE + CONTROLLER_LINK_MAX_PAYLOAD_SIZE + CONTROLLER_LINK_CRC_SIZE)
#define CONTROLLER_LINK_FRAME_TIMEOUT_MS              20U
#define CONTROLLER_LINK_CRC16_POLYNOMIAL               0x1021U
#define CONTROLLER_LINK_CRC16_INITIAL                  0xFFFFU
#define CONTROLLER_LINK_CRC16_XOR_OUT                  0x0000U
#define CONTROLLER_LINK_CRC16_REFLECT_INPUT            0U
#define CONTROLLER_LINK_CRC16_REFLECT_OUTPUT           0U
#define CONTROLLER_LINK_SYNC_BYTE_0                    0xA5U
#define CONTROLLER_LINK_SYNC_BYTE_1                    0x5AU

/* PING/PONG 和测试输入只用于显式打开的调试模式，默认关闭。 */
#define CONTROLLER_TEST_INPUT_PAYLOAD_SIZE             11U
#define CONTROLLER_TEST_INPUT_VALID_FLAGS_OFFSET       0U
#define CONTROLLER_TEST_INPUT_JOYSTICK_X_OFFSET        1U
#define CONTROLLER_TEST_INPUT_JOYSTICK_Y_OFFSET        3U
#define CONTROLLER_TEST_INPUT_GYRO_ROLL_OFFSET         5U
#define CONTROLLER_TEST_INPUT_GYRO_PITCH_OFFSET        7U
#define CONTROLLER_TEST_INPUT_GYRO_YAW_OFFSET          9U
#define CONTROLLER_TEST_INPUT_JOYSTICK_VALID           0x01U
#define CONTROLLER_TEST_INPUT_GYRO_VALID               0x02U
#define CONTROLLER_LINK_TEST_PING_PERIOD_MS            1000U
#define CONTROLLER_LINK_TEST_INPUT_PERIOD_MS           50U
#ifndef CONTROLLER_LINK_TEST_MODE
#define CONTROLLER_LINK_TEST_MODE                      0U
#endif

/* 原生 FreeRTOS 任务优先级：数值越大越高。 */
#define CONTROLLER_TASK_PRIORITY_OLED                  8U
#define CONTROLLER_TASK_PRIORITY_KEY                   20U
#define CONTROLLER_TASK_PRIORITY_24G                   23U
#define CONTROLLER_TASK_PRIORITY_SAMPLE                24U
#define CONTROLLER_TASK_PRIORITY_CONTROL               25U
#define CONTROLLER_TASK_PRIORITY_LINK                  26U

/* Cortex-M NVIC 优先级数值越小越高；5 是当前可调用 FreeRTOS ISR API 的上限。 */
#define CONTROLLER_USART3_IRQ_PRIORITY                  5U

#if ((CONTROLLER_LINK_RX_BUFFER_SIZE & (CONTROLLER_LINK_RX_BUFFER_SIZE - 1U)) != 0U)
#error "CONTROLLER_LINK_RX_BUFFER_SIZE must be a power of two"
#endif

#endif /* CONTROLLER_CONFIG_H */
