#ifndef __HLKAS201_H
#define __HLKAS201_H

#include <stdint.h>

#define MyPi 3.1415926

/* 当前工程使用完整50字节上报；DMA容量沿用原来的150字节。 */
#define HLKAS201_DMA_BUFFER_SIZE          150U
#define HLKAS201_REPORT_FRAME_SIZE        50U
#define HLKAS201_REPORT_LENGTH            (HLKAS201_REPORT_FRAME_SIZE - 5U)
#define HLKAS201_CHECKSUM_OFFSET          (HLKAS201_REPORT_FRAME_SIZE - 3U)
#define HLKAS201_UART_BITS_PER_BYTE       10U
#define HLKAS201_HEAD_0                   0xFAU
#define HLKAS201_HEAD_1                   0xFBU
#define HLKAS201_REPORT_COMMAND           0x00U
#define HLKAS201_TAIL_0                   0xFCU
#define HLKAS201_TAIL_1                   0xFDU


/* HLK传感器继续走USART1 DMA；USART3已留给双手柄链路。 */
void HLKAs201_init (void);
/* 返回1并写入新完整帧的接收观察时刻；返回0时不应刷新姿态时间。 */
uint8_t HLKAs201_getData(uint32_t *sample_tick);
void HLKAs201_getData_A (void);
void HLKAs201_getData_G (void);
void HLKAs201_getData_Euler (void);
void HLKAs201_getData_M (void);
void HLKAs201_getData_Q (void);
void HLKAs201_getData_T (void);
void HLKAs201_getData_P (void);
void HLKAs201_getData_H (void);









/* DMA在任务之外写入，volatile保证每次读取都是内存中的实际字节。 */
extern volatile uint8_t HLKAs201_Dma[];
extern uint8_t HLKAs201_Data[];
extern float HLKAs201_Data_A[];//X,Y,Z   m^2/s
extern float HLKAs201_Data_G[];//X,Y,Z    度/S
extern float HLKAs201_Data_Euler[];//R,P,Y  度
extern float HLKAs201_Data_M[];//X,Y,Z uT
extern float HLKAs201_Data_Q[];//四元数 1，2，3，4
extern float HLKAs201_Data_T[];//摄氏度
extern float HLKAs201_Data_P[];//Pa
extern float HLKAs201_Data_H[];//M







#endif

