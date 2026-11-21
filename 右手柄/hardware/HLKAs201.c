#include "Mydef.h" 

volatile uint8_t HLKAs201_Dma[HLKAS201_DMA_BUFFER_SIZE]={0};
uint8_t HLKAs201_Data[HLKAS201_REPORT_FRAME_SIZE]={0};
float HLKAs201_Data_A[3]={0};//X,Y,Z   m^2/s
float HLKAs201_Data_G[3]={0};//X,Y,Z    度/S
float HLKAs201_Data_Euler[3]={0};//R,P,Y  度
float HLKAs201_Data_M[3]={0};//X,Y,Z uT
float HLKAs201_Data_Q[4]={0};//四元数 W，X，Y，Z
float HLKAs201_Data_T[1]={0};//摄氏度
float HLKAs201_Data_P[1]={0};//Pa
float HLKAs201_Data_H[1]={0};//M






/* 这些解析状态只有GYROTask访问；无需中断回调，不增加ISR耗时。 */
static uint8_t gHlkFrame[HLKAS201_REPORT_FRAME_SIZE];
static uint8_t gHlkFrameLength;
static uint16_t gHlkDmaReadIndex;
static TickType_t gHlkLastPollTick;
static uint8_t gHlkRxStarted;

void HLKAs201_init(void)
{
    gHlkFrameLength = 0U;
    gHlkDmaReadIndex = 0U;
    gHlkLastPollTick = xTaskGetTickCount();
    /* 沿用USART1循环DMA；启动失败时不把未收到的数据发布成有效姿态。 */
    gHlkRxStarted = (HAL_UART_Receive_DMA(&huart1, (uint8_t *)HLKAs201_Dma,
        HLKAS201_DMA_BUFFER_SIZE) == HAL_OK) ? 1U : 0U;
}

/* 厂家V1.1协议：FA FB | len | cmd | data | 累加和 | FC FD。
 * 当前完整上报长50字节，len=45；校验覆盖cmd和data，不包含帧头/长度/帧尾。
 */
static uint8_t HlkReportIsValid(void)
{
    uint8_t checksum = 0U;
    uint8_t index;
    if (gHlkFrame[2] != HLKAS201_REPORT_LENGTH ||
        gHlkFrame[3] != HLKAS201_REPORT_COMMAND ||
        gHlkFrame[HLKAS201_REPORT_FRAME_SIZE - 2U] != HLKAS201_TAIL_0 ||
        gHlkFrame[HLKAS201_REPORT_FRAME_SIZE - 1U] != HLKAS201_TAIL_1)
    {
        return 0U;
    }
    for (index = 3U; index < HLKAS201_CHECKSUM_OFFSET; index++)
    {
        checksum = (uint8_t)(checksum + gHlkFrame[index]);
    }
    return (checksum == gHlkFrame[HLKAS201_CHECKSUM_OFFSET]) ? 1U : 0U;
}

/* 坏帧后保留后面的候选帧头，避免丢一个字节后始终错位。
 * 单独的末尾FA也保留，下一次调用收到FB后仍可继续拼帧。
 */
static void HlkKeepPossibleFrameHead(void)
{
    uint8_t index;
    for (index = 1U; index < gHlkFrameLength; index++)
    {
        if (gHlkFrame[index] == HLKAS201_HEAD_0 &&
            (index + 1U == gHlkFrameLength || gHlkFrame[index + 1U] == HLKAS201_HEAD_1))
        {
            gHlkFrameLength = (uint8_t)(gHlkFrameLength - index);
            memmove(gHlkFrame, &gHlkFrame[index], gHlkFrameLength);
            return;
        }
    }
    gHlkFrameLength = 0U;
}

uint8_t HLKAs201_getData(uint32_t *sample_tick)
{
    uint16_t remaining;
    uint16_t writeIndex;
    uint32_t bufferFillMs;
    TickType_t now = xTaskGetTickCount();
    uint8_t updated = 0U;
    uint8_t byte;

    if (sample_tick == NULL || gHlkRxStarted == 0U || huart1.hdmarx == NULL ||
        huart1.RxState != HAL_UART_STATE_BUSY_RX || huart1.Init.BaudRate == 0U)
    {
        return 0U;
    }
    /* CNDTR给出DMA剩余容量；只读写指针之前已写完的字节，保留跨调用的半帧。 */
    remaining = (uint16_t)__HAL_DMA_GET_COUNTER(huart1.hdmarx);
    if (remaining > HLKAS201_DMA_BUFFER_SIZE)
    {
        return 0U;
    }
    writeIndex = (uint16_t)((HLKAS201_DMA_BUFFER_SIZE - remaining) % HLKAS201_DMA_BUFFER_SIZE);
    bufferFillMs = HLKAS201_DMA_BUFFER_SIZE * HLKAS201_UART_BITS_PER_BYTE * 1000U /
        huart1.Init.BaudRate;
    /* 115200下150字节约13ms。长时间未轮询时无法区分绕圈次数，直接丢弃缓存。
     * 宁可等待下一完整帧，也不能把可能被覆盖或很旧的数据打上当前时间。
     */
    if (bufferFillMs == 0U ||
        (TickType_t)(now - gHlkLastPollTick) >= pdMS_TO_TICKS(bufferFillMs))
    {
        gHlkDmaReadIndex = writeIndex;
        gHlkFrameLength = 0U;
        gHlkLastPollTick = now;
        return 0U;
    }
    gHlkLastPollTick = now;
    while (gHlkDmaReadIndex != writeIndex)
    {
        byte = HLKAs201_Dma[gHlkDmaReadIndex];
        gHlkDmaReadIndex = (uint16_t)((gHlkDmaReadIndex + 1U) % HLKAS201_DMA_BUFFER_SIZE);
        if (gHlkFrameLength == 0U && byte != HLKAS201_HEAD_0)
        {
            continue;
        }
        gHlkFrame[gHlkFrameLength++] = byte;
        if ((gHlkFrameLength == 2U && byte != HLKAS201_HEAD_1) ||
            (gHlkFrameLength >= 3U && gHlkFrame[2] != HLKAS201_REPORT_LENGTH))
        {
            HlkKeepPossibleFrameHead();
        }
        else if (gHlkFrameLength == HLKAS201_REPORT_FRAME_SIZE)
        {
            if (HlkReportIsValid() != 0U)
            {
                memcpy(HLKAs201_Data, gHlkFrame, sizeof(HLKAs201_Data));
                updated = 1U;
                gHlkFrameLength = 0U;
            }
            else
            {
                HlkKeepPossibleFrameHead();
            }
        }
    }
    if (updated != 0U)
    {
        /* 使用本轮读取DMA指针的时刻；即使之后被高优先级任务抢占也不补刷新时间。 */
        *sample_tick = (uint32_t)now;
    }
    /* 即使数值完全相同，新接收的有效帧也会返回1；静止不能当成断流。 */
    return updated;
}

void HLKAs201_getData_A (void){

int16_t A_cache[3]={0};


A_cache[0]=((HLKAs201_Data[6]<<8)|HLKAs201_Data[7]);
A_cache[1]=((HLKAs201_Data[8]<<8)|HLKAs201_Data[9]);
A_cache[2]=((HLKAs201_Data[10]<<8)|HLKAs201_Data[11]);




HLKAs201_Data_A[0]=A_cache[0]*0.00478515625f;
HLKAs201_Data_A[1]=A_cache[1]*0.00478515625f;
HLKAs201_Data_A[2]=A_cache[2]*0.00478515625f;


}


void HLKAs201_getData_G (void){

int16_t G_cache[3]={0};


G_cache[0]=((HLKAs201_Data[12]<<8)|HLKAs201_Data[13]);
G_cache[1]=((HLKAs201_Data[14]<<8)|HLKAs201_Data[15]);
G_cache[2]=((HLKAs201_Data[16]<<8)|HLKAs201_Data[17]);




HLKAs201_Data_G[0]=G_cache[0]*0.0625f;
HLKAs201_Data_G[1]=G_cache[1]*0.0625f;
HLKAs201_Data_G[2]=G_cache[2]*0.0625f;


}

void HLKAs201_getData_Euler (void){

int16_t Euler_cache[3]={0};


Euler_cache[0]=((HLKAs201_Data[18]<<8)|HLKAs201_Data[19]);
Euler_cache[1]=((HLKAs201_Data[20]<<8)|HLKAs201_Data[21]);
Euler_cache[2]=((HLKAs201_Data[22]<<8)|HLKAs201_Data[23]);




HLKAs201_Data_Euler[0]=Euler_cache[0]*0.0054931640625f;
HLKAs201_Data_Euler[1]=Euler_cache[1]*0.0054931640625f;
HLKAs201_Data_Euler[2]=Euler_cache[2]*0.0054931640625f;


}

void HLKAs201_getData_M (void){

int16_t M_cache[3]={0};


M_cache[0]=((HLKAs201_Data[24]<<8)|HLKAs201_Data[25]);
M_cache[1]=((HLKAs201_Data[26]<<8)|HLKAs201_Data[27]);
M_cache[2]=((HLKAs201_Data[28]<<8)|HLKAs201_Data[29]);




HLKAs201_Data_M[0]=M_cache[0]*0.006103515625f;
HLKAs201_Data_M[1]=M_cache[1]*0.006103515625f;
HLKAs201_Data_M[2]=M_cache[2]*0.006103515625f;


}

void HLKAs201_getData_Q (void){

uint16_t Q_cache[4]={0};


Q_cache[0]=((HLKAs201_Data[30]<<8)|HLKAs201_Data[31]);
Q_cache[1]=((HLKAs201_Data[32]<<8)|HLKAs201_Data[33]);
Q_cache[2]=((HLKAs201_Data[34]<<8)|HLKAs201_Data[35]);
Q_cache[3]=((HLKAs201_Data[36]<<8)|HLKAs201_Data[37]);



HLKAs201_Data_Q[0]=Q_cache[0]*0.000030517578125f;
HLKAs201_Data_Q[1]=Q_cache[1]*0.000030517578125f;
HLKAs201_Data_Q[2]=Q_cache[2]*0.000030517578125f;
HLKAs201_Data_Q[3]=Q_cache[3]*0.000030517578125f;

}

void HLKAs201_getData_T (void){

int16_t T_cache[1]={0};


T_cache[0]=((HLKAs201_Data[38]<<8)|HLKAs201_Data[39]);



HLKAs201_Data_T[0]=T_cache[0]*0.01f;


}


void HLKAs201_getData_P (void){

uint32_t P_cache[1]={0};


P_cache[0]=((((HLKAs201_Data[40]<<24)|HLKAs201_Data[41])<<16)|((HLKAs201_Data[42]<<8)|HLKAs201_Data[43]));



HLKAs201_Data_P[0]=P_cache[0]*0.0002384185791f;


}


void HLKAs201_getData_H (void){

int32_t H_cache[1]={0};


H_cache[0]=((((HLKAs201_Data[44]<<24)|HLKAs201_Data[45])<<16)|((HLKAs201_Data[46]<<8)|HLKAs201_Data[47]));



HLKAs201_Data_H[0]=H_cache[0]*0.0010728836f;


}
