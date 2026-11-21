#ifndef __FREQ2_4G_H
#define __FREQ2_4G_H

/* SPI command base */
#define SI24R1_R_REGISTER           0x00u   /* 读寄存器: 0x00 | reg */
#define SI24R1_W_REGISTER           0x20u   /* 写寄存器: 0x20 | reg */

/* SPI command fixed *///SPI命令
#define SI24R1_R_RX_PL_WID          0x60u   /* 读取动态负载长度 */
#define SI24R1_R_RX_PAYLOAD         0x61u   /* 读取RX FIFO数据 */
#define SI24R1_W_TX_PAYLOAD         0xA0u   /* 写TX FIFO数据 */
#define SI24R1_W_ACK_PAYLOAD_BASE   0xA8u   /* 写ACK payload: 0xA8 | pipe */
#define SI24R1_W_TX_PAYLOAD_NOACK   0xB0u   /* 无应答发送 */
#define SI24R1_FLUSH_TX             0xE1u   /* 清空TX FIFO */
#define SI24R1_FLUSH_RX             0xE2u   /* 清空RX FIFO */
#define SI24R1_REUSE_TX_PL          0xE3u   /* 重发上一次payload */
#define SI24R1_NOP                  0xFFu   /* 无操作，返回STATUS */


/* Si24R1 register address *///寄存器地址
#define SI24R1_CONFIG         0x00u   /* 配置寄存器 */
#define SI24R1_EN_AA          0x01u   /* 自动应答使能 */
#define SI24R1_SETUP_AW       0x03u   /* 地址宽度配置 */
#define SI24R1_SETUP_RETR     0x04u   /* 自动重发配置 */
#define SI24R1_RX_ADDR_P0     0x0Au   /* 接收地址 Pipe0，5字节 */
#define SI24R1_TX_ADDR        0x10u   /* 发送地址，5字节 */
#define SI24R1_DYNPD          0x1Cu   /* 动态负载使能 */
#define SI24R1_FEATURE        0x1Du   /* 特征寄存器 */
#define SI24R1_RF_CH          0x05u   /* 射频信道寄存器 */
#define SI24R1_EN_RXADDR      0x02u   /* 接收通道使能 */
#define SI24R1_OBSERVE_TX 	  0x08u	  /*发射结果统计*/
#define SI24R1_STATUS  	  	  0x07u	  /*状态寄存器*/
#define SI24R1_RSSI			  0x09u	  /*接收信号强度检测*/ 	
#define SI24R1_RF_SETUP		  0x06u	  /*射频配置*/




#define SI24R1_WriteRegister(X)			0x20u|X		//写X寄存器
#define SI24R1_ReadRegister(X)			0x00u|X		//读X寄存器



typedef struct {

	uint8_t *TXData;	//填想要发送数据地址
	
	uint8_t *RXData;	//填想要接收数据地址
	
	uint8_t TXLen;		//填发送数据长度
	
	uint8_t	RXLen;		//返回接收数据长度
	
	uint8_t STATUS;		//返回当前状态
	
	
}Send_Data;

typedef struct {
	uint8_t *RXData;	//填想要接收数据地址
	
	uint8_t *TXData;	//填想要发送数据地址
	
	uint8_t TXLen;		//填发送数据长度	
	
	uint8_t	RXLen;		//返回接收数据长度
	
	uint8_t STATUS;		//返回当前状态
	

}Read_Data;




typedef struct {

	uint8_t *RXData;	//填想要接收数据地址
	
	uint8_t *TXData;	//填想要发送数据地址
	
	uint8_t TXLen;		//填发送数据长度	
	
	uint8_t	RXLen;		//返回接收数据长度


}SendClc_Data;

typedef struct {

	uint8_t *RXData_P0;	//填想要接收数据地址/*六个通道分别存放六个通道数据*/
	
	uint8_t *RXData_P1;	//填想要接收数据地址
	
	uint8_t *RXData_P2;	//填想要接收数据地址
	
	uint8_t *RXData_P3;	//填想要接收数据地址
	
	uint8_t *RXData_P4;	//填想要接收数据地址
	
	uint8_t *RXData_P5;	//填想要接收数据地址
	
	uint8_t *TXData;	//填想要发送数据地址
	
	uint8_t TXLen;		//填发送数据长度	
	
	uint8_t	RXLen;		//返回接收数据长度

	uint8_t RX_PIN;
	
	
}ReadClc_Data;



void FREQ2_4G_SendInit (void);
void FREQ2_4G_ReadInit (void);

uint8_t FREQ2_4G_SendBag (Send_Data *DATA);
uint8_t FREQ2_4G_ReadBag (Read_Data *DATA);



 

extern uint16_t FREQ2_4G_SendFREQ;
extern uint16_t FREQ2_4G_ReadFREQ;

extern uint64_t FREQ2_4G_SendARR;
extern uint64_t FREQ2_4G_ReadARR;


uint8_t FREQ2_4G_SendClc(SendClc_Data *Data);
uint8_t FREQ2_4G_ReadClc(ReadClc_Data *Data);

void FREQ2_4G_GetSignal(uint8_t* VAL);
void FREQ2_4G_GetlossPack (uint8_t* VAL);

#endif

