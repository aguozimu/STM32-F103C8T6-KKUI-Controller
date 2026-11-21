
#include "Mydef.h"

uint16_t FREQ2_4G_SendFREQ=2400;
uint16_t FREQ2_4G_ReadFREQ=2400;

uint64_t FREQ2_4G_SendARR=0xA5A5A5A5A5;
uint64_t FREQ2_4G_ReadARR=0xA5A5A5A5A5;


void FREQ2_4G_SendInit (void){
////////ACK+ACK包回应模式
uint8_t TX;
uint8_t RX;
	HAL_GPIO_WritePin(SPI_CE_GPIO_Port, SPI_CE_Pin, GPIO_PIN_RESET);
	//上电等待

vTaskDelay(100);
	
	//设置地址长度
	SPI_CSN(0);
	TX=SI24R1_WriteRegister(SI24R1_SETUP_AW);
	HAL_SPI_TransmitReceive(&hspi1,&TX,&RX,1,HAL_MAX_DELAY);
	TX=0x03;						//5个地址
	HAL_SPI_TransmitReceive(&hspi1,&TX,&RX,1,HAL_MAX_DELAY);
	SPI_CSN(1);
	
	//设置动态负载
	SPI_CSN(0);
	TX=SI24R1_WriteRegister(SI24R1_FEATURE);
	HAL_SPI_TransmitReceive(&hspi1,&TX,&RX,1,HAL_MAX_DELAY);
	TX=0x06;
	HAL_SPI_TransmitReceive(&hspi1,&TX,&RX,1,HAL_MAX_DELAY);	
	SPI_CSN(1);	
	
	//打开0通道动态负载
	SPI_CSN(0);
	TX=SI24R1_WriteRegister(SI24R1_DYNPD);
	HAL_SPI_TransmitReceive(&hspi1,&TX,&RX,1,HAL_MAX_DELAY);
	TX=0x01;
	HAL_SPI_TransmitReceive(&hspi1,&TX,&RX,1,HAL_MAX_DELAY);	
	SPI_CSN(1);		
	
	//设置频率
	SPI_CSN(0);
	TX=SI24R1_WriteRegister(SI24R1_RF_CH);
	HAL_SPI_TransmitReceive(&hspi1,&TX,&RX,1,HAL_MAX_DELAY);
	TX=FREQ2_4G_SendFREQ-2400;
	HAL_SPI_TransmitReceive(&hspi1,&TX,&RX,1,HAL_MAX_DELAY);
	SPI_CSN(1);
	
	//设置发送地址
	SPI_CSN(0);
	TX=SI24R1_WriteRegister(SI24R1_TX_ADDR);
	HAL_SPI_TransmitReceive(&hspi1,&TX,&RX,1,HAL_MAX_DELAY);
	for (int i = 0; i < 5; i++)
	{
    TX = (uint8_t)((FREQ2_4G_SendARR >> (8 * i)) & 0xFF);
	HAL_SPI_TransmitReceive(&hspi1,&TX,&RX,1,HAL_MAX_DELAY);
	}
	SPI_CSN(1);
	
	//设置接收0通道地址
	SPI_CSN(0);
	TX=SI24R1_WriteRegister(SI24R1_RX_ADDR_P0);
	HAL_SPI_TransmitReceive(&hspi1,&TX,&RX,1,HAL_MAX_DELAY);
	for (int i = 0; i < 5; i++)
	{
    TX = (uint8_t)((FREQ2_4G_SendARR >> (8 * i)) & 0xFF);
	HAL_SPI_TransmitReceive(&hspi1,&TX,&RX,1,HAL_MAX_DELAY);
	}
	SPI_CSN(1);
	
	
	//设置射频设置
	
	SPI_CSN(0);
	TX=SI24R1_WriteRegister(SI24R1_RF_SETUP);
	HAL_SPI_TransmitReceive(&hspi1,&TX,&RX,1,HAL_MAX_DELAY);
	TX=0x0f; 					
	HAL_SPI_TransmitReceive(&hspi1,&TX,&RX,1,HAL_MAX_DELAY);
	SPI_CSN(1);
	
	//自动重发配置
	SPI_CSN(0);
	TX=SI24R1_WriteRegister(SI24R1_SETUP_RETR);
	HAL_SPI_TransmitReceive(&hspi1,&TX,&RX,1,HAL_MAX_DELAY);
	TX=0xff; 					
	HAL_SPI_TransmitReceive(&hspi1,&TX,&RX,1,HAL_MAX_DELAY);
	SPI_CSN(1);
	
	
	
	//设置模式
	SPI_CSN(0);
	TX=SI24R1_WriteRegister(SI24R1_CONFIG);
	HAL_SPI_TransmitReceive(&hspi1,&TX,&RX,1,HAL_MAX_DELAY);
	TX=0x0E; 					//配置为发射模式，CRC，开机
	HAL_SPI_TransmitReceive(&hspi1,&TX,&RX,1,HAL_MAX_DELAY);
	SPI_CSN(1);
	
	HAL_GPIO_WritePin(SPI_CE_GPIO_Port, SPI_CE_Pin, GPIO_PIN_SET);
	
	
	
}


void FREQ2_4G_ReadInit (void)
{

////////ACK+ACK包回应模式
uint8_t TX;
uint8_t RX;
	HAL_GPIO_WritePin(SPI_CE_GPIO_Port, SPI_CE_Pin, GPIO_PIN_RESET);
	//上电等待

vTaskDelay(100);

	//设置接收地址
	SPI_CSN(0);
	TX=SI24R1_WriteRegister(SI24R1_RX_ADDR_P0);
	HAL_SPI_TransmitReceive(&hspi1,&TX,&RX,1,HAL_MAX_DELAY);
	for (int i = 0; i < 5; i++)
	{
    TX = (uint8_t)((FREQ2_4G_ReadARR >> (8 * i)) & 0xFF);
	HAL_SPI_TransmitReceive(&hspi1,&TX,&RX,1,HAL_MAX_DELAY);
	}
	SPI_CSN(1);

	//使能接收通道1
	SPI_CSN(0);
	TX=SI24R1_WriteRegister(SI24R1_EN_RXADDR);
	HAL_SPI_TransmitReceive(&hspi1,&TX,&RX,1,HAL_MAX_DELAY);
	TX=0x01;
	HAL_SPI_TransmitReceive(&hspi1,&TX,&RX,1,HAL_MAX_DELAY);
	SPI_CSN(1);
	
	// 选择射频信道
	SPI_CSN(0);
	TX=SI24R1_WriteRegister(SI24R1_RF_CH);
	HAL_SPI_TransmitReceive(&hspi1,&TX,&RX,1,HAL_MAX_DELAY);
	TX=FREQ2_4G_ReadFREQ-2400;
	HAL_SPI_TransmitReceive(&hspi1,&TX,&RX,1,HAL_MAX_DELAY);
	SPI_CSN(1);
	
	//设置地址长度
	SPI_CSN(0);
	TX=SI24R1_WriteRegister(SI24R1_SETUP_AW);
	HAL_SPI_TransmitReceive(&hspi1,&TX,&RX,1,HAL_MAX_DELAY);
	TX=0x03;
	HAL_SPI_TransmitReceive(&hspi1,&TX,&RX,1,HAL_MAX_DELAY);
	SPI_CSN(1);	
	
	//动态负载使能
	SPI_CSN(0);
	TX=SI24R1_WriteRegister(SI24R1_DYNPD);
	HAL_SPI_TransmitReceive(&hspi1,&TX,&RX,1,HAL_MAX_DELAY);
	TX=0x01;
	HAL_SPI_TransmitReceive(&hspi1,&TX,&RX,1,HAL_MAX_DELAY);
	SPI_CSN(1);	

	//配置特征寄存器
	SPI_CSN(0);
	TX=SI24R1_WriteRegister(SI24R1_FEATURE);
	HAL_SPI_TransmitReceive(&hspi1,&TX,&RX,1,HAL_MAX_DELAY);
	TX=0x06;
	HAL_SPI_TransmitReceive(&hspi1,&TX,&RX,1,HAL_MAX_DELAY);
	SPI_CSN(1);	
	
		//设置射频设置
	
	SPI_CSN(0);
	TX=SI24R1_WriteRegister(SI24R1_RF_SETUP);
	HAL_SPI_TransmitReceive(&hspi1,&TX,&RX,1,HAL_MAX_DELAY);
	TX=0x27; 					
	HAL_SPI_TransmitReceive(&hspi1,&TX,&RX,1,HAL_MAX_DELAY);
	SPI_CSN(1);
	
	//自动重发配置
	SPI_CSN(0);
	TX=SI24R1_WriteRegister(SI24R1_SETUP_RETR);
	HAL_SPI_TransmitReceive(&hspi1,&TX,&RX,1,HAL_MAX_DELAY);
	TX=0xff; 					
	HAL_SPI_TransmitReceive(&hspi1,&TX,&RX,1,HAL_MAX_DELAY);
	SPI_CSN(1);
	
	//启动RX
	SPI_CSN(0);
	TX=SI24R1_WriteRegister(SI24R1_CONFIG);	
	HAL_SPI_TransmitReceive(&hspi1,&TX,&RX,1,HAL_MAX_DELAY);
	TX=0x0F;
	HAL_SPI_TransmitReceive(&hspi1,&TX,&RX,1,HAL_MAX_DELAY);
	SPI_CSN(1);	
	
	

	
	HAL_GPIO_WritePin(SPI_CE_GPIO_Port, SPI_CE_Pin, GPIO_PIN_SET);


}





uint8_t FREQ2_4G_SendBag (Send_Data *DATA){
	uint8_t TX=SI24R1_W_TX_PAYLOAD;
	uint8_t RX;
	uint8_t RX_Len=0;	
	uint8_t STATUS=0;
	
	//发送数据	
	uint8_t TX_len=DATA->TXLen;

if(TX_len){
	uint8_t* TX_Data=DATA->TXData;
	uint8_t RX_NULL[TX_len];
	
	SPI_CSN(0);
	HAL_SPI_TransmitReceive(&hspi1,&TX,&STATUS,1,HAL_MAX_DELAY);
	HAL_SPI_TransmitReceive(&hspi1,TX_Data,RX_NULL,TX_len,HAL_MAX_DELAY);
	SPI_CSN(1);
};
	//获取ACK数据包字节数
	

	SPI_CSN(0);
	TX=SI24R1_R_RX_PL_WID;
	HAL_SPI_TransmitReceive(&hspi1,&TX,&STATUS,1,HAL_MAX_DELAY);
	TX=0x00;
	HAL_SPI_TransmitReceive(&hspi1,&TX,&RX_Len,1,HAL_MAX_DELAY);
	SPI_CSN(1);
	
	
	if(RX_Len){
	
	//获取ACK数据包
	uint8_t *RX_Data=DATA->RXData;
	uint8_t TX_NULL[RX_Len];
	
	TX=SI24R1_R_RX_PAYLOAD;
	SPI_CSN(0);
	HAL_SPI_TransmitReceive(&hspi1,&TX,&RX,1,HAL_MAX_DELAY);
	HAL_SPI_TransmitReceive(&hspi1,TX_NULL,RX_Data,RX_Len,HAL_MAX_DELAY);
	SPI_CSN(1);	
	};
	
	
	DATA->RXLen=RX_Len;
	DATA->STATUS=STATUS;
	
	
return HAL_OK;
}


uint8_t FREQ2_4G_ReadBag (Read_Data *DATA){


	uint8_t TX=SI24R1_R_RX_PL_WID;
	uint8_t RX=0;
	uint8_t RX_Len=0;
	uint8_t STATUS=0;

	//获取收到字节数
	SPI_CSN(0);
	HAL_SPI_TransmitReceive(&hspi1,&TX,&STATUS,1,HAL_MAX_DELAY);
	TX=0x00;
	HAL_SPI_TransmitReceive(&hspi1,&TX,&RX_Len,1,HAL_MAX_DELAY);
	SPI_CSN(1);

if(RX_Len){
	uint8_t *RX_Data=DATA->RXData;
	uint8_t TX_NULL[RX_Len];
	
	//读取字节
	TX=SI24R1_R_RX_PAYLOAD;
	SPI_CSN(0);	
	HAL_SPI_TransmitReceive(&hspi1,&TX,&RX,1,HAL_MAX_DELAY);
	HAL_SPI_TransmitReceive(&hspi1,TX_NULL,RX_Data,RX_Len,HAL_MAX_DELAY);
	SPI_CSN(1);	

	
	//写ACK BAG
	uint8_t TX_len=DATA->TXLen;

	
	if(TX_len)
	{
	uint8_t* TX_Data=DATA->TXData;
	uint8_t RX_NULL[TX_len];
	TX=SI24R1_W_ACK_PAYLOAD_BASE;
	SPI_CSN(0);		
	HAL_SPI_TransmitReceive(&hspi1,&TX,&RX,1,HAL_MAX_DELAY);
	HAL_SPI_TransmitReceive(&hspi1,TX_Data,RX_NULL,TX_len,HAL_MAX_DELAY);
	SPI_CSN(1);
	
	};
	
};
	
	
	DATA->RXLen=RX_Len;
	DATA->STATUS=STATUS;
	
	
	return HAL_OK;
	

}

uint8_t FREQ2_4G_SendClc(SendClc_Data *Data){

uint8_t	RX[100];
	
	
	Send_Data DATA;
	DATA.RXData=RX;
	DATA.TXData=Data->TXData;
	DATA.TXLen=Data->TXLen;
	
	
	FREQ2_4G_SendBag(&DATA);

if(DATA.STATUS&0x10){//清除已满标志位
uint8_t TXX,RXX;
	
	
SPI_CSN(0);
TXX=SI24R1_WriteRegister(SI24R1_STATUS);
HAL_SPI_TransmitReceive(&hspi1,&TXX,&RXX,1,HAL_MAX_DELAY);
TXX=(DATA.STATUS|0x10);
HAL_SPI_TransmitReceive(&hspi1,&TXX,&RXX,1,HAL_MAX_DELAY);
SPI_CSN(1);


}else{
	if((DATA.STATUS&0x60)==0x60){
	for(int i=0;i<DATA.RXLen;i++){

	*((Data->RXData)+i)=*(RX+i);

	}

};
Data->RXLen=DATA.RXLen;
};



return HAL_OK;
}



uint8_t FREQ2_4G_ReadClc(ReadClc_Data *Data){


uint8_t RX_BUFF[100];
 Read_Data DATA;


DATA.RXData=RX_BUFF;
DATA.TXData=Data->TXData ;
DATA.TXLen=Data->TXLen;



FREQ2_4G_ReadBag(&DATA);

	if((DATA.RXLen<32)&&(DATA.RXLen!=0)){
	if((DATA.STATUS&0x0E)==0x00){//P0接到数据
	
	Data->RX_PIN=0;
		
	for(int i=0;i<DATA.RXLen;i++){
	
		*((Data->RXData_P0)+i)=*(RX_BUFF+i);
	
	};
	
	
	}else if((DATA.STATUS&0x0E)==0x02){//P1接到数据
	
	Data->RX_PIN=1;
	
		for(int i=0;i<DATA.RXLen;i++){
	
		*((Data->RXData_P1)+i)=*(RX_BUFF+i);
	
	};

	}else if((DATA.STATUS&0x0E)==0x04){//P2接到数据
	
	Data->RX_PIN=2;
	
		for(int i=0;i<DATA.RXLen;i++){
	
		*((Data->RXData_P2)+i)=*(RX_BUFF+i);
	
	};
	
	}else if((DATA.STATUS&0x0E)==0x06){//P3接到数据
	
	Data->RX_PIN=3;
		for(int i=0;i<DATA.RXLen;i++){
	
		*((Data->RXData_P3)+i)=*(RX_BUFF+i);
	
	};
	
	
	}else if((DATA.STATUS&0x0E)==0x08){//P4接到数据
	
	Data->RX_PIN=4;
	
		for(int i=0;i<DATA.RXLen;i++){
	
		*((Data->RXData_P4)+i)=*(RX_BUFF+i);
	
	};


	
		
		

	
	
	
	}else if((DATA.STATUS&0x0E)==0x0A){////P5接到数据
	
	Data->RX_PIN=5;
	
		for(int i=0;i<DATA.RXLen;i++){
	
		*((Data->RXData_P5)+i)=*(RX_BUFF+i);
	
	};
	
	};
};
	
	Data->RXLen=DATA.RXLen;
	
	

if((DATA.STATUS&0x0E)==0x0E||(DATA.STATUS&0x0E)==0x0C){//未收到包或错误处理

return HAL_ERROR;
	
}else{
return HAL_OK;
}
}



void FREQ2_4G_GetSignal(uint8_t* VAL){//信号强度
	uint8_t TX;
	uint8_t RX;
	HAL_GPIO_WritePin(SPI_CE_GPIO_Port,SPI_CE_Pin,GPIO_PIN_RESET);
SPI_CSN(0);
TX=SI24R1_ReadRegister(SI24R1_RSSI);
HAL_SPI_TransmitReceive(&hspi1,&TX,&RX,1,HAL_MAX_DELAY);
TX=0x00;
HAL_SPI_TransmitReceive(&hspi1,&TX,&RX,1,HAL_MAX_DELAY);
SPI_CSN(1);

if(RX){*VAL=1;}	else{*VAL=0;};
	HAL_GPIO_WritePin(SPI_CE_GPIO_Port,SPI_CE_Pin,GPIO_PIN_SET);
	
}

void FREQ2_4G_GetlossPack (uint8_t* VAL){//丢包状态
	uint8_t TX;
	uint8_t RX;
	HAL_GPIO_WritePin(SPI_CE_GPIO_Port,SPI_CE_Pin,GPIO_PIN_RESET);
SPI_CSN(0);
TX=SI24R1_ReadRegister(SI24R1_OBSERVE_TX);
HAL_SPI_TransmitReceive(&hspi1,&TX,&RX,1,HAL_MAX_DELAY);
TX=0x00;
HAL_SPI_TransmitReceive(&hspi1,&TX,&RX,1,HAL_MAX_DELAY);
SPI_CSN(1);

*VAL=RX;
	HAL_GPIO_WritePin(SPI_CE_GPIO_Port,SPI_CE_Pin,GPIO_PIN_SET);
	
}

