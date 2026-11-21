#include "MYdef.h"
#include "controller_config.h"
#include "controller_mode.h"
#include "handset_link.h"

#define CONTROL_INPUT_TIMEOUT_TICKS pdMS_TO_TICKS(CONTROLLER_INPUT_TIMEOUT_MS)

typedef enum
{
	CONTROL_SAFE_STOP_SPEED,
	CONTROL_SAFE_STOP_RATE
} control_safe_stop_mode_t;


float GYRO_X=0;
float GYRO_Y=0;

float Euler_P_standard=0,Euler_R_standard=0,Euler_Y_standard=0,H_standard=0;



volatile uint8_t Key_Lock=0;

volatile uint8_t OLED_FeatureClearPending = 0U;
static uint8_t OLED_CtrlFeatureVisible = 0U;

/* SP/RA控制算法仍生成原2.4G命令；显示遥测值同时入快照，由LEFT侧链路同步给RIGHT。 */
static void ShowFeatureValue(uint8_t index, uint8_t line, uint8_t column,
	float value, uint8_t length)
{
	OLED_ShowSmoothSignedNum(line, column, value, length);
	TASK_PublishFeatureValue(index, value);
}

/* 非阻塞读取最新快照；所需传感器无效或超过100ms时失败，由控制分支停速度。 */
static uint8_t GetFreshInputSnapshot(controller_input_snapshot_t *snapshot,
	uint8_t require_joystick, uint8_t require_gyro)
{
	TickType_t now;

	if (TASK_GetInputSnapshot(snapshot) == 0U)
	{
		return 0U;
	}
	now = xTaskGetTickCount();

	if (require_joystick != 0U &&
		(snapshot->joystick_valid == 0U ||
		(TickType_t)(now - (TickType_t)snapshot->joystick_sample_tick) >=
		CONTROL_INPUT_TIMEOUT_TICKS))
	{
		return 0U;
	}

	if (require_gyro != 0U &&
		(snapshot->gyro_valid == 0U ||
		(TickType_t)(now - (TickType_t)snapshot->gyro_sample_tick) >=
		CONTROL_INPUT_TIMEOUT_TICKS))
	{
		return 0U;
	}

	return 1U;
}

static void DisplayJoystickSnapshot(const controller_input_snapshot_t *snapshot)
{
	if (snapshot->joystick_valid != 0U)
	{
		ShowFeatureValue(0U, 2U, 7U, snapshot->joystick_x_percent, 4U);
		ShowFeatureValue(1U, 3U, 7U, snapshot->joystick_y_percent, 4U);
	}
}

static void DisplayGyroSnapshot(const controller_input_snapshot_t *snapshot)
{
	if (snapshot->gyro_valid != 0U)
	{
		ShowFeatureValue(0U, 2U, 7U, snapshot->gyro_roll_deg, 4U);
		ShowFeatureValue(1U, 3U, 7U, snapshot->gyro_pitch_deg, 4U);
	}
}



void OLED_Ctrl(void)
{
	task_menu_snapshot_t menuState;
	uint8_t featureVisible;
	uint8_t mwGyroState = 0U;

	TASK_GetMenuSnapshot(&menuState);
	/* 记录功能页是否可见；离开时请求清理遥测值，避免旧页面数据残留。 */
	featureVisible = ((menuState.DoubleModle == 0U ||
		menuState.ConnectionState == CONTROLLER_LINK_STATE_DUAL) &&
		menuState.FloorSwitch == 2 &&
		(((menuState.Floor1 == 0 || menuState.Floor1 == 1) &&
		  (menuState.Floor2 == 0 || menuState.Floor2 == 3)) ||
		 (menuState.Floor1 == 2 && menuState.Floor2 == 0))) ? 1U : 0U;
	if (OLED_CtrlFeatureVisible != 0U && featureVisible == 0U)
	{
		OLED_FeatureClearPending = 1U;
	}
	else if (featureVisible != 0U)
	{
		OLED_FeatureClearPending = 0U;
	}
	OLED_CtrlFeatureVisible = featureVisible;
	/* 把当前菜单变量交给UI状态机；这里不直接写OLED。 */
	if (menuState.ConnectionState == CONTROLLER_LINK_STATE_DUAL &&
		menuState.Floor1 == 2 && menuState.Floor2 == 0 && menuState.FloorSwitch == 2)
	{
		mwGyroState = (uint8_t)(menuState.Floor4 + 1);
	}
	OLED_RenderControllerUI((uint8_t)menuState.Floor1, (uint8_t)menuState.Floor2,
		(uint8_t)menuState.FloorSwitch, (uint8_t)menuState.Floor3,
		menuState.ConnectionState, ControllerMode_GetLocalRole(),
		mwGyroState);
	HandsetLink_NotifyUIChanged();
}

/* 超时只把对应速度字段清零；RA目标字段保留，避免把目标误清成零。 */
static void QueueControlSafeStop(control_safe_stop_mode_t mode)
{
	if (mode == CONTROL_SAFE_STOP_SPEED)
	{
		TX_Data.Data_1 = 0U;
		TX_Data.Data_2 = 0U;
	}
	else
	{
		/* RA字段携带目标值和速度，输入超时只停速度并保留最后目标。 */
		TX_Data.Data_6 = 0U;
		TX_Data.Data_10 = 0U;
	}

	xQueueOverwrite(FREQ2_4G_DataQueueHandle, &TX_Data);
}


/* SP摇杆模式：摇杆样本无效或过期时排安全停止命令，不继续使用旧输入。 */
void SP_JS_ON (void){

				int16_t X_BUFF,Y_BUFF;
				uint8_t turn_flag;
				int16_t Lratio,Rratio;
				int16_t Lspeed,Rspeed;				
				controller_input_snapshot_t inputSnapshot;
				/*偏移校准*/
				int8_t JS_X_Offset=0,JS_Y_Offset=-2;				
				
				/*死区生成*/
				
				if (GetFreshInputSnapshot(&inputSnapshot, 1U, 0U) == 0U)
				{
					QueueControlSafeStop(CONTROL_SAFE_STOP_SPEED);
					return;
				}
					if(inputSnapshot.joystick_x_percent>(5+JS_X_Offset)||inputSnapshot.joystick_x_percent<(-5+JS_X_Offset))X_BUFF=inputSnapshot.joystick_x_percent;//死区
						else X_BUFF=0;
								
					if(inputSnapshot.joystick_y_percent>(5+JS_Y_Offset)||inputSnapshot.joystick_y_percent<(-5+JS_Y_Offset))Y_BUFF=inputSnapshot.joystick_y_percent;//死区
						else Y_BUFF=0;
	
				
				/*数据解析*/
					if(X_BUFF>0)turn_flag=1;
					else if(X_BUFF<0)turn_flag=2;
					else if(X_BUFF==0)turn_flag=0;	
				
					switch (turn_flag){
						case 0:
							Lratio=100;
							Rratio=100;
							break;
						case 1:
							
						Lratio=100;
						Rratio=100-X_BUFF;
							break;
						
						case 2:
						Rratio=100;
						Lratio=100+X_BUFF;
						
							break;
					
					
					};
	
					Lspeed=(Lratio/99.0)*Y_BUFF;
					Rspeed=(Rratio/99.0)*Y_BUFF;
				
				
				/*原地旋转分支*/
					if(Y_BUFF==0){
						switch (turn_flag){
						
							case 0:
								
								break;
							case 1:
							Lspeed=X_BUFF;
							Rspeed=-X_BUFF;
								break;
							
							case 2:
							Lspeed=X_BUFF;
							Rspeed=-X_BUFF;
							
								break;
						
						};
						Lspeed=((float)Lspeed/100*50.0);
						Rspeed=((float)Rspeed/100*50.0);
						
						};


				
				/*数据发送*/
						
						
						
						
					// 数据调整
					TX_Data.Data_1=(int8_t)(((float)Lspeed)/100*100.0);
					TX_Data.Data_2=(int8_t)(((float)Rspeed)/100*100.0);	
						
				xQueueOverwrite(FREQ2_4G_DataQueueHandle, &TX_Data);
						
						
										
				DisplayJoystickSnapshot(&inputSnapshot);
		

}



void SP_JS_OFF (void){

				controller_input_snapshot_t inputSnapshot;

					TX_Data.Data_1=0;
					TX_Data.Data_2=0;	
				xQueueOverwrite(FREQ2_4G_DataQueueHandle, &TX_Data);
						
						
										
				if (TASK_GetInputSnapshot(&inputSnapshot) != 0U)
				{
					DisplayJoystickSnapshot(&inputSnapshot);
				}













}












static int8_t MW_ConvertAxisToCommand(float value, float offset)
{
	if (value > (float)CONTROLLER_MW_SPIN_LIMIT_PERCENT)
		value = (float)CONTROLLER_MW_SPIN_LIMIT_PERCENT;
	else if (value < -(float)CONTROLLER_MW_SPIN_LIMIT_PERCENT)
		value = -(float)CONTROLLER_MW_SPIN_LIMIT_PERCENT;

	if (value > ((float)CONTROLLER_MW_SPIN_DEADZONE_PERCENT + offset) ||
		value < (-(float)CONTROLLER_MW_SPIN_DEADZONE_PERCENT + offset))
	{
		return (int8_t)value;
	}
	return 0;
}

/* RIGHT显示复用LEFT发包的死区/限幅换算，保证Z值与协议字段一致。 */
void MW_UpdateAngularJoystickDisplay(float joystick_x_percent)
{
	task_menu_snapshot_t menuState;

	if (CONTROLLER_LOCAL_ROLE != CONTROLLER_ROLE_RIGHT)
	{
		return;
	}
	TASK_GetMenuSnapshot(&menuState);
	if (menuState.ConnectionState == CONTROLLER_LINK_STATE_DUAL &&
		menuState.Floor1 == 2 && menuState.Floor2 == 0 &&
		menuState.FloorSwitch == 2 &&
		(menuState.Floor4 == (int8_t)CONTROLLER_MW_SPIN_STATE_LIVE ||
		 menuState.Floor4 == (int8_t)CONTROLLER_MW_SPIN_STATE_SETUP))
	{
		/* LIVE显示当前角速度，SETUP显示待锁定预览；锁定态不追随摇杆。 */
		OLED_UpdateMwAngularValue(
			(int32_t)MW_ConvertAxisToCommand(joystick_x_percent, 0.0f));
	}
}

/* MW/JS：单右手柄单机仍用X/Y平移；双手柄时输入上传给LEFT合成自转字段。 */
void MW_JS_ON (void)
{
	controller_input_snapshot_t inputSnapshot;
	const float JS_X_Offset = 0.0f;
	const float JS_Y_Offset = -2.0f;

	TX_Data.Data_10 = 0U;
	if (GetFreshInputSnapshot(&inputSnapshot, 1U, 0U) == 0U)
	{
		QueueControlSafeStop(CONTROL_SAFE_STOP_SPEED);
		return;
	}

	TX_Data.Data_1 = (uint8_t)MW_ConvertAxisToCommand(
		inputSnapshot.joystick_x_percent, JS_X_Offset);
	TX_Data.Data_2 = (uint8_t)MW_ConvertAxisToCommand(
		inputSnapshot.joystick_y_percent, JS_Y_Offset);
	xQueueOverwrite(FREQ2_4G_DataQueueHandle, &TX_Data);
	DisplayJoystickSnapshot(&inputSnapshot);
}

/* MW/JS关闭时清零x/y/z三个字段，再覆盖无线单槽队列。 */
void MW_JS_OFF (void)
{
	controller_input_snapshot_t inputSnapshot;

	TX_Data.Data_1 = 0U;
	TX_Data.Data_2 = 0U;
	TX_Data.Data_10 = 0U;
	xQueueOverwrite(FREQ2_4G_DataQueueHandle, &TX_Data);
	if (TASK_GetInputSnapshot(&inputSnapshot) != 0U)
	{
		DisplayJoystickSnapshot(&inputSnapshot);
	}
}

void SP_GYRO_ON (void){



				int16_t X_BUFF,Y_BUFF;
				uint8_t turn_flag;
				int16_t Lratio,Rratio;
				int16_t Lspeed,Rspeed;	
				uint8_t LOCK_flag=0;	
				controller_input_snapshot_t inputSnapshot;
				/*偏移校准*/
				int8_t GYRO_P_Offset=0,GYRO_R_Offset=5;
	
	float R_BUFF,P_BUFF;
			
	
			/*死区生成*/			
				if (GetFreshInputSnapshot(&inputSnapshot, 0U, 1U) == 0U)
				{
					QueueControlSafeStop(CONTROL_SAFE_STOP_SPEED);
					return;
				}
				
				R_BUFF=inputSnapshot.gyro_roll_deg-Euler_R_standard;
				P_BUFF=inputSnapshot.gyro_pitch_deg-Euler_P_standard;
	
				if(P_BUFF>(5+GYRO_P_Offset)||P_BUFF<(-5+GYRO_P_Offset))X_BUFF=P_BUFF;//死区
				else X_BUFF=0;
				
				if(R_BUFF>(5+GYRO_R_Offset)||R_BUFF<(-5+GYRO_R_Offset))Y_BUFF=-R_BUFF*2;//死区
				else Y_BUFF=0;

				
					/*数据解析*/
				if(X_BUFF>0)turn_flag=1;
				else if(X_BUFF<0)turn_flag=2;
				else if(X_BUFF==0)turn_flag=0;
				
				switch (turn_flag){
					case 0:
						Lratio=100;
						Rratio=100;
						break;
					case 1:
						
					Lratio=100;
					Rratio=100-X_BUFF;
						break;
					
					case 2:
					Rratio=100;
					Lratio=100+X_BUFF;
					
						break;
				
				
				};				
				
				Lspeed=(Lratio/99.0)*Y_BUFF;
				Rspeed=(Rratio/99.0)*Y_BUFF;
				
				if(Y_BUFF==0&&X_BUFF==0)LOCK_flag=0;

					if(Y_BUFF==0&&LOCK_flag==0){
							switch (turn_flag){
							
							case 0:
									
									break;
								case 1:
								Lspeed=X_BUFF;
								Rspeed=-X_BUFF;
									break;
								
								case 2:
								Lspeed=X_BUFF;
								Rspeed=-X_BUFF;
								
									break;
							
							};
					


					}else {
					LOCK_flag=1;
					
					
					};	

				
				
				
			
				ShowFeatureValue(0U, 2U, 7U, R_BUFF, 4U);
				ShowFeatureValue(1U, 3U, 7U, P_BUFF, 4U);
					
					//限位
					if(Lspeed>100)Lspeed=100;
					else if(Lspeed<-100)Lspeed=-100;
					
					if(Rspeed>100)Rspeed=100;
					else if(Rspeed<-100)Rspeed=-100;					
					
				
					
					
					
			TX_Data.Data_1=(uint8_t)Lspeed;
			TX_Data.Data_2=(uint8_t)Rspeed;		
			
			
			
			
			
			/*数据发送*/
	xQueueOverwrite(FREQ2_4G_DataQueueHandle, &TX_Data);








}

void SP_GYRO_OFF(void){

	controller_input_snapshot_t inputSnapshot;

	if (GetFreshInputSnapshot(&inputSnapshot, 0U, 1U) == 0U)
	{
		QueueControlSafeStop(CONTROL_SAFE_STOP_SPEED);
		return;
	}
	Euler_P_standard=inputSnapshot.gyro_pitch_deg;
	Euler_R_standard=inputSnapshot.gyro_roll_deg;

			TX_Data.Data_1=0;
			TX_Data.Data_2=0;		
			

			/*数据发送*/
	xQueueOverwrite(FREQ2_4G_DataQueueHandle, &TX_Data);

	
	

				DisplayGyroSnapshot(&inputSnapshot);




}






/* RA摇杆模式带积分目标；控制周期快于采样周期，同一序号只积分一次。 */
void RA_JS_ON (void){



				int16_t X_BUFF,Y_BUFF;
				static uint16_t hight=0;
				static int16_t angle=0;
				static uint32_t lastProcessedJoystickTick;
				static uint8_t processedJoystickSample=0U;
				controller_input_snapshot_t inputSnapshot;
				uint8_t hight_Speed;
				int8_t	angle_Speed;
				float hight_COEF=0.5,angle_COEF=2;
	
				/*偏移校准*/
				int8_t JS_X_Offset=0,JS_Y_Offset=-2;				
				
				/*死区生成*/
				
				if (GetFreshInputSnapshot(&inputSnapshot, 1U, 0U) == 0U)
				{
					QueueControlSafeStop(CONTROL_SAFE_STOP_RATE);
					return;
				}
				/* 控制周期快于摇杆发布周期，积分型RA输入只处理每个新样本一次。 */
				if (processedJoystickSample != 0U &&
					inputSnapshot.joystick_sample_tick == lastProcessedJoystickTick)
				{
					return;
				}
				lastProcessedJoystickTick = inputSnapshot.joystick_sample_tick;
				processedJoystickSample = 1U;

					if(inputSnapshot.joystick_x_percent>(5+JS_X_Offset)||inputSnapshot.joystick_x_percent<(-5+JS_X_Offset))X_BUFF=inputSnapshot.joystick_x_percent;//死区
						else X_BUFF=0;
								
					if(inputSnapshot.joystick_y_percent>(5+JS_Y_Offset)||inputSnapshot.joystick_y_percent<(-5+JS_Y_Offset))Y_BUFF=inputSnapshot.joystick_y_percent;//死区
						else Y_BUFF=0;
	
				
				/*数据解析*/
	/*高度目标值，高度速度*//*旋转目标值，旋转速度*//*各3bit*/
				/*精确到小数点后两位*/
				
				/*累加高度和角度*/
				hight+=Y_BUFF*hight_COEF;
				angle+=X_BUFF*angle_COEF;
				/*高度和角度移动速度*/
				
				hight_Speed=abs(Y_BUFF);
				angle_Speed=X_BUFF;
				
				
				
				/*限幅*/
				if(hight>12800)hight=12800;//最高128mm
				else if((int16_t)hight<0)hight=0;
				/*环状循环*/
				if(angle>18000)angle=-18000;
				else if(angle<-18000)angle=18000;
				
				
				/*数据发送*/
				
				TX_Data.Data_4=(hight>>8);//目标高度高8位
				TX_Data.Data_5=(hight&0xff);//目标高度低8位
				TX_Data.Data_6=hight_Speed;//速度
				
				TX_Data.Data_8=(angle>>8);//目标角度高8位
				TX_Data.Data_9=(angle&0xff);//目标角度低8位
				TX_Data.Data_10=angle_Speed;//速度
				
				ShowFeatureValue(0U, 2U, 7U, hight / 100.0f, 4U);
				ShowFeatureValue(1U, 3U, 7U, angle / 100.0f, 4U);
				
				
					xQueueOverwrite(FREQ2_4G_DataQueueHandle, &TX_Data);

}


void RA_JS_OFF (void){

	uint16_t hight;
	int16_t angle;


					TX_Data.Data_6=0;
					TX_Data.Data_10=0;	
					xQueueOverwrite(FREQ2_4G_DataQueueHandle, &TX_Data);
						
						hight=(TX_Data.Data_4<<8);
						hight=hight|TX_Data.Data_5;
	
						angle=(TX_Data.Data_8<<8);
	                    angle=angle|TX_Data.Data_9;
	
										
				ShowFeatureValue(0U, 2U, 7U, hight / 100.0f, 4U);
				ShowFeatureValue(1U, 3U, 7U, angle / 100.0f, 4U);











}






/* RA姿态模式只对新姿态样本推进目标，避免重复处理上一帧造成积分漂移。 */
void RA_GYRO_ON (void){
				 int16_t X_BUFF,Y_BUFF,Z_BUFF;
				volatile static float hight_BUFF;
				volatile uint16_t hight=0;
				int16_t angle=0;
				static int32_t	angle_BUFF=0;
				static uint32_t lastProcessedGyroTick;
				static uint8_t processedGyroSample=0U;
				controller_input_snapshot_t inputSnapshot;
				uint8_t hight_Speed;
				int16_t	angle_Speed;
				static uint16_t Last_hight;
				static	int16_t Last_angle;
	
				/*偏移校准*/
				int8_t GYRO_P_Offset=0,GYRO_R_Offset=5,GYRO_Y_Offset=0;
	
				float R_BUFF,P_BUFF,Y__BUFF;
	
			/*死区生成*/			
				if (GetFreshInputSnapshot(&inputSnapshot, 0U, 1U) == 0U)
				{
					QueueControlSafeStop(CONTROL_SAFE_STOP_RATE);
					return;
				}
				/* 使用最新快照而非队列等待；积分状态只随新姿态样本推进。 */
				if (processedGyroSample != 0U &&
					inputSnapshot.gyro_sample_tick == lastProcessedGyroTick)
				{
					return;
				}
				lastProcessedGyroTick = inputSnapshot.gyro_sample_tick;
				processedGyroSample = 1U;
				
				R_BUFF=inputSnapshot.gyro_roll_deg-Euler_R_standard;
				P_BUFF=inputSnapshot.gyro_pitch_deg-Euler_P_standard;
				Y__BUFF=inputSnapshot.gyro_yaw_deg-Euler_Y_standard;

	
					//界限限制
				if (Y__BUFF > 180.0f) {
				Y__BUFF -= 360.0f;  
			} 
				else if (Y__BUFF < -180.0f) {
				Y__BUFF += 360.0f; 
			};

			
				if(P_BUFF>(10+GYRO_P_Offset)||P_BUFF<(-10+GYRO_P_Offset))X_BUFF=P_BUFF;//死区
				else X_BUFF=0;
				
				if(R_BUFF>(5+GYRO_R_Offset)||R_BUFF<(-5+GYRO_R_Offset))Y_BUFF=R_BUFF;//死区
				else Y_BUFF=0;

				if(Y__BUFF>(5+GYRO_Y_Offset)||Y__BUFF<(-5+GYRO_Y_Offset))Z_BUFF=Y__BUFF;//死区
				else Z_BUFF=0;	
								
					/*数据解析*/

				
				

				hight_BUFF+=Y_BUFF/2.0;		
				//+Z_BUFF*100
				angle_BUFF+=X_BUFF/2.0;
				
				/*限幅*/
			if (hight_BUFF > 12800.0f) {
				hight_BUFF = 12800.0f;  
			} 
				else if ((int16_t)hight_BUFF < 0) {
				hight_BUFF = 0; 
			};	
			
//				if(angle_BUFF>18000)angle_BUFF=-18000;
//				else if(angle_BUFF<-18000)angle_BUFF=18000;			
			
				hight=hight_BUFF;
				angle=angle_BUFF+Z_BUFF*100;
				
			
			
							if (angle > 18000.0f) {
				angle -= 36000.0f;  
			} 
				else if (angle < -18000.0f) {
				angle += 36000.0f; 
			};	
				
			
			
			
				hight_Speed=abs((int)(Last_hight-hight));
				angle_Speed=(angle-Last_angle)*2;
				
				
				

				 
				/*环状循环*//*有问题*/

//				if(angle>18000)angle=-18000;
//				else if(angle<-18000)angle=18000;
			
	
			
				
				//界限限制
				if (angle_Speed > 180.0f) {
				angle_Speed -= 360.0f;  
			} 
				else if (angle_Speed < -180.0f) {
				angle_Speed += 360.0f; 
			};	

			if(angle_Speed>100){
	
			angle_Speed=100;
			
				}else if(angle_Speed<-100){
				
				
				angle_Speed=-100;
				
				
				};

			
							
				/*数据发送*/
				
				TX_Data.Data_4=(hight>>8);//目标高度高8位
				TX_Data.Data_5=(hight&0xff);//目标高度低8位
				TX_Data.Data_6=hight_Speed;//速度
				
				TX_Data.Data_8=(angle>>8);//目标角度高8位
				TX_Data.Data_9=(angle&0xff);//目标角度低8位
				TX_Data.Data_10=(int8_t)angle_Speed;//速度
				
//				OLED_ShowSmoothSignedNum(2,7,Y_BUFF,4);
				ShowFeatureValue(0U, 2U, 7U, hight / 100.0f, 4U);
				ShowFeatureValue(1U, 3U, 7U, angle / 100.0f, 4U);
				
				
				xQueueOverwrite(FREQ2_4G_DataQueueHandle, &TX_Data);


				Last_hight=hight;	
				Last_angle=angle;





}


void RA_GYRO_OFF (void){

	uint16_t hight;
	int16_t angle;
	controller_input_snapshot_t inputSnapshot;
	if (GetFreshInputSnapshot(&inputSnapshot, 0U, 1U) == 0U)
	{
		QueueControlSafeStop(CONTROL_SAFE_STOP_RATE);
		return;
	}
	Euler_P_standard=inputSnapshot.gyro_pitch_deg;
	Euler_R_standard=inputSnapshot.gyro_roll_deg;
	Euler_Y_standard=inputSnapshot.gyro_yaw_deg;
	H_standard=inputSnapshot.gyro_height_m;

					TX_Data.Data_6=0;
					TX_Data.Data_10=0;	
					xQueueOverwrite(FREQ2_4G_DataQueueHandle, &TX_Data);
						
						hight=(TX_Data.Data_4<<8);
						hight=hight|TX_Data.Data_5;
	
						angle=(TX_Data.Data_8<<8);
	                    angle=angle|TX_Data.Data_9;
	
										
				ShowFeatureValue(0U, 2U, 7U, hight / 100.0f, 4U);
				ShowFeatureValue(1U, 3U, 7U, angle / 100.0f, 4U);





}




 void KEY_LOCKTimerFun( TimerHandle_t xTimer ){

	 Key_Lock=0;
 
 }


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

  /* Prevent unused argument(s) compilation warning */
	if(GPIO_Pin==Key_MergeInput_Pin){
	
	
	xEventGroupSetBitsFromISR(KEY_StartEventGroupHandle,(1 << 0),NULL);
	
	
	
	
	}else if(GPIO_Pin==Key_U_Pin){
	
	
	xEventGroupSetBitsFromISR(KEY_StartEventGroupHandle,(1 << 0),NULL);
	
	
	
	
	}else if(GPIO_Pin==Key_L_Pin){
	
	
	xEventGroupSetBitsFromISR(KEY_StartEventGroupHandle,(1 << 0),NULL);
	
	
	
	
	}else if(GPIO_Pin==Key_D_Pin){
	
	
	
	xEventGroupSetBitsFromISR(KEY_StartEventGroupHandle,(1 << 0),NULL);
	
	
	
	}else if(GPIO_Pin==Key_R_Pin){
	
	
	
	xEventGroupSetBitsFromISR(KEY_StartEventGroupHandle,(1 << 0),NULL);
	
	
	
	};
	

  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_GPIO_EXTI_Callback could be implemented in the user file
   */
}








