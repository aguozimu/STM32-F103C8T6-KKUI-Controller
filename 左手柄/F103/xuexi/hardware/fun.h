#ifndef __FUN_H
#define __FUN_H


void OLED_Ctrl (void);

void SP_JS_ON (void);
void SP_JS_OFF (void);

void SP_GYRO_ON (void);
void SP_GYRO_OFF(void);

void RA_JS_ON (void);
void RA_JS_OFF (void);

void RA_GYRO_ON (void);
void RA_GYRO_OFF (void);

/* 麦轮控制：本机摇杆X/Y负责平移；合体后右侧四键辅助平移，PA8使能右摇杆X输出Z。 */
void MW_JS_ON (void);
void MW_JS_OFF (void);

void KEY_LOCKTimerFun( TimerHandle_t xTimer );

extern float GYRO_X;
extern float GYRO_Y;

extern float Euler_P_standard,Euler_R_standard;
extern volatile uint8_t Key_Lock;
extern volatile uint8_t OLED_FeatureClearPending;
#endif

