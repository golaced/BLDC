///////////////////////////////////////////////////////////////////////////////

//#pragma once
#include "stm32f10x.h"
///////////////////////////////////////////////////////////////////////////////

extern uint8_t rcActive;

///////////////////////////////////////////////////////////////////////////////

void rcInit(void);

///////////////////////////////////////////////////////////////////////////////

uint16_t rxRead(uint8_t channel);

///////////////////////////////////////////////////////////////////////////////

void TIM8_Cap_Init(u16 arr,u16 psc);
extern u8  TIM2CH1_CAPTURE_STA[4];	//输入捕获状态		    				
extern u16	TIM2CH1_CAPTURE_VAL[4];	//输入捕获值
extern float attitude[3];
extern float exp_angle[3];
#define ABS(x) ( (x)>0?(x):-(x) )
#define LIMIT( x,min,max ) ( (x) < (min)  ? (min) : ( (x) > (max) ? (max) : (x) ) )
#define _MIN(a, b) ((a) < (b) ? (a) : (b))
#define _MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))