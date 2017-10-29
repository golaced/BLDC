///////////////////////////////////////////////////////////////////////////////

#include "board.h"

///////////////////////////////////////////////////////////////////////////////

static uint16_t rc3pulseWidth = 3000;

static uint16_t rc4pulseWidth = 3000;

static uint16_t rc5pulseWidth = 3000;;

uint8_t rcActive = false;

///////////////////////////////////////////////////////////////////////////////

void rcInit(void)
{
    GPIO_InitTypeDef         GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseInitStructure;
		EXTI_InitTypeDef EXTI_InitStructure;
    ///////////////////////////////////

    __disable_irq_nested();

    //Luke Liu GPIO_PinRemapConfig(GPIO_Remap_SWJ_NoJTRST,     ENABLE);
    //Luke Liu GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);               // JTAG-DP Disabled and SW-DP Enabled

    //EXTI IN GPIO Config

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;  // PB3-Pitch, PB4-Roll, PB5-Yaw
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPD;                         // Set to Input Pull Down
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                      // GPIO Speed

    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource3);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource4);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource5);

    

    EXTI_InitStructure.EXTI_Line    = EXTI_Line3 | EXTI_Line4 | EXTI_Line5;
    EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    EXTI_ClearITPendingBit(EXTI_Line3 | EXTI_Line4 | EXTI_Line5);

    NVIC_EnableIRQ(EXTI3_IRQn);   // Enable interrupt
    NVIC_EnableIRQ(EXTI4_IRQn);   // Enable interrupt
    NVIC_EnableIRQ(EXTI9_5_IRQn); // Enable interrupt

    ///////////////////////////////////

    TIM_TimeBaseInitStructure.TIM_Prescaler     = 36 - 1; // 2 MHz
    TIM_TimeBaseInitStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period        = 0xFFFF;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;

    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);

    TIM_Cmd(TIM3, ENABLE);

    __enable_irq_nested();
}

///////////////////////////////////////////////////////////////////////////////

void EXTI3_IRQHandler(void) //EXTernal interrupt routine PB3
{
    uint16_t diff;
    static uint16_t upTime3;

    EXTI->PR |= (1 << 3);  // Clear pending interrupt

    if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_3) == 1)
        upTime3 = TIM3->CNT;
    else
    {
        diff = TIM3->CNT - upTime3;

        if ((diff > 1800) && (diff < 4200))
            rc3pulseWidth = diff;
        else
            rc3pulseWidth = 3000;

        rcActive = true;
    }
}

///////////////////////////////////////////////////////////////////////////////

void EXTI4_IRQHandler(void) //EXTernal interrupt routine PB4
{
    uint16_t diff;
    static uint16_t upTime4;

    EXTI->PR |= (1 << 4);  // Clear pending interrupt

    if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_4) == 1)
        upTime4 = TIM3->CNT;
    else
    {
        diff = TIM3->CNT - upTime4;

        if ((diff > 1800) && (diff < 4200))
            rc4pulseWidth = diff;
        else
            rc4pulseWidth = 3000;

        rcActive = true;
    }
}

///////////////////////////////////////////////////////////////////////////////

void EXTI9_5_IRQHandler(void) //EXTernal interrupt routine PB5
{
    uint16_t diff;
    static uint16_t upTime5;

    EXTI->PR |= (1 << 5);  // clear pending interrupt

    if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5) == 1)
        upTime5 = TIM3->CNT;
    else
    {
        diff = TIM3->CNT - upTime5;

        if ((diff > 1800) && (diff < 4200))
            rc5pulseWidth = diff;
        else
            rc5pulseWidth = 3000;

        rcActive = true;
    }
}

///////////////////////////////////////////////////////////////////////////////
//�õ��������ԭʼ��������ͨ���ⲿ�ж������ȡ��ʱ��ֵ��ȡ
uint16_t rxRead(uint8_t channel)
{
    if (channel == 0)       // Roll
        return rc4pulseWidth;
    else if (channel == 1)  // Pitch
        return rc3pulseWidth;
    else if (channel == 2)  // Yaw
        return rc5pulseWidth;
    else
        return 3000;
}

/////////////////////////////////////ENCODER/////////////////////////

///////////////////////////////////////////////////////////////////////////////

TIM_ICInitTypeDef  TIM8_ICInitStructure;

void TIM8_Cap_Init(u16 arr,u16 psc)
{	 
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
 	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);	//ʹ��TIM2ʱ��
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);  //ʹ��GPIOAʱ��
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;  //PA0 ���֮ǰ����  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //PA0 ����  
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOC,GPIO_Pin_6);						 //PA0 ����
	GPIO_ResetBits(GPIOC,GPIO_Pin_7);						 //PA0 ����
	GPIO_ResetBits(GPIOC,GPIO_Pin_8);						 //PA0 ����
	GPIO_ResetBits(GPIOC,GPIO_Pin_9);						 //PA0 ����
	//��ʼ����ʱ��2 TIM2	 
	TIM_TimeBaseStructure.TIM_Period = arr; //�趨�������Զ���װֵ 
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 	//Ԥ��Ƶ��   
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
  
	//��ʼ��TIM2���벶�����
	TIM8_ICInitStructure.TIM_Channel = TIM_Channel_1; //CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
  TIM8_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
  TIM8_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
  TIM8_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //���������Ƶ,����Ƶ 
  TIM8_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 ���������˲��� ���˲�
  TIM_ICInit(TIM8, &TIM8_ICInitStructure);

	//��ʼ��TIM2���벶�����
	TIM8_ICInitStructure.TIM_Channel = TIM_Channel_2; //CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
  TIM8_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
  TIM8_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
  TIM8_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //���������Ƶ,����Ƶ 
  TIM8_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 ���������˲��� ���˲�
  TIM_ICInit(TIM8, &TIM8_ICInitStructure);
	//��ʼ��TIM2���벶�����
	TIM8_ICInitStructure.TIM_Channel = TIM_Channel_3; //CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
  TIM8_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
  TIM8_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
  TIM8_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //���������Ƶ,����Ƶ 
  TIM8_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 ���������˲��� ���˲�
  TIM_ICInit(TIM8, &TIM8_ICInitStructure);
	//��ʼ��TIM2���벶�����
	TIM8_ICInitStructure.TIM_Channel = TIM_Channel_4; //CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
  TIM8_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
  TIM8_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
  TIM8_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //���������Ƶ,����Ƶ 
  TIM8_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 ���������˲��� ���˲�
  TIM_ICInit(TIM8, &TIM8_ICInitStructure);
	//�жϷ����ʼ��
	NVIC_InitStructure.NVIC_IRQChannel = TIM8_CC_IRQn;  //TIM2�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  //��ռ���ȼ�2��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;  //�����ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ��� 
	
	TIM_ITConfig(TIM8,TIM_IT_Update|TIM_IT_CC1,ENABLE);//��������ж� ,����CC1IE�����ж�	
	TIM_ITConfig(TIM8,TIM_IT_Update|TIM_IT_CC2,ENABLE);//��������ж� ,����CC1IE�����ж�	
	TIM_ITConfig(TIM8,TIM_IT_Update|TIM_IT_CC3,ENABLE);//��������ж� ,����CC1IE�����ж�	
	TIM_ITConfig(TIM8,TIM_IT_Update|TIM_IT_CC4,ENABLE);//��������ж� ,����CC1IE�����ж�	
  TIM_Cmd(TIM8,ENABLE ); 	//ʹ�ܶ�ʱ��2
 
}
float attitude[3];
float rad[3];
float pos[3];
u8 state_dj[5]={0,0,0,0,0};
u8 state_dj_rx[5]={0,0,0,0,0};
u8 IO_STATE[5]={0,0,0,0,0};
u8 IO_STATER[5]={0,0,0,0,0};
u32 cnt_sample1,now_dj[4]={0},lastUpdate_dj[4]={0};
u8  TIM2CH1_CAPTURE_STA[4]={0};	//���벶��״̬		    				
u16	TIM2CH1_CAPTURE_VAL[4]={0};	//���벶��ֵ
#define MAX_ENCODER 917
//��ʱ��5�жϷ������	 
int max_encoder_cal=0;
float dt_encoder_update;
void TIM8_CC_IRQHandler(void)
{ 
	static float att_reg[4];
  u8 sel=ROLL;
	 __disable_irq_nested();
 	if((TIM2CH1_CAPTURE_STA[sel]&0X80)==0)//��δ�ɹ�����	
	{	  
		if (TIM_GetITStatus(TIM8, TIM_IT_Update) != RESET)
		 
		{	    
			if(TIM2CH1_CAPTURE_STA[sel]&0X40)//�Ѿ����񵽸ߵ�ƽ��
			{
				if((TIM2CH1_CAPTURE_STA[sel]&0X3F)==0X3F)//�ߵ�ƽ̫����
				{
					TIM2CH1_CAPTURE_STA[sel]|=0X80;//��ǳɹ�������һ��
					TIM2CH1_CAPTURE_VAL[sel]=0XFFFF;
				}else TIM2CH1_CAPTURE_STA[sel]++;
			}	
     	
		}
	if (TIM_GetITStatus(TIM8, TIM_IT_CC1) != RESET)//����1���������¼�
		{	
			if(TIM2CH1_CAPTURE_STA[sel]&0X40)		//����һ���½��� 		
			{	  			
				TIM2CH1_CAPTURE_STA[sel]|=0X80;		//��ǳɹ�����һ��������
				now_dj[sel] = micros();  //��ȡʱ��
				if( now_dj[sel] <lastUpdate_dj[sel]){cnt_sample1 =  ((float)( now_dj[sel]  + (0xffff- lastUpdate_dj[sel])) );}
				else	{ cnt_sample1 =  ((float)( now_dj[sel]  - lastUpdate_dj[sel])); }
				pos[sel]=cnt_sample1;
				if(pos[sel]>max_encoder_cal)
					max_encoder_cal=pos[sel];
				bldc.attitude[sel]=attitude[sel]=Moving_Median(4,5,LIMIT(pos[sel],0,MAX_ENCODER)/MAX_ENCODER*360);
				bldc.en_code_connect[sel]=1;
				bldc.encoder_loss[sel]=0;
		   	TIM_OC1PolarityConfig(TIM8,TIM_ICPolarity_Rising); //CC1P=0 ����Ϊ�����ز���
			}else  								//��δ��ʼ,��һ�β���������
			{
				TIM2CH1_CAPTURE_STA[sel]=0;			//���
				TIM2CH1_CAPTURE_VAL[sel]=0;
	 			lastUpdate_dj[sel] =  micros();
				TIM2CH1_CAPTURE_STA[sel]|=0X40;		//��ǲ�����������
		   	TIM_OC1PolarityConfig(TIM8,TIM_ICPolarity_Falling);		//CC1P=1 ����Ϊ�½��ز���
			}		 
		}			     	    					   
 	}
//
	sel=PITCH;
 	if((TIM2CH1_CAPTURE_STA[sel]&0X80)==0)//��δ�ɹ�����	
	{	  
		if (TIM_GetITStatus(TIM8, TIM_IT_Update) != RESET)
		 
		{	    
			if(TIM2CH1_CAPTURE_STA[sel]&0X40)//�Ѿ����񵽸ߵ�ƽ��
			{
				if((TIM2CH1_CAPTURE_STA[sel]&0X3F)==0X3F)//�ߵ�ƽ̫����
				{
					TIM2CH1_CAPTURE_STA[sel]|=0X80;//��ǳɹ�������һ��
					TIM2CH1_CAPTURE_VAL[sel]=0XFFFF;
				}else TIM2CH1_CAPTURE_STA[sel]++;
			}	
    	
		}
	if (TIM_GetITStatus(TIM8, TIM_IT_CC2) != RESET)//����1���������¼�
		{	
			if(TIM2CH1_CAPTURE_STA[sel]&0X40)		//����һ���½��� 		
			{	  			
				TIM2CH1_CAPTURE_STA[sel]|=0X80;		//��ǳɹ�����һ��������
				now_dj[sel] = micros();  //��ȡʱ��
				if( now_dj[sel] <lastUpdate_dj[sel]){cnt_sample1 =  ((float)( now_dj[sel]  + (0xffff- lastUpdate_dj[sel])) );}
				else	{ cnt_sample1 =  ((float)( now_dj[sel]  - lastUpdate_dj[sel])); }
				pos[sel]=cnt_sample1;
				if(pos[sel]>max_encoder_cal)
				max_encoder_cal=pos[sel];
				dt_encoder_update=(float)Get_Cycle_T(DT_ENCODER)/1000000.;
				bldc.attitude[sel]=attitude[sel]=Moving_Median(5,5,LIMIT(pos[sel],0,MAX_ENCODER)/MAX_ENCODER*360);
				//attitude[sel]=LIMIT(pos[sel],0,MAX_ENCODER)/MAX_ENCODER*360;
        rad[sel]=Moving_Median(1,5,(attitude[sel]-att_reg[sel]));
				att_reg[sel]=attitude[sel];
				bldc.en_code_connect[sel]=1;
				bldc.encoder_loss[sel]=0;
		   	TIM_OC2PolarityConfig(TIM8,TIM_ICPolarity_Rising); //CC1P=0 ����Ϊ�����ز���
			}else  								//��δ��ʼ,��һ�β���������
			{
				TIM2CH1_CAPTURE_STA[sel]=0;			//���
				TIM2CH1_CAPTURE_VAL[sel]=0;
	 			lastUpdate_dj[sel] =  micros();
				TIM2CH1_CAPTURE_STA[sel]|=0X40;		//��ǲ�����������
		   	TIM_OC2PolarityConfig(TIM8,TIM_ICPolarity_Falling);		//CC1P=1 ����Ϊ�½��ز���
			}		 
   
		}			     	    					   
 	}
//
	sel=YAW;
 	if((TIM2CH1_CAPTURE_STA[sel]&0X80)==0)//��δ�ɹ�����	
	{	  
		if (TIM_GetITStatus(TIM8, TIM_IT_Update) != RESET)
		 
		{	    
			if(TIM2CH1_CAPTURE_STA[sel]&0X40)//�Ѿ����񵽸ߵ�ƽ��
			{
				if((TIM2CH1_CAPTURE_STA[sel]&0X3F)==0X3F)//�ߵ�ƽ̫����
				{
					TIM2CH1_CAPTURE_STA[sel]|=0X80;//��ǳɹ�������һ��
					TIM2CH1_CAPTURE_VAL[sel]=0XFFFF;
				}else TIM2CH1_CAPTURE_STA[sel]++;
			}	
    	
		}
	if (TIM_GetITStatus(TIM8, TIM_IT_CC3) != RESET)//����1���������¼�
		{	
			if(TIM2CH1_CAPTURE_STA[sel]&0X40)		//����һ���½��� 		
			{	  			
				TIM2CH1_CAPTURE_STA[sel]|=0X80;		//��ǳɹ�����һ��������
				now_dj[sel] = micros();  //��ȡʱ��
				if( now_dj[sel] <lastUpdate_dj[sel]){cnt_sample1 =  ((float)( now_dj[sel]  + (0xffff- lastUpdate_dj[sel])) );}
				else	{ cnt_sample1 =  ((float)( now_dj[sel]  - lastUpdate_dj[sel])); }
				pos[sel]=cnt_sample1;
				if(pos[sel]>max_encoder_cal)
				max_encoder_cal=pos[sel];
				bldc.attitude[sel]=attitude[sel]=Moving_Median(6,5,LIMIT(pos[sel],0,MAX_ENCODER)/MAX_ENCODER*360);
				bldc.en_code_connect[sel]=1;
				bldc.encoder_loss[sel]=0;
		   	TIM_OC3PolarityConfig(TIM8,TIM_ICPolarity_Rising); //CC1P=0 ����Ϊ�����ز���
			}else  								//��δ��ʼ,��һ�β���������
			{
				TIM2CH1_CAPTURE_STA[sel]=0;			//���
				TIM2CH1_CAPTURE_VAL[sel]=0;
	 			lastUpdate_dj[sel] =  micros();
				TIM2CH1_CAPTURE_STA[sel]|=0X40;		//��ǲ�����������
		   	TIM_OC3PolarityConfig(TIM8,TIM_ICPolarity_Falling);		//CC1P=1 ����Ϊ�½��ز���
			}		 
   
		}			     	    					   
 	}	
	
	if(max_encoder_cal>2000)
  max_encoder_cal=0;
	
 TIM_ClearITPendingBit(TIM8, TIM_IT_Update|TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4); //����жϱ�־λ		
	 __enable_irq_nested();
}