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
//得到三个轴的原始控制量，通过外部中断里面截取定时器值获取
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

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);	//使能TIM2时钟
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);  //使能GPIOA时钟
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;  //PA0 清除之前设置  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //PA0 输入  
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOC,GPIO_Pin_6);						 //PA0 下拉
	GPIO_ResetBits(GPIOC,GPIO_Pin_7);						 //PA0 下拉
	GPIO_ResetBits(GPIOC,GPIO_Pin_8);						 //PA0 下拉
	GPIO_ResetBits(GPIOC,GPIO_Pin_9);						 //PA0 下拉
	//初始化定时器2 TIM2	 
	TIM_TimeBaseStructure.TIM_Period = arr; //设定计数器自动重装值 
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 	//预分频器   
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
  
	//初始化TIM2输入捕获参数
	TIM8_ICInitStructure.TIM_Channel = TIM_Channel_1; //CC1S=01 	选择输入端 IC1映射到TI1上
  TIM8_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
  TIM8_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
  TIM8_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //配置输入分频,不分频 
  TIM8_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 配置输入滤波器 不滤波
  TIM_ICInit(TIM8, &TIM8_ICInitStructure);

	//初始化TIM2输入捕获参数
	TIM8_ICInitStructure.TIM_Channel = TIM_Channel_2; //CC1S=01 	选择输入端 IC1映射到TI1上
  TIM8_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
  TIM8_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
  TIM8_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //配置输入分频,不分频 
  TIM8_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 配置输入滤波器 不滤波
  TIM_ICInit(TIM8, &TIM8_ICInitStructure);
	//初始化TIM2输入捕获参数
	TIM8_ICInitStructure.TIM_Channel = TIM_Channel_3; //CC1S=01 	选择输入端 IC1映射到TI1上
  TIM8_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
  TIM8_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
  TIM8_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //配置输入分频,不分频 
  TIM8_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 配置输入滤波器 不滤波
  TIM_ICInit(TIM8, &TIM8_ICInitStructure);
	//初始化TIM2输入捕获参数
	TIM8_ICInitStructure.TIM_Channel = TIM_Channel_4; //CC1S=01 	选择输入端 IC1映射到TI1上
  TIM8_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
  TIM8_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
  TIM8_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //配置输入分频,不分频 
  TIM8_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 配置输入滤波器 不滤波
  TIM_ICInit(TIM8, &TIM8_ICInitStructure);
	//中断分组初始化
	NVIC_InitStructure.NVIC_IRQChannel = TIM8_CC_IRQn;  //TIM2中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  //先占优先级2级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;  //从优先级0级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器 
	
	TIM_ITConfig(TIM8,TIM_IT_Update|TIM_IT_CC1,ENABLE);//允许更新中断 ,允许CC1IE捕获中断	
	TIM_ITConfig(TIM8,TIM_IT_Update|TIM_IT_CC2,ENABLE);//允许更新中断 ,允许CC1IE捕获中断	
	TIM_ITConfig(TIM8,TIM_IT_Update|TIM_IT_CC3,ENABLE);//允许更新中断 ,允许CC1IE捕获中断	
	TIM_ITConfig(TIM8,TIM_IT_Update|TIM_IT_CC4,ENABLE);//允许更新中断 ,允许CC1IE捕获中断	
  TIM_Cmd(TIM8,ENABLE ); 	//使能定时器2
 
}
float attitude[3];
float rad[3];
float pos[3];
u8 state_dj[5]={0,0,0,0,0};
u8 state_dj_rx[5]={0,0,0,0,0};
u8 IO_STATE[5]={0,0,0,0,0};
u8 IO_STATER[5]={0,0,0,0,0};
u32 cnt_sample1,now_dj[4]={0},lastUpdate_dj[4]={0};
u8  TIM2CH1_CAPTURE_STA[4]={0};	//输入捕获状态		    				
u16	TIM2CH1_CAPTURE_VAL[4]={0};	//输入捕获值
u16 MAX_ENCODER=960;
//定时器5中断服务程序	 
int max_encoder_cal=0;
float dt_encoder_update;
void TIM8_CC_IRQHandler(void)
{ 
	static float att_reg[4];
  u8 sel=PITCH;
	 __disable_irq_nested();
 	if((TIM2CH1_CAPTURE_STA[sel]&0X80)==0)//还未成功捕获	
	{	  
		if (TIM_GetITStatus(TIM8, TIM_IT_Update) != RESET)
		 
		{	    
			if(TIM2CH1_CAPTURE_STA[sel]&0X40)//已经捕获到高电平了
			{
				if((TIM2CH1_CAPTURE_STA[sel]&0X3F)==0X3F)//高电平太长了
				{
					TIM2CH1_CAPTURE_STA[sel]|=0X80;//标记成功捕获了一次
					TIM2CH1_CAPTURE_VAL[sel]=0XFFFF;
				}else TIM2CH1_CAPTURE_STA[sel]++;
			}	
     	
		}
	if (TIM_GetITStatus(TIM8, TIM_IT_CC2) != RESET)//捕获1发生捕获事件
		{	
			if(TIM2CH1_CAPTURE_STA[sel]&0X40)		//捕获到一个下降沿 		
			{	  			
				TIM2CH1_CAPTURE_STA[sel]|=0X80;		//标记成功捕获到一次上升沿
				now_dj[sel] = micros();  //读取时间
				if( now_dj[sel] <lastUpdate_dj[sel]){cnt_sample1 =  ((float)( now_dj[sel]  + (0xffff- lastUpdate_dj[sel])) );}
				else	{ cnt_sample1 =  ((float)( now_dj[sel]  - lastUpdate_dj[sel])); }
				pos[sel]=cnt_sample1;
				if(pos[sel]>max_encoder_cal)
					max_encoder_cal=pos[sel];
				bldc.attitude[sel]=attitude[sel]=Moving_Median(4,5,LIMIT(pos[sel],0,MAX_ENCODER)/MAX_ENCODER*360);
				bldc.en_code_connect[sel]=1;
				bldc.encoder_loss[sel]=0;
		   	TIM_OC2PolarityConfig(TIM8,TIM_ICPolarity_Rising); //CC1P=0 设置为上升沿捕获
			}else  								//还未开始,第一次捕获上升沿
			{
				TIM2CH1_CAPTURE_STA[sel]=0;			//清空
				TIM2CH1_CAPTURE_VAL[sel]=0;
	 			lastUpdate_dj[sel] =  micros();
				TIM2CH1_CAPTURE_STA[sel]|=0X40;		//标记捕获到了上升沿
		   	TIM_OC2PolarityConfig(TIM8,TIM_ICPolarity_Falling);		//CC1P=1 设置为下降沿捕获
			}		 
		}			     	    					   
 	}
//
	sel=ROLL;
 	if((TIM2CH1_CAPTURE_STA[sel]&0X80)==0)//还未成功捕获	
	{	  
		if (TIM_GetITStatus(TIM8, TIM_IT_Update) != RESET)
		 
		{	    
			if(TIM2CH1_CAPTURE_STA[sel]&0X40)//已经捕获到高电平了
			{
				if((TIM2CH1_CAPTURE_STA[sel]&0X3F)==0X3F)//高电平太长了
				{
					TIM2CH1_CAPTURE_STA[sel]|=0X80;//标记成功捕获了一次
					TIM2CH1_CAPTURE_VAL[sel]=0XFFFF;
				}else TIM2CH1_CAPTURE_STA[sel]++;
			}	
    	
		}
	if (TIM_GetITStatus(TIM8, TIM_IT_CC3) != RESET)//捕获1发生捕获事件
		{	
			if(TIM2CH1_CAPTURE_STA[sel]&0X40)		//捕获到一个下降沿 		
			{	  			
				TIM2CH1_CAPTURE_STA[sel]|=0X80;		//标记成功捕获到一次上升沿
				now_dj[sel] = micros();  //读取时间
				if( now_dj[sel] <lastUpdate_dj[sel]){cnt_sample1 =  ((float)( now_dj[sel]  + (0xffff- lastUpdate_dj[sel])) );}
				else	{ cnt_sample1 =  ((float)( now_dj[sel]  - lastUpdate_dj[sel])); }
				pos[sel]=cnt_sample1;
				if(pos[sel]>max_encoder_cal)
				max_encoder_cal=pos[sel];
				dt_encoder_update=(float)Get_Cycle_T(DT_ENCODER)/1000000.;
				bldc.attitude[sel]=attitude[sel]=Moving_Median(5,5,LIMIT(pos[sel],0,MAX_ENCODER)/MAX_ENCODER*360);
        rad[sel]=Moving_Median(1,5,(attitude[sel]-att_reg[sel]));
				att_reg[sel]=attitude[sel];
				bldc.en_code_connect[sel]=1;
				bldc.encoder_loss[sel]=0;
		   	TIM_OC3PolarityConfig(TIM8,TIM_ICPolarity_Rising); //CC1P=0 设置为上升沿捕获
			}else  								//还未开始,第一次捕获上升沿
			{
				TIM2CH1_CAPTURE_STA[sel]=0;			//清空
				TIM2CH1_CAPTURE_VAL[sel]=0;
	 			lastUpdate_dj[sel] =  micros();
				TIM2CH1_CAPTURE_STA[sel]|=0X40;		//标记捕获到了上升沿
		   	TIM_OC3PolarityConfig(TIM8,TIM_ICPolarity_Falling);		//CC1P=1 设置为下降沿捕获
			}		 
   
		}			     	    					   
 	}
//
	sel=YAW;
 	if((TIM2CH1_CAPTURE_STA[sel]&0X80)==0)//还未成功捕获	
	{	  
		if (TIM_GetITStatus(TIM8, TIM_IT_Update) != RESET)
		 
		{	    
			if(TIM2CH1_CAPTURE_STA[sel]&0X40)//已经捕获到高电平了
			{
				if((TIM2CH1_CAPTURE_STA[sel]&0X3F)==0X3F)//高电平太长了
				{
					TIM2CH1_CAPTURE_STA[sel]|=0X80;//标记成功捕获了一次
					TIM2CH1_CAPTURE_VAL[sel]=0XFFFF;
				}else TIM2CH1_CAPTURE_STA[sel]++;
			}	
    	
		}
	if (TIM_GetITStatus(TIM8, TIM_IT_CC4) != RESET)//捕获1发生捕获事件
		{	
			if(TIM2CH1_CAPTURE_STA[sel]&0X40)		//捕获到一个下降沿 		
			{	  			
				TIM2CH1_CAPTURE_STA[sel]|=0X80;		//标记成功捕获到一次上升沿
				now_dj[sel] = micros();  //读取时间
				if( now_dj[sel] <lastUpdate_dj[sel]){cnt_sample1 =  ((float)( now_dj[sel]  + (0xffff- lastUpdate_dj[sel])) );}
				else	{ cnt_sample1 =  ((float)( now_dj[sel]  - lastUpdate_dj[sel])); }
				pos[sel]=cnt_sample1;
				if(pos[sel]>max_encoder_cal)
				max_encoder_cal=pos[sel];
				bldc.attitude[sel]=attitude[sel]=Moving_Median(6,5,LIMIT(pos[sel],0,MAX_ENCODER)/MAX_ENCODER*360);
				bldc.en_code_connect[sel]=1;
				bldc.encoder_loss[sel]=0;
		   	TIM_OC4PolarityConfig(TIM8,TIM_ICPolarity_Rising); //CC1P=0 设置为上升沿捕获
			}else  								//还未开始,第一次捕获上升沿
			{
				TIM2CH1_CAPTURE_STA[sel]=0;			//清空
				TIM2CH1_CAPTURE_VAL[sel]=0;
	 			lastUpdate_dj[sel] =  micros();
				TIM2CH1_CAPTURE_STA[sel]|=0X40;		//标记捕获到了上升沿
		   	TIM_OC4PolarityConfig(TIM8,TIM_ICPolarity_Falling);		//CC1P=1 设置为下降沿捕获
			}		 
   
		}			     	    					   
 	}	
	
	if(max_encoder_cal>2000)
  max_encoder_cal=0;
	
 TIM_ClearITPendingBit(TIM8, TIM_IT_Update|TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4); //清除中断标志位		
	 __enable_irq_nested();
}




void ExtINTConfig(void)
{
		GPIO_InitTypeDef GPIO_InitStructure;
		EXTI_InitTypeDef EXTI_InitStructure;
		NVIC_InitTypeDef NVIC_InitStructure;
		//外中断初始化
		EXTI_DeInit();
		//开PB口时钟
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
		//IO配置 PB8 下拉输入
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 
		GPIO_Init(GPIOC, &GPIO_InitStructure);
		//PB8中断模式 设置
		GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource0);
		EXTI_InitStructure.EXTI_Line = EXTI_Line0;
		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;     //上升沿触发
		EXTI_InitStructure.EXTI_LineCmd = ENABLE;
		EXTI_Init(&EXTI_InitStructure);
			//PB8中断模式 设置
		GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource1);
		EXTI_InitStructure.EXTI_Line = EXTI_Line1;
		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;     //上升沿触发
		EXTI_InitStructure.EXTI_LineCmd = ENABLE;
		EXTI_Init(&EXTI_InitStructure);
				//PB8中断模式 设置
		GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource2);
		EXTI_InitStructure.EXTI_Line = EXTI_Line2;
		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;     //上升沿触发
		EXTI_InitStructure.EXTI_LineCmd = ENABLE;
		EXTI_Init(&EXTI_InitStructure);
		//外中断配置
		NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
		//外中断配置
		NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
		//外中断配置
		NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
}
u32 now_encoder[3]={0},last_encoder[3]={0};
u8 io_state[3]={0};
void EXTI0_IRQHandler(void)
{
	u32 temp;
  	if(EXTI_GetITStatus(EXTI_Line0)) 
		{
			
			switch(io_state[0])
			{
			  case 0:
					if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_0)==1)
					{io_state[0]=1;last_encoder[0] =  micros();}
			  break;
				case 1:
	        if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_0)==0)
					{io_state[0]=0;
					now_encoder[0] = micros();  //读取时间
					if( now_encoder[0] <last_encoder[0]){temp =  ((float)( now_encoder[0]  + (0xffff- last_encoder[0])) );}
					else	{ temp =  ((float)( now_encoder[0]  - last_encoder[0])); }
					pos[0]=temp;
					if(pos[0]>max_encoder_cal)
					max_encoder_cal=pos[0];
					bldc.attitude[0]=attitude[0]=Moving_Median(5,5,LIMIT(pos[0],0,MAX_ENCODER)/MAX_ENCODER*360);
					bldc.en_code_connect[0]=1;
					bldc.encoder_loss[0]=0;		
					}
				break;
			}
				//清零中断
				EXTI_ClearFlag(EXTI_Line0);  
				EXTI_ClearITPendingBit(EXTI_Line0);
		}  
}	

void EXTI1_IRQHandler(void)
{
	u32 temp;
  	if(EXTI_GetITStatus(EXTI_Line1)) 
		{
			
			switch(io_state[1])
			{
			  case 0:
					if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_1)==1)
					{io_state[1]=1;last_encoder[1] =  micros();}
			  break;
				case 1:
	        if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_1)==0)
					{io_state[1]=0;
					now_encoder[1] = micros();  //读取时间
					if( now_encoder[1] <last_encoder[1]){temp =  ((float)( now_encoder[1]  + (0xffff- last_encoder[1])) );}
					else	{ temp =  ((float)( now_encoder[1]  - last_encoder[1])); }
					pos[1]=temp;
					if(pos[1]>max_encoder_cal)
					max_encoder_cal=pos[1];
					bldc.attitude[1]=attitude[1]=Moving_Median(6,5,LIMIT(pos[1],0,MAX_ENCODER)/MAX_ENCODER*360);
					bldc.en_code_connect[1]=1;
					bldc.encoder_loss[1]=0;		
					}
				break;
			}
				//清零中断
				EXTI_ClearFlag(EXTI_Line1);  
				EXTI_ClearITPendingBit(EXTI_Line1);
		}  
}		   


void EXTI2_IRQHandler(void)
{
	u32 temp;
  	if(EXTI_GetITStatus(EXTI_Line2)) 
		{
			
			switch(io_state[2])
			{
			  case 0:
					if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_2)==1)
					{io_state[2]=1;last_encoder[2] =  micros();}
			  break;
				case 1:
	        if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_2)==0)
					{io_state[2]=0;
					now_encoder[2] = micros();  //读取时间
					if( now_encoder[2] <last_encoder[2]){temp =  ((float)( now_encoder[2]  + (0xffff- last_encoder[2])) );}
					else	{ temp =  ((float)( now_encoder[2]  - last_encoder[2])); }
					pos[2]=temp;
					if(pos[2]>max_encoder_cal)
					max_encoder_cal=pos[2];
					bldc.attitude[2]=attitude[2]=Moving_Median(7,5,LIMIT(pos[2],0,MAX_ENCODER)/MAX_ENCODER*360);
					bldc.en_code_connect[2]=1;
					bldc.encoder_loss[2]=0;		
					}
				break;
			}
				//清零中断
				EXTI_ClearFlag(EXTI_Line2);  
				EXTI_ClearITPendingBit(EXTI_Line2);
		}  
}		   



   
//初始化ADC
//这里我们仅以规则通道为例
//我们默认将开启通道0~3																	   
void  Adc_Init(void)
{ 	
	ADC_InitTypeDef ADC_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |RCC_APB2Periph_ADC1	, ENABLE );	  //使能ADC1通道时钟
 

	RCC_ADCCLKConfig(RCC_PCLK2_Div6);   //设置ADC分频因子6 72M/6=12,ADC最大时间不能超过14M

	//PA1 作为模拟通道输入引脚                         
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//模拟输入引脚
	GPIO_Init(GPIOA, &GPIO_InitStructure);	

	ADC_DeInit(ADC1);  //复位ADC1,将外设 ADC1 的全部寄存器重设为缺省值

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//ADC工作模式:ADC1和ADC2工作在独立模式
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;	//模数转换工作在单通道模式
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;	//模数转换工作在单次转换模式
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//转换由软件而不是外部触发启动
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//ADC数据右对齐
	ADC_InitStructure.ADC_NbrOfChannel = 1;	//顺序进行规则转换的ADC通道的数目
	ADC_Init(ADC1, &ADC_InitStructure);	//根据ADC_InitStruct中指定的参数初始化外设ADCx的寄存器   

  
	ADC_Cmd(ADC1, ENABLE);	//使能指定的ADC1
	
	ADC_ResetCalibration(ADC1);	//使能复位校准  
	 
	while(ADC_GetResetCalibrationStatus(ADC1));	//等待复位校准结束
	
	ADC_StartCalibration(ADC1);	 //开启AD校准
 
	while(ADC_GetCalibrationStatus(ADC1));	 //等待校准结束
 
//	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//使能指定的ADC1的软件转换启动功能

}				  
//获得ADC值
//ch:通道值 0~3
u16 Get_Adc(u8 ch)   
{
  	//设置指定ADC的规则组通道，一个序列，采样时间
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_239Cycles5 );	//ADC1,ADC通道,采样时间为239.5周期	  			    
  
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//使能指定的ADC1的软件转换启动功能	
	 
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//等待转换结束

	return ADC_GetConversionValue(ADC1);	//返回最近一次ADC1规则组的转换结果
}



