/*
 *  usart.c
 *
 *  Created on: Jun 26, 2013
 *      Author: Denis aka caat
 */
///////////////////////////////////////////////////////////////////////////////

#include "board.h"

///////////////////////////////////////////////////////////////////////////////

static tRingBuffer RingBufferUART4TX;
static tRingBuffer RingBufferUART4RX;
unsigned int IrqCntUart4;

///////////////////////////////////////////////////////////////////////////////

void InitUart4Buffer(void);

///////////////////////////////////////////////////////////////////////////////

void InitUart4BufferIRQ(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Configure the NVIC Preemption Priority Bits */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    /* Enable the USARTy Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; //Preemption Priority
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

///////////////////////////////////////////////////////////////////////////////

void Usart4Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    USART_ClockInitTypeDef USART_ClockInitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);

    //Set USART4 Tx (PC10) as AF push-pull
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    //Set USART4 Rx (PC11) as input pull down
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(GPIOC, &GPIO_InitStructure);


    USART_ClockStructInit(&USART_ClockInitStructure);
    USART_ClockInit(UART4, &USART_ClockInitStructure);
    //USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_BaudRate = 256000;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No ;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(UART4, &USART_InitStructure);

    //Enable UART4 Receive interrupt
    USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
    //Enable USART4
    USART_Cmd(UART4, ENABLE);

    InitUart4Buffer();
    InitUart4BufferIRQ();
}

///////////////////////////////////////////////////////////////////////////////

int USART_GetChar(void)
{
    return RingBufferGet(&RingBufferUART4RX);
}

///////////////////////////////////////////////////////////////////////////////

int USART_Peek(void)
{
    return RingBufferPeek(&RingBufferUART4RX);
}

///////////////////////////////////////////////////////////////////////////////

int USART_Available(void)
{
    return RingBufferFillLevel(&RingBufferUART4RX);
}

///////////////////////////////////////////////////////////////////////////////

void USART_Flush(void)
{
    while (RingBufferFillLevel(&RingBufferUART4TX) != 0)
        ;
}

///////////////////////////////////////////////////////////////////////////////

void USART_PutCharDirect(uint8_t ch)
{
    while (!(UART4->SR & USART_SR_TXE));

    UART4->DR = ch;
}

///////////////////////////////////////////////////////////////////////////////

void USART_PutChar(uint8_t ch)
{
    //while (!(UART4->SR & USART_SR_TXE));
    //  UART4->DR = ch;
    RingBufferPut(&RingBufferUART4TX, ch, 1);
}

///////////////////////////////////////////////////////////////////////////////

void USART_PutStringDirect(uint8_t *str)
{
    while (*str != 0)
    {
        USART_PutCharDirect(*str);
        str++;
    }
}

///////////////////////////////////////////////////////////////////////////////

void USART_PutString(uint8_t *str)
{
    __disable_irq_nested();

    while (*str != 0)
    {
        USART_PutChar(*str);
        str++;
    }

    __enable_irq_nested();
}

void UART4EnableTxInterrupt(void)
{
    //USART_ITConfig(UART4, USART_IT_TXE, ENABLE);
}

///////////////////////////////////////////////////////////////////////////////

BLDC bldc;
void Data_Receive_Anl(u8 *data_buf,u8 num)
{
	vs16 rc_value_temp;
	u8 sum = 0;
	u8 i;
	for( i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//??sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//????
  if(*(data_buf+2)==0x21)//QR
  {
	bldc.connect=1;
	bldc.lose_cnt=0;
	bldc.en_bldc[0]=*(data_buf+4);
	bldc.en_bldc[1]=*(data_buf+5);
	bldc.en_bldc[2]=*(data_buf+6);	
	bldc.reset=*(data_buf+7);		
	bldc.exp_att[0]=(float)((int16_t)(*(data_buf+8)<<8)|*(data_buf+9))/10.;	
	bldc.exp_att[1]=(float)((int16_t)(*(data_buf+10)<<8)|*(data_buf+11))/10.;	
  bldc.exp_att[2]=(float)((int16_t)(*(data_buf+12)<<8)|*(data_buf+13))/10.;
  bldc.exp_rad[0]=(float)((int16_t)(*(data_buf+14)<<8)|*(data_buf+15))/10.;	
	bldc.exp_rad[1]=(float)((int16_t)(*(data_buf+16)<<8)|*(data_buf+17))/10.;	
  bldc.exp_rad[2]=(float)((int16_t)(*(data_buf+18)<<8)|*(data_buf+19))/10.;		
  bldc.power[0]=*(data_buf+20);
	bldc.power[1]=*(data_buf+21);
	bldc.power[2]=*(data_buf+22);			
	}	
}
u8 TxBuffer5[256];
u8 TxCounter5=0;
u8 count5=0; 
u8 Rx_Buf5[256];	//??????
u8 RxBuffer5[50];
u8 RxState5 = 0;
u8 RxBufferNum5 = 0;
u8 RxBufferCnt5 = 0;
u8 RxLen5 = 0;
static u8 _data_len5 = 0,_data_cnt5 = 0;
void UART4_IRQHandler(void) //UART4 Interrupt handler implementation
{ 
	u8 com_data;
   if(UART4->SR & USART_SR_ORE)//ORE??
	{
		com_data = UART4->DR;
	}

  //????
	if( USART_GetITStatus(UART4,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(UART4,USART_IT_RXNE);//??????

		com_data = UART4->DR;
	
				if(RxState5==0&&com_data==0xAA)
		{
			RxState5=1;
			RxBuffer5[0]=com_data;
		}
		else if(RxState5==1&&com_data==0xAF)
		{
			RxState5=2;
			RxBuffer5[1]=com_data;
		}
		else if(RxState5==2&&com_data>0&&com_data<0XF1)
		{
			RxState5=3;
			RxBuffer5[2]=com_data;
		}
		else if(RxState5==3&&com_data<50)
		{
			RxState5 = 4;
			RxBuffer5[3]=com_data;
			_data_len5 = com_data;
			_data_cnt5 = 0;
		}
		else if(RxState5==4&&_data_len5>0)
		{
			_data_len5--;
			RxBuffer5[4+_data_cnt5++]=com_data;
			if(_data_len5==0)
				RxState5 = 5;
		}
		else if(RxState5==5)
		{
			RxState5 = 0;
			RxBuffer5[4+_data_cnt5]=com_data;
			Data_Receive_Anl(RxBuffer5,_data_cnt5+5);
		}
		else
			RxState5 = 0;
	}
	if( USART_GetITStatus(UART4,USART_IT_TXE ) )
	{
				
		UART4->DR = TxBuffer5[TxCounter5++]; //?DR??????          
		if(TxCounter5 == count5)
		{
			UART4->CR1 &= ~USART_CR1_TXEIE;		//??TXE(????)??
		}
		USART_ClearITPendingBit(UART4,USART_IT_TXE);
	}
		
}

///////////////////////////////////////////////////////////////////////////////

void InitUart4Buffer(void)
{
    RingBufferInit(&RingBufferUART4TX, &UART4EnableTxInterrupt);
    RingBufferInit(&RingBufferUART4RX, 0L);
}

///////////////////////////////////////////////////////////////////////////////

void UsartSend_GOL_LINK(uint8_t ch)
{

while(USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET);
USART_SendData(UART4, ch); 
}

static void Send_Data_GOL_LINK(u8 *dataToSend , u8 length)
{
u16 i;
  for(i=0;i<length;i++)
     UsartSend_GOL_LINK(dataToSend[i]);
}


void Send_BLDC(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50];
	u8 _cnt=0;
	vs16 _temp;
  data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x68;//???
	data_to_send[_cnt++]=0;//???
	
	
	_temp = (vs16)(bldc.attitude[0]*10);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(bldc.attitude[1]*10);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(bldc.attitude[2]*10);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_GOL_LINK(data_to_send, _cnt);
}
