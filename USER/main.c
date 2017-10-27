
/*
  Sept 2013

  bgc32 Rev -

  Copyright (c) 2013 John Ihlein.  All rights reserved.

  Open Source STM32 Based Brushless Gimbal Controller Software

  Includes code and/or ideas from:

  1)AeroQuad
  2)BaseFlight
  3)CH Robotics
  4)MultiWii
  5)S.O.H. Madgwick
  6)UAVX

  Designed to run on the EvvGC Brushless Gimbal Controller Board

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/
 
/*  

instructions:
	
	SimpleBGC 开源三轴无刷云台算法完全解说。
	An interpreter：康朝阳
	Email：547336083@qq.com
	date:2016/09/19
	
*/
///////////////////////////////////////////////////////////////////////////////

#include "board.h"

///////////////////////////////////////////////////////////////////////////////

eepromConfig_t eepromConfig;

sensors_t      sensors;

float          testPhase      = -1.0f * D2R;
float          testPhaseDelta = 10.0f * D2R;

uint16_t       timerValue;

/////////////////////////////////////////////////////////////////////////////// 
float exp_angle[3]={90+180,90};
float exp_rad[3]={0};
float kp=8.8;
float kd=0;
u8 test[3]={0};
void position_control(float dt)
{ u8 i;
  float ero[3];
	static float ero_reg[3];
	for(i=0;i<3;i++)
	{
	  ero[i]=LIMIT(exp_angle[i]-attitude[i],-99,99);
	  exp_rad[i]=ero[i]*kp+(ero[i]-ero_reg[i])*dt*kd;
		ero_reg[i]=ero[i];
	}

}	

int main(void)
{  
	u8 i;
    uint32_t currentTime;

    systemInit();

    //initOrientation();

    systemReady = true;

	  while (1)
		{  
			 if(1){
			
			  if (frame_50Hz)
        {
            frame_50Hz = false;

            currentTime      = micros();
            deltaTime50Hz    = currentTime - previous50HzTime;
            previous50HzTime = currentTime;

           // processPointingCommands();//接收遥控器控制指令。
					
            executionTime50Hz = micros() - currentTime;
        }
				
				  if (frame_10Hz)
        {
            frame_10Hz = false;

            currentTime      = micros();
            deltaTime10Hz    = currentTime - previous10HzTime;
            previous10HzTime = currentTime;

            // HJI if (newMagData == true)
            // HJI {
            // HJI     sensors.mag10Hz[XAXIS] =   (float)rawMag[XAXIS].value * magScaleFactor[XAXIS] - eepromConfig.magBias[XAXIS];
            // HJI     sensors.mag10Hz[YAXIS] =   (float)rawMag[YAXIS].value * magScaleFactor[YAXIS] - eepromConfig.magBias[YAXIS];
            // HJI     sensors.mag10Hz[ZAXIS] = -((float)rawMag[ZAXIS].value * magScaleFactor[ZAXIS] - eepromConfig.magBias[ZAXIS]);

            // HJI     newMagData = false;
            // HJI     magDataUpdate = true;
            // HJI }

            //cliCom();//接收来自串口的命令并处理
					  static float angle;
						if(test[1])
						{
						exp_angle[0]=exp_angle[1]=180+90*sin(angle/57.3);	
						angle+=0.1*10;
						if(angle>360)angle=0;
						}
            executionTime10Hz = micros() - currentTime;
        }

        ///////////////////////////////
				
		 if (frame_500Hz)
        { //本if分支里面是云台算法核心
#ifdef _DTIMING
            LA2_ENABLE;
#endif
            frame_500Hz = false;

            currentTime       = micros();//获取当前时间
            deltaTime500Hz    = currentTime - previous500HzTime;//得到时间增量
            previous500HzTime = currentTime;//保存当前时间

            TIM_Cmd(TIM6, DISABLE);//关闭TIM6
            timerValue = TIM_GetCounter(TIM6);//得到TIM6计数值 ，单位 0.5 uSec Tick
            TIM_SetCounter(TIM6, 0);//清空 TIMx->CNT 寄存器
            TIM_Cmd(TIM6, ENABLE);//开启TIM6 ，准备获取下一次的时间增量

						//定时器计数值*0.5us,得到时间增量，单位是 us 
            dt500Hz = (float)timerValue * 0.0000005f;  
            position_control(dt500Hz);
						for(i=0;i<4;i++)
						if(TIM2CH1_CAPTURE_STA[i]&0X80)//成功捕获到了一次高电平
						{
						TIM2CH1_CAPTURE_STA[i]=0;			//开启下一次捕获
						}
						}
            executionTime500Hz = micros() - currentTime;//本次运算执行时间长度保存到executionTime500Hz

#ifdef _DTIMING
            LA2_DISABLE;
#endif
        }
		
		
				
				
				    ///////////////////////////////

        if (frame_100Hz)
        {
            frame_100Hz = false;
         
            currentTime       = micros();
            deltaTime100Hz    = currentTime - previous100HzTime;
            previous100HzTime = currentTime;
            computeMotorCommands(dt500Hz);//计算电机控制量，入口参数时间增量Dt
            executionTime100Hz = micros() - currentTime;
					 
        }

        ///////////////////////////////

        if (frame_5Hz)
        {
            frame_5Hz = false;

            currentTime     = micros();
            deltaTime5Hz    = currentTime - previous5HzTime;
            previous5HzTime = currentTime;

            LED2_TOGGLE;

            executionTime5Hz = micros() - currentTime;
        }

        ///////////////////////////////

        if (frame_1Hz)
        {
            frame_1Hz = false;

            currentTime     = micros();
            deltaTime1Hz    = currentTime - previous1HzTime;
            previous1HzTime = currentTime;

            LED1_TOGGLE;
						static u8 flag;
						if(test[0])
						{
						if(flag)
						exp_angle[0]=exp_angle[1]=90;
						else
						exp_angle[0]=exp_angle[1]=90+180;	
						flag=!flag;
						}
						executionTime1Hz = micros() - currentTime;
        }
		}
}

///////////////////////////////////////////////////////////////////////////////

