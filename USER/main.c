
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
	
	SimpleBGC ��Դ������ˢ��̨�㷨��ȫ��˵��
	An interpreter��������
	Email��547336083@qq.com
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
u8 test[4];
float rad_sin=111;
int main(void)
{
    uint32_t currentTime;
    u8 i;
    systemInit();

    initOrientation();

    systemReady = true;

    while (1)
    {
        ///////////////////////////////

        if (frame_50Hz)
        {
            frame_50Hz = false;

            currentTime      = micros();
            deltaTime50Hz    = currentTime - previous50HzTime;
            previous50HzTime = currentTime;

            processPointingCommands();//����ң��������ָ�
            Send_BLDC();
            executionTime50Hz = micros() - currentTime;
        }

        ///////////////////////////////

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

            cliCom();//�������Դ��ڵ��������
            static float angle;
						if(test[1])
						{
						exp_angle[0]=exp_angle[1]=180+90*sin(angle/57.3);	
						angle+=0.1*rad_sin;
						if(angle>360)angle=0;
						}
            executionTime10Hz = micros() - currentTime;
        }

        ///////////////////////////////

        if (frame_500Hz)
        { //��if��֧��������̨�㷨����
#ifdef _DTIMING
            LA2_ENABLE;
#endif
            frame_500Hz = false;

            currentTime       = micros();//��ȡ��ǰʱ��
            deltaTime500Hz    = currentTime - previous500HzTime;//�õ�ʱ������
            previous500HzTime = currentTime;//���浱ǰʱ��

            TIM_Cmd(TIM6, DISABLE);//�ر�TIM6
            timerValue = TIM_GetCounter(TIM6);//�õ�TIM6����ֵ ����λ 0.5 uSec Tick
            TIM_SetCounter(TIM6, 0);//��� TIMx->CNT �Ĵ���
            TIM_Cmd(TIM6, ENABLE);//����TIM6 ��׼����ȡ��һ�ε�ʱ������

						//��ʱ������ֵ*0.5us,�õ�ʱ����������λ�� us 
            dt500Hz = (float)timerValue * 0.0000005f;  

						//(1/8192) * 9.8065  (8192 LSB = 1 G)
					  //���������������ĵ�ǰ���ٶ�����-�¶Ȳ���ƫ�* ��С�ֱ��� ����С�ֱ��ʾ��ǣ�(1/8192) * 9.8065�� 
					 //1G���̵�8192����������֮1����Ӧ�������ٶ�9.8065m/1G��8192��֮1
					//accelData500Hz��������Systick�ж����汻���µģ����µ�ֵ�Ǿ�������������ֵ������Ƶ����500hz
            sensors.accel500Hz[XAXIS] =  ((float)accelData500Hz[XAXIS] - accelTCBias[XAXIS]) * ACCEL_SCALE_FACTOR;
            sensors.accel500Hz[YAXIS] =  ((float)accelData500Hz[YAXIS] - accelTCBias[YAXIS]) * ACCEL_SCALE_FACTOR;
            sensors.accel500Hz[ZAXIS] = -((float)accelData500Hz[ZAXIS] - accelTCBias[ZAXIS]) * ACCEL_SCALE_FACTOR;

						// (1/65.5) * pi/180   (65.5 LSB = 1 DPS)
						//���������������ĵ�ǰ����������-�����Ǿ�ֹ״̬�»���5000�ε�ƽ��ֵ-�¶Ȳ���ƫ�* ��С�ֱ��� 
						//��С�ֱ��ʾ��� (1/65.5) * pi/180  
						//1DPS����������ֵ��65.5, 65.5��֮1 ���� PI Ȼ�����180 ������С�ֱ���
						//gyroData500Hz��������Systick�ж����汻���µģ����µ�ֵ�Ǿ�������������ֵ������Ƶ����500hz
            sensors.gyro500Hz[ROLL ] =  ((float)gyroData500Hz[ROLL ] - gyroRTBias[ROLL ] - gyroTCBias[ROLL ]) * GYRO_SCALE_FACTOR;
            sensors.gyro500Hz[PITCH] =  ((float)gyroData500Hz[PITCH] - gyroRTBias[PITCH] - gyroTCBias[PITCH]) * GYRO_SCALE_FACTOR;
            sensors.gyro500Hz[YAW  ] = -((float)gyroData500Hz[YAW  ] - gyroRTBias[YAW  ] - gyroTCBias[YAW  ]) * GYRO_SCALE_FACTOR;


						//��ǰ��λ�������㣬���� accAngleSmooth ��ͨ�����ٶ����ݾ���atan2f�������������ŷ���ǣ����ҽ�����һ���ͺ��˲�
						//�����˲��㷨Ҳ���ڵ�ͨ�˲���һ�֣����ŵ㣺 �������Ը��ž������õ��������� �����ڲ���Ƶ�ʽϸߵĳ���,
						// ȱ�㣺 ��λ�ͺ������ȵ� �ͺ�̶�ȡ����aֵ��С ���������˲�Ƶ�ʸ��ڲ���Ƶ�ʵ�1/2�ĸ����ź�,
						//getOrientation�����ڲ�ʹ����accAngleSmoothŷ���������������ݽ����˻����˲��ں��㷨�õ��ȶ���ŷ���ǣ�����
						//��ŵ�sensors.evvgcCFAttitude500Hz���棬sensors.accel500Hz��sensors.gyro500Hz�Ǿ��������㷨�����ļ��ٶ�����
						//�����������ݣ�dt500Hz��ʱ��������Ҳ����ִ��if (frame_500Hz){} ����Ĵ�����
            getOrientation(accAngleSmooth, sensors.evvgcCFAttitude500Hz, sensors.accel500Hz, sensors.gyro500Hz, dt500Hz);


						//�Լ��ٶ����ݽ��� һ�׵�ͨ�˲� ������sensors.accel500Hz�Ǵ������˲���ֵ��firstOrderFilters���˲�������
						// Low Pass:
						// GX1 = 1 / (1 + A)
						// GX2 = 1 / (1 + A)
						// GX3 = (1 - A) / (1 + A)
            sensors.accel500Hz[ROLL ] = firstOrderFilter(sensors.accel500Hz[ROLL ], &firstOrderFilters[ACCEL_X_500HZ_LOWPASS ]);
            sensors.accel500Hz[PITCH] = firstOrderFilter(sensors.accel500Hz[PITCH], &firstOrderFilters[ACCEL_Y_500HZ_LOWPASS]);
            sensors.accel500Hz[YAW  ] = firstOrderFilter(sensors.accel500Hz[YAW  ], &firstOrderFilters[ACCEL_Z_500HZ_LOWPASS  ]);

						//���˲ο�ϵͳ���£���ڲ������������������ݣ�������ٶ����ݣ��������������,�Լ�ָʾ�Ƿ���´��������ݵ�magDataUpdate������
						//magDataUpdate=false��ʾ�����´��������ݣ�magDataUpdate=true��ʾ���´��������ݣ����һ��������ʱ������Dt��Ҳ���Ǵ˺�����
						//��ִ�����ϴ�ִ�е�ʱ������
            MargAHRSupdate(sensors.gyro500Hz[ROLL],   sensors.gyro500Hz[PITCH],  sensors.gyro500Hz[YAW],
                           sensors.accel500Hz[XAXIS], sensors.accel500Hz[YAXIS], sensors.accel500Hz[ZAXIS],
                           sensors.mag10Hz[XAXIS],    sensors.mag10Hz[YAXIS],    sensors.mag10Hz[ZAXIS],
                           magDataUpdate , dt500Hz);

            magDataUpdate = false;//Ĭ�ϲ����´���������

            //printUSART("r=%.1f,p=%.1f,y=%.1f\r\n",sensors.margAttitude500Hz[ROLL ],sensors.margAttitude500Hz[PITCH],sensors.margAttitude500Hz[YAW ]);


						for(i=0;i<4;i++)
						if(TIM2CH1_CAPTURE_STA[i]&0X80)//�ɹ�������һ�θߵ�ƽ
						{
						TIM2CH1_CAPTURE_STA[i]=0;			//������һ�β���
						}
						
				
            computeMotorCommands(dt500Hz);//����������������ڲ���ʱ������Dt

            executionTime500Hz = micros() - currentTime;//��������ִ��ʱ�䳤�ȱ��浽executionTime500Hz

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
            if(bldc.connect)
						{
						exp_angle[0]=bldc.exp_att[0];
						exp_angle[1]=bldc.exp_att[1];
						exp_angle[2]=bldc.exp_att[2];
						eepromConfig.pitchEnabled=bldc.en_bldc[0];
						eepromConfig.rollEnabled=bldc.en_bldc[1];		
						eepromConfig.yawEnabled=bldc.en_bldc[2];
						eepromConfig.pitchPower=LIMIT(bldc.power[0],0,100);
						eepromConfig.rollPower= LIMIT(bldc.power[1],0,100);		
						eepromConfig.yawPower=  LIMIT(bldc.power[2],0,100);	
            if(bldc.exp_rad[0]!=0)
            spd[0]=	bldc.exp_rad[0]*57.3;						
						if(bldc.exp_rad[1]!=0)
            spd[1]=	bldc.exp_rad[1]*57.3;						
				    if(bldc.exp_rad[2]!=0)
            spd[2]=	bldc.exp_rad[2]*57.3;						
					  }
						if(bldc.en_bldc_force[0])
						eepromConfig.pitchEnabled=1;
						if(bldc.en_bldc_force[1])
						eepromConfig.rollEnabled=1;
						if(bldc.en_bldc_force[2])
						eepromConfig.yawEnabled=1;
						if(bldc.reset){bldc.reset=0;
							eepromConfig.pitchEnabled=eepromConfig.yawEnabled=eepromConfig.rollEnabled=0;
						}
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
						if(bldc.lose_cnt++>2)
							 bldc.connect=0;
						
						for(i=0;i<3;i++)
						if(bldc.encoder_loss[i]++>2){
						bldc.en_code_connect[i]=0;
						}
            executionTime1Hz = micros() - currentTime;
        }

        ////////////////////////////////
    }
}

///////////////////////////////////////////////////////////////////////////////

