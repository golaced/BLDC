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

///////////////////////////////////////////////////////////////////////////////

#include "board.h"

///////////////////////////////////////////////////////////////////////////////

uint8_t holdIntegrators = true;

#define F_CUT 20.0f

static float rc;

///////////////////////////////////////////////////////////////////////////////

void initPID(void)
{
    uint8_t index;

    rc = 1.0f / ( TWO_PI * F_CUT );

    for (index = 0; index < NUMBER_OF_PIDS; index++)
    {
    	eepromConfig.PID[index].iTerm          = 0.0f;
    	eepromConfig.PID[index].lastDcalcValue = 0.0f;
    	eepromConfig.PID[index].lastDterm      = 0.0f;
    	eepromConfig.PID[index].lastLastDterm  = 0.0f;
	}
}

///////////////////////////////////////////////////////////////////////////////

float updatePID(float command/*ң�������ƵĻ�е�Ƕ�ת���ɵ��ӽǶ�*/, 
								float state/*��Ԫ������Ļ�е�Ƕ�ת���ɵ��ӽǶ�*/, 
								float deltaT/*ʱ������dT*/,
								uint8_t iHold/*�Ƿ�ӵ�л���I*/, 
								struct PIDdata *PIDparameters/*PID����*/)
{
    float error;
    float dTerm;
    float dTermFiltered;
    float dAverage;

    ///////////////////////////////////
	  //PID��ʽ
		/* Kp*e + Ki*��edt + Kd*��de/dt�� */
	
    error = command - state;//ң�������Ƶ������Ƕ�-��Ԫ������Ļ���Ƕ� �õ���ǰƫ�� ����ʽ e

    if (PIDparameters->type == ANGULAR)//�������Ϊ�Ƕ�
        error = standardRadianFormat(error);//ת���ɱ�׼����

    ///////////////////////////////////

    if (iHold == false)//���iHold == false�ż�������Ĭ�����û�����
    {
    	PIDparameters->iTerm += error * deltaT;//�������л��֣���ʽ ��edt
    	PIDparameters->iTerm = constrain(PIDparameters->iTerm, -PIDparameters->windupGuard, PIDparameters->windupGuard);//�����޷�
    }

    ///////////////////////////////////
	
    if (PIDparameters->dErrorCalc == D_ERROR)  // Calculate D term from error
    {
			//ͨ��error����΢�����ʽ��de/dt��������de����(error - PIDparameters->lastDcalcValue) 
		dTerm = (error - PIDparameters->lastDcalcValue) / deltaT;
        PIDparameters->lastDcalcValue = error;//���浱ǰƫ��
	}
	else                                       // Calculate D term from state
	{
		//����ʹ����Ԫ������Ļ�е�Ƕ�ת���ɵ��ӽǶȵ�ֵ����΢������
		dTerm = (PIDparameters->lastDcalcValue - state) / deltaT;

		if (PIDparameters->type == ANGULAR)//�������Ϊ�Ƕ�
		    dTerm = standardRadianFormat(dTerm);//ת���ɱ�׼����

		PIDparameters->lastDcalcValue = state;//���浱ǰ״̬
	}

    ///////////////////////////////////
		//��΢�������һ�׵�ͨ�˲� deltaT / (rc + deltaT) ��������˲�ϵ��a �� rc=1.0f/(2.0f*PI*F)
    dTermFiltered = PIDparameters->lastDterm + deltaT / (rc + deltaT) * (dTerm - PIDparameters->lastDterm);

	//����ʷ����΢���������ƽ��
    dAverage = (dTermFiltered + PIDparameters->lastDterm + PIDparameters->lastLastDterm) * 0.333333f;

    PIDparameters->lastLastDterm = PIDparameters->lastDterm;//�ϴ�΢����浽���ϴ�΢����洢����
    PIDparameters->lastDterm = dTermFiltered;//��ǰ΢����浽�ϴ�΢����洢����

    ///////////////////////////////////
//����PID������
    if (PIDparameters->type == ANGULAR)//�������Ϊ�Ƕ�
        return(PIDparameters->P * error     /*  Kp*e  */           +
	           PIDparameters->I * PIDparameters->iTerm + /*   Ki*��edt   */
	           PIDparameters->D * dAverage);/*   Kd*��de/dt��  */
    else
        return(PIDparameters->P * PIDparameters->B * command /* Kp *(B * point) */  +
               PIDparameters->I * PIDparameters->iTerm /*   Ki*��edt   */      +
               PIDparameters->D * dAverage   /*   Kd*��de/dt��  */ -
               PIDparameters->P * state);//��������

    ///////////////////////////////////
}

///////////////////////////////////////////////////////////////////////////////

void setPIDintegralError(uint8_t IDPid, float value)
{
	eepromConfig.PID[IDPid].iTerm = value;
}

///////////////////////////////////////////////////////////////////////////////

void zeroPIDintegralError(void)
{
    uint8_t index;

    for (index = 0; index < NUMBER_OF_PIDS; index++)
         setPIDintegralError(index, 0.0f);
}

///////////////////////////////////////////////////////////////////////////////

void setPIDstates(uint8_t IDPid, float value)
{
    eepromConfig.PID[IDPid].lastDcalcValue = value;
    eepromConfig.PID[IDPid].lastDterm      = value;
    eepromConfig.PID[IDPid].lastLastDterm  = value;
}

///////////////////////////////////////////////////////////////////////////////

void zeroPIDstates(void)
{
    uint8_t index;

    for (index = 0; index < NUMBER_OF_PIDS; index++)
         setPIDstates(index, 0.0f);
}

///////////////////////////////////////////////////////////////////////////////


