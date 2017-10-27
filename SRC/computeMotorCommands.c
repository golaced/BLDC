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

float mechanical2electricalDegrees[3] = { 1.0f, 1.0f, 1.0f };
float electrical2mechanicalDegrees[3] = { 1.0f, 1.0f, 1.0f };

float outputRate[3];

float pidCmd[3];

float pidCmdPrev[3] = {0.0f, 0.0f, 0.0f};

float yawCmd;

#define YAP_DEADBAND     2.00f  // in radians with respect to one motor pole, actual angle is (DEADBAND / numberPoles) * R2D
#define MOTORPOS2SETPNT  0.35f  // scaling factor for how fast it should move
#define AUTOPANSMOOTH   40.00f

float centerPoint = 0.0f;
float stepSmooth  = 0.0f;
float step        = 0.0f;

///////////////////////////////////////////////////////////////////////////////
// Yaw AutoPan
///////////////////////////////////////////////////////////////////////////////

float autoPan(float motorPos, float setpoint)
{
    if (motorPos < centerPoint - YAP_DEADBAND)
    {
        centerPoint = YAP_DEADBAND;
        step = MOTORPOS2SETPNT * motorPos; //  dampening
    }
    else if (motorPos > centerPoint + YAP_DEADBAND)
    {
        centerPoint = -YAP_DEADBAND;
        step = MOTORPOS2SETPNT * motorPos; //  dampening
    }
    else
    {
        step = 0.0f;
        centerPoint = 0.0f;
    }

    stepSmooth = (stepSmooth * (AUTOPANSMOOTH - 1.0f) + step) / AUTOPANSMOOTH;

    return (setpoint -= stepSmooth);
}

///////////////////////////////////////////////////////////////////////////////
// Compute Motor Commands
///////////////////////////////////////////////////////////////////////////////
float spd[3]={0,0,0};
float spd_out[3]={0},spd_out_flt[3]={0};
float flt=0.68;
void computeMotorCommands(float dt)
{
    holdIntegrators = false;//���û�������

    ///////////////////////////////////

    if (eepromConfig.rollEnabled == true)//���ʹ���� ��ת�ᣨROLL��
    {
			//����PID������ŵ�pidCmd[ROLL]
        pidCmd[ROLL] = updatePID(pointingCmd[ROLL] * mechanical2electricalDegrees[ROLL],
                                 -sensors.margAttitude500Hz[ROLL] * mechanical2electricalDegrees[ROLL],
                                 dt, holdIntegrators, 
                                 &eepromConfig.PID[ROLL_PID]);//mechanical2electricalDegrees = motor poles/2   14poles/2=7
			
			//��ǰPID�����ȥ�ϴ�PID����õ��仯��
        outputRate[ROLL] = pidCmd[ROLL] - pidCmdPrev[ROLL];

        if (outputRate[ROLL] > eepromConfig.rateLimit)//����仯��������ת�ٶ����Ƶ����ֵ
            pidCmd[ROLL] = pidCmdPrev[ROLL] + eepromConfig.rateLimit;//��ôʹ���ϴ�PID������+��ת�ٶ����Ƶ����ֵ ��Ϊ����PID���

        if (outputRate[ROLL] < -eepromConfig.rateLimit)//����仯��С����ת�ٶ����Ƶ���Сֵ
            pidCmd[ROLL] = pidCmdPrev[ROLL] - eepromConfig.rateLimit;//��ôʹ���ϴ�PID������-��ת�ٶ����Ƶ���Сֵ����ȥһ������
					//	�൱�ڼ������������ֵ������Ϊ����PID���

        pidCmdPrev[ROLL] = pidCmd[ROLL];//���汾��PID�������� pidCmdPrev[ROLL] ��Ϊ�ɵ�ֵ��������´Σ���
         if(spd[ROLL]!=0)
					 spd_out[ROLL]-=dt*spd[ROLL];
				 else 
					 spd_out[ROLL]+=dt*exp_rad[ROLL];
         spd_out_flt[ROLL]= spd_out[ROLL]*flt+(1-flt)*spd_out_flt[ROLL];
         setRollMotor(spd_out_flt[ROLL], (int)eepromConfig.rollPower);// roll power is 50 in default 				 
			
    }

    ///////////////////////////////////

    if (eepromConfig.pitchEnabled == true)//���ʹ���� �����ᣨPitch��
    {
			//����PID������ŵ�pidCmd[PITCH]
        pidCmd[PITCH] = updatePID(pointingCmd[PITCH] * mechanical2electricalDegrees[PITCH],
                                  sensors.margAttitude500Hz[PITCH] * mechanical2electricalDegrees[PITCH],
                                  dt, holdIntegrators, &eepromConfig.PID[PITCH_PID]);

        outputRate[PITCH] = pidCmd[PITCH] - pidCmdPrev[PITCH];//��ǰPID�����ȥ�ϴ�PID����õ��仯��

        if (outputRate[PITCH] > eepromConfig.rateLimit)//����仯��������ת�ٶ����Ƶ����ֵ
            pidCmd[PITCH] = pidCmdPrev[PITCH] + eepromConfig.rateLimit;//��ôʹ���ϴ�PID������+��ת�ٶ����Ƶ����ֵ ��Ϊ����PID���

        if (outputRate[PITCH] < -eepromConfig.rateLimit)//����仯��С����ת�ٶ����Ƶ���Сֵ
            pidCmd[PITCH] = pidCmdPrev[PITCH] - eepromConfig.rateLimit;//��ôʹ���ϴ�PID������-��ת�ٶ����Ƶ���Сֵ����ȥһ������
					//	�൱�ڼ������������ֵ������Ϊ����PID���

        pidCmdPrev[PITCH] = pidCmd[PITCH];//���汾��PID�������� pidCmdPrev[PITCH] ��Ϊ�ɵ�ֵ��������´Σ���
    if(spd[PITCH]!=0){spd_out[PITCH]-=dt*spd[PITCH];
					setPitchMotor(spd_out[PITCH], (int)eepromConfig.pitchPower);// roll power is 50 in default 
				 }else {spd_out[PITCH]+=dt*exp_rad[PITCH];
				  setRollMotor(spd_out[PITCH], (int)eepromConfig.pitchPower);// roll power is 50 in default
         }			
    }

    ///////////////////////////////////

    if (eepromConfig.yawEnabled == true)//���ʹ���� ƫ���ᣨYAW��
    {
        if (eepromConfig.yawAutoPanEnabled == true)//���ʹ����ƫ�����Զ�ȫ��
            yawCmd = autoPan(pidCmd[YAW], yawCmd * mechanical2electricalDegrees[YAW]) * electrical2mechanicalDegrees[YAW];
        else
            yawCmd = -pointingCmd[YAW];

				//����PID������ŵ�pidCmd[YAW]
        pidCmd[YAW] = updatePID(yawCmd * mechanical2electricalDegrees[YAW],
                                sensors.margAttitude500Hz[YAW] * mechanical2electricalDegrees[YAW],
                                dt, holdIntegrators, &eepromConfig.PID[YAW_PID]);

        outputRate[YAW] = pidCmd[YAW] - pidCmdPrev[YAW];//��ǰPID�����ȥ�ϴ�PID����õ��仯��

        if (outputRate[YAW] > eepromConfig.rateLimit)//����仯��������ת�ٶ����Ƶ����ֵ
            pidCmd[YAW] = pidCmdPrev[YAW] + eepromConfig.rateLimit;//��ôʹ���ϴ�PID������+��ת�ٶ����Ƶ����ֵ ��Ϊ����PID���

        if (outputRate[YAW] < -eepromConfig.rateLimit)//����仯��С����ת�ٶ����Ƶ���Сֵ
            pidCmd[YAW] = pidCmdPrev[YAW] - eepromConfig.rateLimit;//��ôʹ���ϴ�PID������-��ת�ٶ����Ƶ���Сֵ����ȥһ������
					//	�൱�ڼ������������ֵ������Ϊ����PID���

        pidCmdPrev[YAW] = pidCmd[YAW];//���汾��PID�������� pidCmdPrev[PITCH] ��Ϊ�ɵ�ֵ��������´Σ���
       if(spd[YAW]!=0){spd_out[YAW]-=dt*spd[YAW];
					setYawMotor(spd_out[YAW], (int)eepromConfig.yawPower);// roll power is 50 in default 
				 }
				// else
       // setYawMotor(pidCmd[YAW], (int)eepromConfig.yawPower);
    }

    ///////////////////////////////////

}

///////////////////////////////////////////////////////////////////////////////
