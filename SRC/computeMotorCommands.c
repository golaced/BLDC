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

float exp_angle[3]={90,90};
float exp_rad[3]={0};
float kp=8.8;
float ki=3.6;
void position_control(float dt)
{ u8 i;
  float ero[3];
	static float interge[3];
	static float ero_reg[3];
	if(ki==0)
		interge[0]=interge[1]=interge[2]=0;
		if(eepromConfig.pitchEnabled||!bldc.en_code_connect[PITCH])
		interge[0]=0;
		if(eepromConfig.rollEnabled||!bldc.en_code_connect[ROLL])
		interge[1]=0;
		if(eepromConfig.yawEnabled||!bldc.en_code_connect[YAW])
		interge[2]=0;
	for(i=0;i<3;i++)
	{
	  ero[i]=LIMIT(exp_angle[i]-attitude[i],-99,99);
		interge[i]+=ero[i]*dt*ki;
		if(bldc.en_code_connect[i])
		exp_rad[i]=LIMIT(ero[i]*kp+interge[i],-255,255);	
		else
	  exp_rad[i]=0;
		ero_reg[i]=ero[i];
	}
}	


float spd[3]={0,0,0},exp_rad[3],exp_angle[3];
float spd_out[3]={0},spd_out_flt[3]={0};
float flt=0.3;
void computeMotorCommands(float dt)
{
    holdIntegrators = false;//ÆôÓÃ»ý·Ö×÷ÓÃ

    ///////////////////////////////////
	
    position_control(dt);
	
    if (eepromConfig.rollEnabled == true)//Èç¹ûÊ¹ÄÜÁË ¹ö×ªÖá£¨ROLL£©
    {
//			//¸üÐÂPID£¬½á¹û·Åµ½pidCmd[ROLL]
//        pidCmd[ROLL] = updatePID(pointingCmd[ROLL] * mechanical2electricalDegrees[ROLL],
//                                 -sensors.margAttitude500Hz[ROLL] * mechanical2electricalDegrees[ROLL],
//                                 dt, holdIntegrators, 
//                                 &eepromConfig.PID[ROLL_PID]);//mechanical2electricalDegrees = motor poles/2   14poles/2=7
			 pidCmd[ROLL] = updatePID(exp_angle[ROLL] * mechanical2electricalDegrees[ROLL]*D2R,
                                 attitude[ROLL] * mechanical2electricalDegrees[ROLL]*D2R,
                                 dt, holdIntegrators, 
                                 &eepromConfig.PID[ROLL_PID]);//mechanical2electricalDegrees = motor poles/2   14poles/2=7
			//µ±Ç°PIDÊä³ö¼õÈ¥ÉÏ´ÎPIDÊä³öµÃµ½±ä»¯Á¿
        outputRate[ROLL] = pidCmd[ROLL] - pidCmdPrev[ROLL];

        if (outputRate[ROLL] > eepromConfig.rateLimit)//Èç¹û±ä»¯Á¿´óÓÚÐý×ªËÙ¶ÈÏÞÖÆµÄ×î´óÖµ
            pidCmd[ROLL] = pidCmdPrev[ROLL] + eepromConfig.rateLimit;//ÄÇÃ´Ê¹ÓÃÉÏ´ÎPIDÊä³ö½á¹û+Ðý×ªËÙ¶ÈÏÞÖÆµÄ×î´óÖµ ×÷Îª±¾´ÎPID½á¹û

        if (outputRate[ROLL] < -eepromConfig.rateLimit)//Èç¹û±ä»¯Á¿Ð¡ÓÚÐý×ªËÙ¶ÈÏÞÖÆµÄ×îÐ¡Öµ
            pidCmd[ROLL] = pidCmdPrev[ROLL] - eepromConfig.rateLimit;//ÄÇÃ´Ê¹ÓÃÉÏ´ÎPIDÊä³ö½á¹û-Ðý×ªËÙ¶ÈÏÞÖÆµÄ×îÐ¡Öµ£¨¼õÈ¥Ò»¸ö¸ºÊý
					//	Ïàµ±ÓÚ¼ÓÉÏÕâ¸öÊýµÄÕýÖµ£©£¬×÷Îª±¾´ÎPID½á¹û

        pidCmdPrev[ROLL] = pidCmd[ROLL];//±£´æ±¾´ÎPIDÊä³ö½á¹ûµ½ pidCmdPrev[ROLL] ×÷Îª¾ÉµÄÖµ£¨Ïà¶ÔÓÚÏÂ´Î£©¡£
         if(spd[ROLL]!=0)
					 spd_out[ROLL]-=dt*spd[ROLL];
				 else 
					 spd_out[ROLL]+=dt*exp_rad[ROLL];
         spd_out_flt[ROLL]= (spd_out[ROLL]*1+pidCmd[ROLL]*0)*flt+(1-flt)*spd_out_flt[ROLL];
         setRollMotor(spd_out_flt[ROLL], (int)eepromConfig.rollPower);// roll power is 50 in default 				 
			   //setRollMotor(pidCmd[ROLL], (int)eepromConfig.rollPower);// roll power is 50 in default 	
    }

    ///////////////////////////////////

    if (eepromConfig.pitchEnabled == true)//Èç¹ûÊ¹ÄÜÁË ¸©ÑöÖá£¨Pitch£©
    {
			//¸üÐÂPID£¬½á¹û·Åµ½pidCmd[PITCH]
        pidCmd[PITCH] = updatePID(pointingCmd[PITCH] * mechanical2electricalDegrees[PITCH],
                                  sensors.margAttitude500Hz[PITCH] * mechanical2electricalDegrees[PITCH],
                                  dt, holdIntegrators, &eepromConfig.PID[PITCH_PID]);

        outputRate[PITCH] = pidCmd[PITCH] - pidCmdPrev[PITCH];//µ±Ç°PIDÊä³ö¼õÈ¥ÉÏ´ÎPIDÊä³öµÃµ½±ä»¯Á¿

        if (outputRate[PITCH] > eepromConfig.rateLimit)//Èç¹û±ä»¯Á¿´óÓÚÐý×ªËÙ¶ÈÏÞÖÆµÄ×î´óÖµ
            pidCmd[PITCH] = pidCmdPrev[PITCH] + eepromConfig.rateLimit;//ÄÇÃ´Ê¹ÓÃÉÏ´ÎPIDÊä³ö½á¹û+Ðý×ªËÙ¶ÈÏÞÖÆµÄ×î´óÖµ ×÷Îª±¾´ÎPID½á¹û

        if (outputRate[PITCH] < -eepromConfig.rateLimit)//Èç¹û±ä»¯Á¿Ð¡ÓÚÐý×ªËÙ¶ÈÏÞÖÆµÄ×îÐ¡Öµ
            pidCmd[PITCH] = pidCmdPrev[PITCH] - eepromConfig.rateLimit;//ÄÇÃ´Ê¹ÓÃÉÏ´ÎPIDÊä³ö½á¹û-Ðý×ªËÙ¶ÈÏÞÖÆµÄ×îÐ¡Öµ£¨¼õÈ¥Ò»¸ö¸ºÊý
					//	Ïàµ±ÓÚ¼ÓÉÏÕâ¸öÊýµÄÕýÖµ£©£¬×÷Îª±¾´ÎPID½á¹û

        pidCmdPrev[PITCH] = pidCmd[PITCH];//±£´æ±¾´ÎPIDÊä³ö½á¹ûµ½ pidCmdPrev[PITCH] ×÷Îª¾ÉµÄÖµ£¨Ïà¶ÔÓÚÏÂ´Î£©¡£
    if(spd[PITCH]!=0)
					 spd_out[PITCH]-=dt*spd[PITCH];
				 else 
					 spd_out[PITCH]+=dt*exp_rad[PITCH];
         spd_out_flt[PITCH]= (spd_out[PITCH]*1+pidCmd[PITCH]*0)*flt+(1-flt)*spd_out_flt[PITCH];
         setPitchMotor(spd_out_flt[PITCH], (int)eepromConfig.pitchPower);// roll power is 50 in default 				 
			   //setPitchMotor(pidCmd[PITCH], (int)eepromConfig.pitchPower);// roll power is 50 in default 			
    }

    ///////////////////////////////////

    if (eepromConfig.yawEnabled == true)//Èç¹ûÊ¹ÄÜÁË Æ«º½Öá£¨YAW£©
    {
        if (eepromConfig.yawAutoPanEnabled == true)//Èç¹ûÊ¹ÄÜÁËÆ«º½Öá×Ô¶¯È«¾°
            yawCmd = autoPan(pidCmd[YAW], yawCmd * mechanical2electricalDegrees[YAW]) * electrical2mechanicalDegrees[YAW];
        else
            yawCmd = -pointingCmd[YAW];

				//¸üÐÂPID£¬½á¹û·Åµ½pidCmd[YAW]
        pidCmd[YAW] = updatePID(yawCmd * mechanical2electricalDegrees[YAW],
                                sensors.margAttitude500Hz[YAW] * mechanical2electricalDegrees[YAW],
                                dt, holdIntegrators, &eepromConfig.PID[YAW_PID]);

        outputRate[YAW] = pidCmd[YAW] - pidCmdPrev[YAW];//µ±Ç°PIDÊä³ö¼õÈ¥ÉÏ´ÎPIDÊä³öµÃµ½±ä»¯Á¿

        if (outputRate[YAW] > eepromConfig.rateLimit)//Èç¹û±ä»¯Á¿´óÓÚÐý×ªËÙ¶ÈÏÞÖÆµÄ×î´óÖµ
            pidCmd[YAW] = pidCmdPrev[YAW] + eepromConfig.rateLimit;//ÄÇÃ´Ê¹ÓÃÉÏ´ÎPIDÊä³ö½á¹û+Ðý×ªËÙ¶ÈÏÞÖÆµÄ×î´óÖµ ×÷Îª±¾´ÎPID½á¹û

        if (outputRate[YAW] < -eepromConfig.rateLimit)//Èç¹û±ä»¯Á¿Ð¡ÓÚÐý×ªËÙ¶ÈÏÞÖÆµÄ×îÐ¡Öµ
            pidCmd[YAW] = pidCmdPrev[YAW] - eepromConfig.rateLimit;//ÄÇÃ´Ê¹ÓÃÉÏ´ÎPIDÊä³ö½á¹û-Ðý×ªËÙ¶ÈÏÞÖÆµÄ×îÐ¡Öµ£¨¼õÈ¥Ò»¸ö¸ºÊý
					//	Ïàµ±ÓÚ¼ÓÉÏÕâ¸öÊýµÄÕýÖµ£©£¬×÷Îª±¾´ÎPID½á¹û

        pidCmdPrev[YAW] = pidCmd[YAW];//±£´æ±¾´ÎPIDÊä³ö½á¹ûµ½ pidCmdPrev[PITCH] ×÷Îª¾ÉµÄÖµ£¨Ïà¶ÔÓÚÏÂ´Î£©¡££
    if(spd[YAW]!=0)
					 spd_out[YAW]-=dt*spd[YAW];
				 else 
					 spd_out[YAW]+=dt*exp_rad[YAW];
         spd_out_flt[YAW]= (spd_out[YAW]*1+pidCmd[YAW]*0)*flt+(1-flt)*spd_out_flt[YAW];
         setYawMotor(spd_out_flt[YAW], (int)eepromConfig.yawPower);// roll power is 50 in default 				 
			   //setYawMotor(pidCmd[YAW], (int)eepromConfig.yawPower);// roll power is 50 in default 	
    }

    ///////////////////////////////////

}

///////////////////////////////////////////////////////////////////////////////
