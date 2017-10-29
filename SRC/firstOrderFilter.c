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

// TAU = Filter Time Constant
// T   = Filter Sample Time

// A   = 2 * TAU / T

// Low Pass:
// GX1 = 1 / (1 + A)
// GX2 = 1 / (1 + A)
// GX3 = (1 - A) / (1 + A)

// High Pass:
// GX1 =  A / (1 + A)
// GX2 = -A / (1 + A)
// GX3 = (1 - A) / (1 + A)

///////////////////////////////////////////////////////////////////////////////

firstOrderFilterData_t firstOrderFilters[NUMBER_OF_FIRST_ORDER_FILTERS];

void initFirstOrderFilter(void)
{
    float a;

    a = 2.0f * eepromConfig.accelX500HzLowPassTau * 500.0f;

    firstOrderFilters[ACCEL_X_500HZ_LOWPASS].gx1 = 1.0f / (1.0f + a);
    firstOrderFilters[ACCEL_X_500HZ_LOWPASS].gx2 = 1.0f / (1.0f + a);
    firstOrderFilters[ACCEL_X_500HZ_LOWPASS].gx3 = (1.0f - a) / (1.0f + a);
    firstOrderFilters[ACCEL_X_500HZ_LOWPASS].previousInput  = 0.0f;
    firstOrderFilters[ACCEL_X_500HZ_LOWPASS].previousOutput = 0.0f;

    ///////////////////////////////////

    a = 2.0f * eepromConfig.accelY500HzLowPassTau * 500.0f;

    firstOrderFilters[ACCEL_Y_500HZ_LOWPASS].gx1 = 1.0f / (1.0f + a);
    firstOrderFilters[ACCEL_Y_500HZ_LOWPASS].gx2 = 1.0f / (1.0f + a);
    firstOrderFilters[ACCEL_Y_500HZ_LOWPASS].gx3 = (1.0f - a) / (1.0f + a);
    firstOrderFilters[ACCEL_Y_500HZ_LOWPASS].previousInput  = 0.0f;
    firstOrderFilters[ACCEL_Y_500HZ_LOWPASS].previousOutput = 0.0f;

    ///////////////////////////////////

    a = 2.0f * eepromConfig.accelZ500HzLowPassTau * 500.0f;

    firstOrderFilters[ACCEL_Z_500HZ_LOWPASS].gx1 = 1.0f / (1.0f + a);
    firstOrderFilters[ACCEL_Z_500HZ_LOWPASS].gx2 = 1.0f / (1.0f + a);
    firstOrderFilters[ACCEL_Z_500HZ_LOWPASS].gx3 = (1.0f - a) / (1.0f + a);
    firstOrderFilters[ACCEL_Z_500HZ_LOWPASS].previousInput  = -9.8065f;
    firstOrderFilters[ACCEL_Z_500HZ_LOWPASS].previousOutput = -9.8065f;

    ///////////////////////////////////

    a = 2.0f * eepromConfig.rollRatePointingCmd50HzLowPassTau * 50.0f;

    firstOrderFilters[ROLL_RATE_POINTING_50HZ_LOWPASS].gx1 = 1.0f / (1.0f + a);
    firstOrderFilters[ROLL_RATE_POINTING_50HZ_LOWPASS].gx2 = 1.0f / (1.0f + a);
    firstOrderFilters[ROLL_RATE_POINTING_50HZ_LOWPASS].gx3 = (1.0f - a) / (1.0f + a);
    firstOrderFilters[ROLL_RATE_POINTING_50HZ_LOWPASS].previousInput  = 0.0f;
    firstOrderFilters[ROLL_RATE_POINTING_50HZ_LOWPASS].previousOutput = 0.0f;

    ///////////////////////////////////

    a = 2.0f * eepromConfig.pitchRatePointingCmd50HzLowPassTau * 50.0f;

    firstOrderFilters[PITCH_RATE_POINTING_50HZ_LOWPASS].gx1 = 1.0f / (1.0f + a);
    firstOrderFilters[PITCH_RATE_POINTING_50HZ_LOWPASS].gx2 = 1.0f / (1.0f + a);
    firstOrderFilters[PITCH_RATE_POINTING_50HZ_LOWPASS].gx3 = (1.0f - a) / (1.0f + a);
    firstOrderFilters[PITCH_RATE_POINTING_50HZ_LOWPASS].previousInput  = 0.0f;
    firstOrderFilters[PITCH_RATE_POINTING_50HZ_LOWPASS].previousOutput = 0.0f;

    ///////////////////////////////////

    a = 2.0f * eepromConfig.yawRatePointingCmd50HzLowPassTau * 50.0f;

    firstOrderFilters[YAW_RATE_POINTING_50HZ_LOWPASS].gx1 = 1.0f / (1.0f + a);
    firstOrderFilters[YAW_RATE_POINTING_50HZ_LOWPASS].gx2 = 1.0f / (1.0f + a);
    firstOrderFilters[YAW_RATE_POINTING_50HZ_LOWPASS].gx3 = (1.0f - a) / (1.0f + a);
    firstOrderFilters[YAW_RATE_POINTING_50HZ_LOWPASS].previousInput  = 0.0f;
    firstOrderFilters[YAW_RATE_POINTING_50HZ_LOWPASS].previousOutput = 0.0f;

    ///////////////////////////////////

    a = 2.0f * eepromConfig.rollAttPointingCmd50HzLowPassTau * 50.0f;

    firstOrderFilters[ROLL_ATT_POINTING_50HZ_LOWPASS].gx1 = 1.0f / (1.0f + a);
    firstOrderFilters[ROLL_ATT_POINTING_50HZ_LOWPASS].gx2 = 1.0f / (1.0f + a);
    firstOrderFilters[ROLL_ATT_POINTING_50HZ_LOWPASS].gx3 = (1.0f - a) / (1.0f + a);
    firstOrderFilters[ROLL_ATT_POINTING_50HZ_LOWPASS].previousInput  = 0.0f;
    firstOrderFilters[ROLL_ATT_POINTING_50HZ_LOWPASS].previousOutput = 0.0f;

    ///////////////////////////////////

    a = 2.0f * eepromConfig.pitchAttPointingCmd50HzLowPassTau * 50.0f;

    firstOrderFilters[PITCH_ATT_POINTING_50HZ_LOWPASS].gx1 = 1.0f / (1.0f + a);
    firstOrderFilters[PITCH_ATT_POINTING_50HZ_LOWPASS].gx2 = 1.0f / (1.0f + a);
    firstOrderFilters[PITCH_ATT_POINTING_50HZ_LOWPASS].gx3 = (1.0f - a) / (1.0f + a);
    firstOrderFilters[PITCH_ATT_POINTING_50HZ_LOWPASS].previousInput  = 0.0f;
    firstOrderFilters[PITCH_ATT_POINTING_50HZ_LOWPASS].previousOutput = 0.0f;

    ///////////////////////////////////

    a = 2.0f * eepromConfig.yawAttPointingCmd50HzLowPassTau * 50.0f;

    firstOrderFilters[YAW_ATT_POINTING_50HZ_LOWPASS].gx1 = 1.0f / (1.0f + a);
    firstOrderFilters[YAW_ATT_POINTING_50HZ_LOWPASS].gx2 = 1.0f / (1.0f + a);
    firstOrderFilters[YAW_ATT_POINTING_50HZ_LOWPASS].gx3 = (1.0f - a) / (1.0f + a);
    firstOrderFilters[YAW_ATT_POINTING_50HZ_LOWPASS].previousInput  = 0.0f;
    firstOrderFilters[YAW_ATT_POINTING_50HZ_LOWPASS].previousOutput = 0.0f;

    ///////////////////////////////////

}

///////////////////////////////////////////////////////////////////////////////

float firstOrderFilter(float input, struct firstOrderFilterData *filterParameters)
{
    float output;

    output = filterParameters->gx1 * input +
             filterParameters->gx2 * filterParameters->previousInput -
             filterParameters->gx3 * filterParameters->previousOutput;

    filterParameters->previousInput  = input;
    filterParameters->previousOutput = output;

    return output;
}

///////////////////////////////////////////////////////////////////////////////




#define MED_WIDTH_NUM 20
#define MED_FIL_ITEM  10

float med_filter_tmp[MED_FIL_ITEM][MED_WIDTH_NUM ];
float med_filter_out[MED_FIL_ITEM];

u8 med_fil_cnt[MED_FIL_ITEM];
// 1  2  3                                9
float Moving_Median(u8 item,u8 width_num,float in)
{
	u8 i,j;
	float t;
	float tmp[MED_WIDTH_NUM];
	if(width_num==0)
	return in;
	else if(item >= MED_FIL_ITEM || width_num >= MED_WIDTH_NUM )
	{
		return 0;
	}
	else
	{
		if( ++med_fil_cnt[item] >= width_num )	
		{
			med_fil_cnt[item] = 0;
		}
		
		med_filter_tmp[item][ med_fil_cnt[item] ] = in;
		
		for(i=0;i<width_num;i++)
		{
			tmp[i] = med_filter_tmp[item][i];
		}
		
		for(i=0;i<width_num-1;i++)
		{
			for(j=0;j<(width_num-1-i);j++)
			{
				if(tmp[j] > tmp[j+1])
				{
					t = tmp[j];
					tmp[j] = tmp[j+1];
					tmp[j+1] = t;
				}
			}
		}

		
		return ( tmp[(u16)width_num/2] );
	}
}

///////////////////////////////////////////////////////////////////////////////


