/*
 * EX15_XT.h
 *
 *  Created on: 2 de jul de 2019
 *      Author: Rinaldo Dos Santos
 *      Sinteck Next
 */

#ifndef MODELS_EX15_XT_H_
#define MODELS_EX15_XT_H_

#include "stm32g0xx_hal.h"
#include "stdio.h"
#include "string.h"

extern float forward, reflected;
extern uint32_t fwd, swr;

#define PA_FWD_POW_MAX_MEASURABLE_MILIVOLTS (float)(3030.0)
#define PA_FWD_POW_TABLE_EXPAND_COEF        10
#define PA_FWD_POW_TABLE_LENGTH             16
#define PA_FWD_POW_TABLE2_LENGTH            (PA_FWD_POW_TABLE_LENGTH-1)*PA_FWD_POW_TABLE_EXPAND_COEF+1 //LENGTH OF INTERPOLATED VECTOR

const float PA_FWD_TABLE_WATTS[PA_FWD_POW_TABLE_LENGTH][2] = {
    {0    , 0}, //this line was inserted only for test
	{86.00, 1},
	{330.0, 2},
	{675.0, 3},
    {930.0, 4},
    {1280.0, 5},
    {1440.0, 6},
    {1620.0, 7},
    {1870.0, 8},
    {2030.0, 9},
    {2270.0, 10},
    {2420.0, 11},
    {2570.0, 12},
    {2710.0, 13},
    {2910.0, 14},
    {3030.0, 15}
};
float PA_FWD_TABLE_WATTS2[PA_FWD_POW_TABLE2_LENGTH][2]; //generated table

void PA_FWD_TABLE_Interpolation()
{
	uint32_t pwdTableIndex = 0;
	uint32_t pwdTableIndex2 = 0;
	memset(PA_FWD_TABLE_WATTS2, 0, sizeof(PA_FWD_TABLE_WATTS2));
	float pwdTableDelta[2];

	for(pwdTableIndex = 0; pwdTableIndex <= PA_FWD_POW_TABLE_LENGTH - 2; pwdTableIndex++){
		pwdTableDelta[0] = (float)(PA_FWD_TABLE_WATTS[pwdTableIndex+1][0] - PA_FWD_TABLE_WATTS[pwdTableIndex][0]) / (float)(PA_FWD_POW_TABLE_EXPAND_COEF);
		pwdTableDelta[1] = (float)(PA_FWD_TABLE_WATTS[pwdTableIndex+1][1] - PA_FWD_TABLE_WATTS[pwdTableIndex][1]) / (float)(PA_FWD_POW_TABLE_EXPAND_COEF);

		for(pwdTableIndex2 = 0; pwdTableIndex2 <= PA_FWD_POW_TABLE_EXPAND_COEF  - 1; pwdTableIndex2++){
			PA_FWD_TABLE_WATTS2[pwdTableIndex*PA_FWD_POW_TABLE_EXPAND_COEF + pwdTableIndex2][0] = PA_FWD_TABLE_WATTS[pwdTableIndex][0] + pwdTableIndex2 * pwdTableDelta[0];
			PA_FWD_TABLE_WATTS2[pwdTableIndex*PA_FWD_POW_TABLE_EXPAND_COEF + pwdTableIndex2][1] = PA_FWD_TABLE_WATTS[pwdTableIndex][1] + pwdTableIndex2 * pwdTableDelta[1];
        }
	}
	PA_FWD_TABLE_WATTS2[PA_FWD_POW_TABLE2_LENGTH-1][0] = PA_FWD_TABLE_WATTS[PA_FWD_POW_TABLE_LENGTH-1][0];
	PA_FWD_TABLE_WATTS2[PA_FWD_POW_TABLE2_LENGTH-1][1] = PA_FWD_TABLE_WATTS[PA_FWD_POW_TABLE_LENGTH-1][1];
}


//Table autogenerator reverse power
#define PA_REV_POW_MAX_MEASURABLE_MILIVOLTS (float)(2500.0)
#define PA_REV_POW_TABLE_EXPAND_COEF        10
#define PA_REV_POW_TABLE_LENGTH             3
#define PA_REV_POW_TABLE2_LENGTH            (PA_REV_POW_TABLE_LENGTH-1)*PA_REV_POW_TABLE_EXPAND_COEF+1 //LENGTH OF INTERPOLATED VECTOR
const float PA_REV_TABLE_WATTS[PA_REV_POW_TABLE_LENGTH][2] = {
  {0, 0},
//  {700.0, 0.5}/
  {1400.0, 1},
// {2000.0, 1.5},
  {2500.0, 2}
//{3000.0, 2.5},;
//{3300.0, 3.0};
};
float PA_REV_TABLE_WATTS2[PA_REV_POW_TABLE2_LENGTH][2]; //generated table

void PA_REV_TABLE_Interpolation()
{
	unsigned int pwdTableIndex = 0;
	unsigned int pwdTableIndex2 = 0;
	memset(PA_REV_TABLE_WATTS2, 0, sizeof(PA_REV_TABLE_WATTS2));
	float pwdTableDelta[2];

	for(pwdTableIndex = 0; pwdTableIndex <= PA_REV_POW_TABLE_LENGTH -2; pwdTableIndex++){
		pwdTableDelta[0] = (float)(PA_REV_TABLE_WATTS[pwdTableIndex+1][0] - PA_REV_TABLE_WATTS[pwdTableIndex][0]) / (float)(PA_REV_POW_TABLE_EXPAND_COEF);
		pwdTableDelta[1] = (float)(PA_REV_TABLE_WATTS[pwdTableIndex+1][1] - PA_REV_TABLE_WATTS[pwdTableIndex][1]) / (float)(PA_REV_POW_TABLE_EXPAND_COEF);

		for(pwdTableIndex2 = 0; pwdTableIndex2 <= PA_REV_POW_TABLE_EXPAND_COEF  -1   ; pwdTableIndex2++){
			PA_REV_TABLE_WATTS2[pwdTableIndex*PA_REV_POW_TABLE_EXPAND_COEF + pwdTableIndex2][0] = PA_REV_TABLE_WATTS[pwdTableIndex][0] + pwdTableIndex2 * pwdTableDelta[0];
			PA_REV_TABLE_WATTS2[pwdTableIndex*PA_REV_POW_TABLE_EXPAND_COEF + pwdTableIndex2][1] = PA_REV_TABLE_WATTS[pwdTableIndex][1] + pwdTableIndex2 * pwdTableDelta[1];
        }
	}
	PA_REV_TABLE_WATTS2[PA_REV_POW_TABLE2_LENGTH-1][0] = PA_REV_TABLE_WATTS[PA_REV_POW_TABLE_LENGTH-1][0];
	PA_REV_TABLE_WATTS2[PA_REV_POW_TABLE2_LENGTH-1][1] = PA_REV_TABLE_WATTS[PA_REV_POW_TABLE_LENGTH-1][1];
}

void ADC_MeasurementCorrection(void)
{
    uint32_t i = 0;

    //Approximate PA's Forward Power
    for (i = 0; i< PA_FWD_POW_TABLE2_LENGTH; i++){
        if( ((float)fwd * (float)(3.30/4095.0) * 1000 ) >= PA_FWD_TABLE_WATTS2[i][0]) { 	//compare millivolts
            forward = PA_FWD_TABLE_WATTS2[i][1];            //watts
        }
    }

    //Approximate PA's Reverse Power
   for (i = 0; i< PA_REV_POW_TABLE2_LENGTH; i++){
       if( ((float)swr * (float)(3.30/4095.0) * 1000)  >= PA_REV_TABLE_WATTS2[i][0])  { 	//compare millivolts
            reflected = PA_REV_TABLE_WATTS2[i][1];			//watts
      }
   }
}

#endif /* MODELS_EX15_XT_H_ */
