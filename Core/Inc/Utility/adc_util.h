/*
 *  Project      : Infantry_Momentum
 *
 *  file         : adc_util.h
 *  Description  : This file contains the ADC functions
 *  LastEditors  : ����ؼ���ᶯ��
 *  Date         : 2021-05-04 20:53:31
 *  LastEditTime : 2021-05-07 02:48:40
 */

#ifndef ADC_UTIL_H
#define ADC_UTIL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "configure.h"

#if __FN_IF_ENABLE(__FN_UTIL_ADC)

#include "adc.h"
#include "string.h"

extern uint32_t ADC_valueBuf[30];
extern float ADC_decodeBuf[30];

void ADC_Init(void);
void ADC_GetData(void);
void ADC_Decode(void);

#endif

#ifdef __cplusplus
}
#endif

#endif
