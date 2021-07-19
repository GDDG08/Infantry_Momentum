/*
 *  Project      : Infantry_Momentum
 * 
 *  file         : dac_util.h
 *  Description  : This file contains the DAC functions
 *  LastEditors  : 动情丶卜灬动心
 *  Date         : 2021-05-04 20:53:31
 *  LastEditTime : 2021-05-08 10:55:27
 */

#ifndef DAC_UTIL_H
#define DAC_UTIL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "configure.h"

#if __FN_IF_ENABLE(__FN_UTIL_DAC)

#include "dac.h"

typedef enum {
    DAC_OFF = 0,
    DAC_ON = 1
} DAC_DACStateEnum;

typedef struct {
    DAC_DACStateEnum state;
    DAC_HandleTypeDef* hdac;
    uint32_t ch;
    float value;
    uint32_t DAC_DecodeValue;
} DAC_DACHandleTypeDef;

extern DAC_DACHandleTypeDef CurrentDAC;

void DAC_StopDAC(void);
void DAC_SetCurrent(float value);
void DAC_StopDAC(void);
void DAC_DecodeValue(void);
void DAC_Init(void);

#endif

#ifdef __cplusplus
}
#endif

#endif
