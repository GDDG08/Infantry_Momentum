/*
 *  Project      : Infantry_Momentum
 * 
 *  file         : dac_util.c
 *  Description  : This file contains the DAC functions
 *  LastEditors  : 动情丶卜灬动心
 *  Date         : 2021-05-04 20:53:31
 *  LastEditTime : 2021-05-07 02:52:29
 */

#include "dac_util.h"

#if __FN_IF_ENABLE(__FN_UTIL_DAC)

#include "const_lib.h"

DAC_DACHandleTypeDef CurrentDAC;
DAC_HandleTypeDef* Current_DAC_HANDLER = &hdac;

/**
  * @brief      DAC initialization
  * @param      NULL
  * @retval     NULL
  */
void DAC_Init() {
    //Initialization related parameters
    CurrentDAC.state = DAC_OFF;
    CurrentDAC.ch = DAC_CHANNEL_1;
    CurrentDAC.DAC_DecodeValue = 0;
    CurrentDAC.hdac = Current_DAC_HANDLER;
    CurrentDAC.value = 0;

    //Close DAC output after initialization
    HAL_DAC_Stop(CurrentDAC.hdac, CurrentDAC.ch);
}

/**
  * @brief      Turn on DAC and DMA
  * @param      value :Set current value(unit: A)
  * @retval     NULL
  */
void DAC_SetCurrent(float value) {
    //decoding
    CurrentDAC.value = value;
    DAC_DecodeValue();

    //Set dma and dac
    HAL_DAC_SetValue(CurrentDAC.hdac, CurrentDAC.ch, DAC_ALIGN_12B_R, CurrentDAC.DAC_DecodeValue);
    HAL_DAC_Start(CurrentDAC.hdac, CurrentDAC.ch);
    CurrentDAC.state = DAC_ON;
}

/**
  * @brief      Close DAC  
  * @param      NULL
  * @retval     NULL
  */
void DAC_StopDAC() {
    HAL_DAC_Stop(CurrentDAC.hdac, CurrentDAC.ch);
    HAL_DAC_Stop_DMA(CurrentDAC.hdac, CurrentDAC.ch);
    CurrentDAC.state = DAC_OFF;
}

/**
 * @brief      Calculate DAC set value
 * @param      NULL
 * @retval     NULL
 */
void DAC_DecodeValue() {
    float voltage = CurrentDAC.value * Const_DAC_DetectRES * Const_DAC_GAIN;
    if (voltage >= 1.1f) {
        voltage = 1.1f;
    }
    float decode = voltage * 4096 / 3.3f;
    CurrentDAC.DAC_DecodeValue = decode;
}

#endif
