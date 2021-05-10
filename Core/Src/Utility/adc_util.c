/*
 *  Project      : Infantry_Momentum
 * 
 *  file         : adc.c
 *  Description  : This file contains the ADC functions (Only for Super Cap and)
 *  LastEditors  : 动情丶卜灬动心
 *  Date         : 2021-05-04 20:53:31
 *  LastEditTime : 2021-05-07 02:51:08
 */


#include "adc_util.h"

#if __FN_IF_ENABLE(__FN_UTIL_ADC)

#include "const_lib.h"


uint32_t ADC_valueBuf[3];                //ADC data array    
float    ADC_decodeBuf[3];		        //ADC decode data

/**
  * @brief      ADC peripheral initialization
  * @param      NULL
  * @retval     NULL
  */
void ADC_Init() {
//		HAL_ADCEx_Calibration_Start(&hadc1);						//ADC calibration
		HAL_ADC_Start_DMA(&hadc1,(uint32_t*)&ADC_valueBuf, 3);		//start ADC DMA,Get the first group data.
}


/**
  * @brief      Get ADC data
  * @param      hadc1 : adc handle
  * @param      ADC_valueBuf : adc_value array
  * @retval     NULL
  */
void ADC_GetData() {
		HAL_ADC_Start_DMA(&hadc1,(uint32_t*)&ADC_valueBuf, 3);
}


/**
  * @brief      Decode ADC data
  * @param      NULL
  * @retval     NULL
  */
void ADC_Decode() {
    memset(ADC_decodeBuf, 0, sizeof(ADC_decodeBuf));
    for (int j = 0; j <= 3; j++)
    ADC_decodeBuf[j] = (float)ADC_valueBuf[j] / 4096.0f * 3.3f ;               //adc decode 3.3V AVCC
}


#endif
