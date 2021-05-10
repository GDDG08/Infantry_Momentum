/*
 *  Project      : Infantry_Momentum
 * 
 *  file         : sensor_periph.c
 *  Description  : This file contains analog sensor function(Only for Super Cap)
 *  LastEditors  : 动情丶卜灬动心
 *  Date         : 2021-05-04 20:53:31
 *  LastEditTime : 2021-05-08 09:16:55
 */


#include "sensor_periph.h"

#if __FN_IF_ENABLE(__FN_PERIPH_SENSOR)

#include "const_lib.h"

Sen_CAPBasisValueTypeDef Sen_basisValue;
Sen_PowerValueTypeDef    Sen_powerValue;
Sen_FilterTypeDef Sen_Filter;

/**
  * @brief      Gets the pointer to the sensor power data object
  * @param      NULL
  * @retval     Pointer to sensor power data object
  */
Sen_PowerValueTypeDef* Sen_GetPowerDataPtr() {
    return &Sen_powerValue;
}


/**
  * @brief      Gets the pointer to the sensor basis data object
  * @param      NULL
  * @retval     Pointer to sensor basis data object
  */
Sen_CAPBasisValueTypeDef* Sen_GetBasisDataPtr() {
    return &Sen_basisValue;
}


/**
  * @brief      Sen initialization
  * @param      NULL
  * @retval     NULL
  */
void Sen_Init(){
    ADC_Init();
    Filter_LowPassInit(0.1, &Sen_Filter.CapVoltageParam);
    Filter_LowPassInit(0.1, &Sen_Filter.ChasissVoltageParam);
    Filter_LowPassInit(0.1, &Sen_Filter.VccVoltageParam);
}


/**
  * @brief      Filtering data
  * @param      Sen_Filter: Filter structure
  * @param      Sen_basisValue: Filter data
  * @retval     NULL
  */
void Sen_Filtering(Sen_FilterTypeDef *Sen_Filter, Sen_CAPBasisValueTypeDef *Sen_basisValue) {
      //ADC Filter
    Sen_basisValue->CapVoltage       = Filter_LowPass(Sen_basisValue->CapVoltage,    &Sen_Filter->CapVoltageParam, &Sen_Filter->CapVoltage);
    Sen_basisValue->VccVoltage       = Filter_LowPass(Sen_basisValue->VccVoltage,    &Sen_Filter->VccVoltageParam, &Sen_Filter->VccVoltage);
    Sen_basisValue->ChasissVoltage   = Filter_LowPass(Sen_basisValue->ChasissVoltage,&Sen_Filter->ChasissVoltageParam, &Sen_Filter->ChasissVoltage);

}


/**
  * @brief      Decode Sen data
  * @param      NULL
  * @retval     NULL
  */
void Sensor_Decode() {
    Sen_CAPBasisValueTypeDef *basisValue = Sen_GetBasisDataPtr();
    Sen_PowerValueTypeDef *powerValue = Sen_GetPowerDataPtr();

    ADC_GetData();
    ADC_Decode();

    /*ADC_decodeBuf What a fool to deal with */
    basisValue->VccVoltage     	  = ADC_decodeBuf[0] * Const_ADC_V_VGAIN;
    basisValue->ChasissVoltage    = ADC_decodeBuf[1] * Const_ADC_V_VGAIN;
    basisValue->CapVoltage     	  = ADC_decodeBuf[2] * Const_ADC_V_VGAIN;
    
    Sen_Filtering(&Sen_Filter, basisValue);

    powerValue->CapRestEnergy     = 0.5f * Const_ADC_CapValue * basisValue->CapVoltage * basisValue->CapVoltage;
    powerValue->CapPercent   	    = (uint8_t)(powerValue->CapRestEnergy * 100 / Const_ADC_Cap_TotalEnergy);
}


#endif
