/*
 *  Project      : Infantry_Momentum
 * 
 *  file         : math_alg.c
 *  Description  : This file contains the math calculate tools
 *  LastEditors  : 动情丶卜灬动心
 *  Date         : 2021-05-04 20:53:31
 *  LastEditTime : 2021-05-08 03:38:32
 */

#include "math_alg.h"

/**
  * @brief      Calculation differential (only two order)(To be improved)
  * @param      arr :point to be differential value
  * @param      order :The differential order
  * @retval     NULL
  */
float Math_Differential(float arr[], uint8_t order) {
    float value;
    switch (order) {
        case 1:
            value = arr[0] - arr[1];
            break;
        case 2:
            value = arr[2] - 2 * arr[1] + arr[0];
            break;
        default:
            value = arr[0];
            break;
    }
    return value;
}

/**
  * @brief      Initialize ramp function control parameters
  * @param      pparam: Pointer to ramp function control parameter
  * @param      kp: P factor
  * @param      ki: I factor
  * @param      kd: D factor
  * @param      sum_max: Integral limiting
  * @param      output_max: Output limiting
  * @retval     NULL
  */
void Math_InitSlopeParam(Math_SlopeParamTypeDef* pparam, float acc, float dec) {
    pparam->acc = acc;
    pparam->dec = dec;
}

/**
  * @brief      Calculate slope function setting
  * @param      rawref: Current setting value
  * @param      targetref: Target set point
  * @param      pparam: Pointer to ramp function control parameter
  * @retval     Slope function setting value. If slope function is not enabled (parameter is 0), the target setting value is returned
  */
float Math_CalcSlopeRef(float rawref, float targetref, Math_SlopeParamTypeDef* pparam) {
    float newref;
    if (pparam->acc == 0 | pparam->dec == 0)
        return targetref;
    if (rawref < targetref - pparam->acc) {
        newref = rawref + pparam->acc;
    } else if (rawref > targetref + pparam->dec) {
        newref = rawref - pparam->dec;
    } else {
        newref = targetref;
    }
    return newref;
}

/**
  * @brief      Calculate the absolute slope function setting value
  * @param      rawref: Current setting value
  * @param      targetref: Target set point
  * @param      pparam: Pointer to ramp function control parameter
  * @retval     Absolute value ramp function setting value. If ramp function is not enabled, the target setting value is returned
  */
float Math_CalcAbsSlopeRef(float rawref, float targetref, Math_SlopeParamTypeDef* pparam) {
    float newref;
    if (pparam->acc == 0 | pparam->dec == 0)
        return targetref;
    if (rawref > 0) {
        if (rawref < targetref - pparam->acc) {
            newref = rawref + pparam->acc;
        } else if (rawref > targetref + pparam->dec) {
            newref = rawref - pparam->dec;
        } else {
            newref = targetref;
        }
    } else {
        if (rawref > targetref + pparam->acc) {
            newref = rawref - pparam->acc;
        } else if (rawref < targetref - pparam->dec) {
            newref = rawref + pparam->dec;
        } else {
            newref = targetref;
        }
    }
    return newref;
}
