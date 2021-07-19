/*
 *  Project      : Infantry_Momentum
 * 
 *  file         : filter_alg.h
 *  Description  : This file contains digital filter correlation function
 *  LastEditors  : 动情丶卜灬动心
 *  Date         : 2021-05-04 20:53:31
 *  LastEditTime : 2021-05-08 09:10:15
 */

#ifndef FILTER_ALG_H
#define FILTER_ALG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "math_alg.h"
#include "stm32f4xx_hal.h"

#define MAX_LENGTH 10

typedef struct {
    float cut_off_frq;
    float filt_para;
    float period;
} Filter_LowPassParamTypeDef;

typedef struct {
    float filted_val;
    float filted_last_val;
} Filter_LowPassTypeDef;

typedef struct {
    float val[MAX_LENGTH];
    float sum;
} Filter_WindowTypeDef;

typedef struct {
    double ybuf[4];
    double xbuf[4];
    float filted_val;
} Filter_Bessel_TypeDef;

void Filter_LowPassInit(float param, Filter_LowPassParamTypeDef* pparam);
float Filter_LowPass(float val, Filter_LowPassParamTypeDef* fparam, Filter_LowPassTypeDef* filt);
float Filter_Aver(float val, Filter_WindowTypeDef* filt);
float Filter_Bessel(float val, Filter_Bessel_TypeDef* filt);

#ifdef __cplusplus
}
#endif

#endif
