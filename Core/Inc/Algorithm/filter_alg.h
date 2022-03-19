/*
 * @Project      : RM_Infantry_Neptune_frame
 * @FilePath     : \GITEE2\Core\Inc\Algorithm\filter_alg.h
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2021-07-24 11:39:13
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-03-19 16:46:01
 */
/*
 *  Project      : Infantry_Momentum
 *
 *  file         : filter_alg.h
 *  Description  : This file contains digital filter correlation function
 *  LastEditors  : ����ؼ���ᶯ��
 *  Date         : 2021-05-04 20:53:31
 *  LastEditTime : 2021-05-08 09:10:15
 */

#ifndef FILTER_ALG_H
#define FILTER_ALG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "math_alg.h"

#define MAX_LENGTH 30

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
    uint8_t length;
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
void Filter_AverInit(Filter_WindowTypeDef* filt, uint8_t length);
float Filter_Aver(float val, Filter_WindowTypeDef* filt);
float Filter_Bessel(float val, Filter_Bessel_TypeDef* filt);

#ifdef __cplusplus
}
#endif

#endif
