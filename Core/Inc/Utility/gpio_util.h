/*
 *  Project      : Infantry_Momentum
 *
 *  file         : gpio_util.h
 *  Description  : This file contains the GPIO functions
 *  LastEditors  : ����ؼ���ᶯ��
 *  Date         : 2021-05-04 20:53:31
 *  LastEditTime : 2021-05-07 03:02:53
 */

#ifndef GPIO_UTIL_H
#define GPIO_UTIL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "configure.h"

#include "stm32f4xx_hal.h"

typedef struct {
    GPIO_TypeDef* gpio_handle;
    uint16_t gpio_pin;
} GPIO_GPIOTypeDef;

#if __FN_IF_ENABLE(__FN_INFANTRY_GIMBAL)

extern GPIO_GPIOTypeDef* LASER;
extern GPIO_GPIOTypeDef* BULLET_CHARGING;

#endif

#if __FN_IF_ENABLE(__FN_SUPER_CAP)

extern GPIO_GPIOTypeDef* BOOST;
extern GPIO_GPIOTypeDef* BUCK;
extern GPIO_GPIOTypeDef* CAP;

#endif

#if __FN_IF_ENABLE(__FN_INFANTRY_CHASSIS)

#endif

void GPIO_Open(GPIO_GPIOTypeDef* gpio);
void GPIO_Close(GPIO_GPIOTypeDef* gpio);

#ifdef __cplusplus
}
#endif

#endif
