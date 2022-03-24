/*
 *  Project      : Infantry_Momentum
 *
 *  file         : gpio_util.c
 *  Description  : This file contains the GPIO functions
 *  LastEditors  : ����ؼ���ᶯ��
 *  Date         : 2021-05-04 20:53:31
 *  LastEditTime : 2021-05-07 03:02:10
 */

#include "gpio_util.h"

#if __FN_IF_ENABLE(__FN_INFANTRY_GIMBAL)

GPIO_GPIOTypeDef LASER_START = {GPIOB, GPIO_PIN_15};
GPIO_GPIOTypeDef BULLET_CHARGING_START = {GPIOB, GPIO_PIN_14};

GPIO_GPIOTypeDef* LASER = &LASER_START;
GPIO_GPIOTypeDef* BULLET_CHARGING = &BULLET_CHARGING_START;

#endif

#if __FN_IF_ENABLE(__FN_SUPER_CAP)

GPIO_GPIOTypeDef BOOST_START = {GPIOC, GPIO_PIN_4};
GPIO_GPIOTypeDef BUCK_START = {GPIOB, GPIO_PIN_3};
GPIO_GPIOTypeDef CAP_START = {GPIOC, GPIO_PIN_5};

GPIO_GPIOTypeDef* BOOST = &BOOST_START;
GPIO_GPIOTypeDef* BUCK = &BUCK_START;
GPIO_GPIOTypeDef* CAP = &CAP_START;

#endif

#if __FN_IF_ENABLE(__FN_INFANTRY_CHASSIS)

#endif

/**
 * @brief      Set GPIO
 * @param      gpio :Mark of peripheral
 * @retval     NULL
 */
void GPIO_Open(GPIO_GPIOTypeDef* gpio) {
    HAL_GPIO_WritePin(gpio->gpio_handle, gpio->gpio_pin, GPIO_PIN_SET);
}

/**
 * @brief      Reset GPIO
 * @param      gpio :Mark of peripheral
 * @retval     NULL
 */
void GPIO_Close(GPIO_GPIOTypeDef* gpio) {
    HAL_GPIO_WritePin(gpio->gpio_handle, gpio->gpio_pin, GPIO_PIN_RESET);
}
