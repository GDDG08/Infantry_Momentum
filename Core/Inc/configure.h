/*
 *  Project      : Infantry_Momentum
 * 
 *  file         : configure.h
 *  Description  : This file contains all functions enable
 *  LastEditors  : 动情丶卜灬动心
 *  Date         : 2021-05-04 20:53:31
 *  LastEditTime : 2021-05-16 20:33:08
 */

// #if 你觉得这份代码写的很好
//      框架及部分代码由20赛季电控组长ckb设计及指导
// #else
//      这是syj瞎写的，别看了
// #endif

// Note:
// In order to avoid bus phenomenon
// if you want to change or add code function
// please operate according to the following code specification           
#include "Code_specification.h"
//        ↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑

#ifndef CONFIGURE_H
#define CONFIGURE_H

#ifdef __cplusplus
extern "C" {
#endif 

/* Main Control program       */

#define __FN_BOARD_TYPE             __FN_BOARD_TYPE_GIMBAL

#define __FN_BOARD_TYPE_CHASSIS     1
#define __FN_BOARD_TYPE_GIMBAL      2
#define __FN_BOARD_TYPE_SUPERCAP    3


#define __FN_INFANTRY_TYPE          2

#if (__FN_INFANTRY_TYPE == 1)
    #define __FN_INFANTRY_NEW_ONE       __FN_ENABLE
#endif

#if (__FN_INFANTRY_TYPE == 2)
    #define __FN_INFANTRY_NEW_TWO       __FN_ENABLE
#endif

#if (__FN_INFANTRY_TYPE == 3)
    #define __FN_INFANTRY_NEW_THREE     __FN_ENABLE
#endif



/*      **********************************************      */

#define __FN_ENABLE     1
#define __FN_DISABLE    0

#define __FN_IF_ENABLE(x) (x == __FN_ENABLE)

/* Detail function define     */

#if __FN_BOARD_TYPE == __FN_BOARD_TYPE_CHASSIS
    #define __FN_INFANTRY           __FN_ENABLE
    #define __FN_INFANTRY_CHASSIS   __FN_ENABLE
    #define __FN_SUPER_CAP_COMM     __FN_ENABLE
#else
    #define __FN_INFANTRY_CHASSIS   __FN_DISABLE
#endif

#if __FN_BOARD_TYPE == __FN_BOARD_TYPE_GIMBAL
    #define __FN_INFANTRY           __FN_ENABLE
    #define __FN_INFANTRY_GIMBAL    __FN_ENABLE
#else
    #define __FN_INFANTRY_GIMBAL    __FN_DISABLE
#endif

#if __FN_BOARD_TYPE == __FN_BOARD_TYPE_SUPERCAP
    #define __FN_INFANTRY           __FN_DISABLE
    #define __FN_SUPER_CAP          __FN_ENABLE
    #define __FN_SUPER_CAP_COMM     __FN_ENABLE
#else
    #define __FN_SUPER_CAP          __FN_DISABLE
#endif

/*  Control function          */

#if __FN_IF_ENABLE(__FN_SUPER_CAP)
    #define __FN_CTRL_CAP_COMM          __FN_ENABLE
    #define __FN_CTRL_CAP               __FN_ENABLE
    #define __FN_WATCHDOG_CAP           __FN_ENABLE
#endif

#if __FN_IF_ENABLE(__FN_INFANTRY_GIMBAL)
    #define __FN_CTRL_REMOTE            __FN_ENABLE
    #define __FN_CTRL_MINIPC            __FN_ENABLE
    #define __FN_CTRL_GIM_COMM          __FN_ENABLE
    #define __FN_CTRL_GIMBAL_GIM        __FN_ENABLE
    #define __FN_WATCHDOG_GIM           __FN_ENABLE
    #define __FN_CTRL_SHOOTER           __FN_ENABLE
#endif

#if __FN_IF_ENABLE(__FN_INFANTRY_CHASSIS)
    #define __FN_CTRL_CHASSIS           __FN_ENABLE
    #define __FN_CTRL_CHA_COMM          __FN_ENABLE
    #define __FN_CTRL_COM_CAP           __FN_ENABLE
    #define __FN_CTRL_POWER             __FN_ENABLE
    #define __FN_CTRL_GIMBAL_YAW_CHA    __FN_ENABLE
    #define __FN_WATCHDOG_CHA           __FN_ENABLE
    #define __FN_CTRL_REFEREE           __FN_ENABLE
#endif



/*      Infantry function enable    */
#if __FN_IF_ENABLE(__FN_INFANTRY)

        /* Base Utility Configuration */
    #define __FN_UTIL_CAN   __FN_ENABLE
    // Enable CAN BUS
    #define __FN_UTIL_UART  __FN_ENABLE
    // Enable serial port
    #define __FN_UTIL_PWM   __FN_ENABLE
    // Enable PWM
    #define __FN_UTIL_USB   __FN_DISABLE
    // Enable USB


        /* Peripheral Configuration */
    #define __FN_PERIPH_MOTOR   __FN_ENABLE
    // Enable Motor
    #if __FN_IF_ENABLE(__FN_INFANTRY_GIMBAL)
        #define __FN_PERIPH_IMU __FN_ENABLE
        // Enable IMU
    #endif
    #if __FN_IF_ENABLE(__FN_INFANTRY_GIMBAL)
        #define __FN_PERIPH_MINIPC __FN_ENABLE
        // Enable MiniPC
    #endif
    #if __FN_IF_ENABLE(__FN_INFANTRY_GIMBAL)
        #define __FN_PERIPH_REMOTE __FN_ENABLE
        // Enable remote
    #endif
    #if __FN_IF_ENABLE(__FN_INFANTRY_CHASSIS)
        #define __FN_PERIPH_REFEREE __FN_ENABLE
        // Enable referee
    #endif
    #define __FN_PERIPH_BEEPER __FN_ENABLE
    // Enable beeper
    #if __FN_IF_ENABLE(__FN_INFANTRY_GIMBAL)
        #define __FN_PERIPH_SERVO __FN_ENABLE
        // Enable servo
    #endif
    #define __FN_PERIPH_LED __FN_ENABLE
    // Enable LED

#endif


    /* Super Cap functions enable   */
#if __FN_IF_ENABLE(__FN_SUPER_CAP)

    #define __FN_UTIL_CAN   __FN_ENABLE
    // Enable CAN BUS
    #define __FN_UTIL_ADC   __FN_ENABLE
    // Enable ADC
    #define __FN_UTIL_UART  __FN_ENABLE
    // Enable UART
    #define __FN_UTIL_PWM   __FN_ENABLE
    // Enable PWM
    #define __FN_UTIL_DAC   __FN_ENABLE
    // Enable DAC
    #define __FN_PERIPH_SENSOR  __FN_ENABLE
    // Enable sensor
    #define __FN_PERIPH_LED __FN_ENABLE
    // Enable LED

#endif

#ifdef __cplusplus
}
#endif

#endif
