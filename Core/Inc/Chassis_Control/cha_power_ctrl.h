/*
 * @Project      : RM_Infantry_Neptune_frame
 * @FilePath     : \GITEE2\Core\Inc\Chassis_Control\cha_power_ctrl.h
 * @Descripttion : 
 * @Author       : GDDG08
 * @Date         : 2021-07-24 11:39:13
 * @LastEditors  : GDDG08
 * @LastEditTime : 2021-10-14 20:08:29
 */
/*
 *  Project      : Infantry_Momentum
 * 
 *  file         : cha_power_ctrl.h
 *  Description  : This file contains power control function
 *  LastEditors  : ����ؼ���ᶯ��
 *  Date         : 2021-05-04 20:53:31
 *  LastEditTime : 2021-05-08 08:31:49
 */

#ifndef CHA_POWER_CTRL_H
#define CHA_POWER_CTRL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "configure.h"

#if __FN_IF_ENABLE(__FN_CTRL_POWER)

#include "motor_periph.h"
#include "referee_periph.h"
#include "math_alg.h"

typedef struct {
    float power_limit;
    float warning_power;
    float warning_power_buff;

    float no_judge_total_current_limit;
    float buffer_total_current_limit;
    float power_total_current_limit;

    float now_power;
    float now_power_buff;

    float total_current_limit;
    float total_current;
} Power_DataTypeDef;

typedef enum {
    POWER_UNLIMITED = 0X00,
    POWER_LIMIT = 0X01,
} Power_ControlModeMnum;

extern Power_DataTypeDef Power_data;
extern Power_ControlModeMnum Power_ControlMode;

void Power_InitPower(void);
Power_DataTypeDef* Power_GetPowerDataPty(void);
void Power_SetLimitPower(void);
void Power_ForceChangePowerMode(Power_ControlModeMnum mode);
void Power_ChangePowerMode(Power_ControlModeMnum mode);
void Power_PowerControl(Motor_MotorGroupTypeDef* chassis);

#ifdef __cplusplus
}
#endif

#endif

#endif
