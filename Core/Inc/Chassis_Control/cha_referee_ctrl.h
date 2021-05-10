/*
 *  Project      : Infantry_Momentum
 * 
 *  file         : 
 *  Description  : 
 *  LastEditors  : 动情丶卜灬动心
 *  Date         : 2021-05-04 20:53:31
 *  LastEditTime : 2021-05-08 07:53:36
 */
/**
 * BattleSpirit Framework Header File
 * 
 * File:        draw_ctrl.h
 * Brief:       裁判系统自定义UI绘制
 * Author:      Chen Kangbing
 * Modified:    2021/4/30 16:25:24
 *
 */


#ifndef DRAW_CTRL_H
#define DRAW_CTRL_H

#ifdef __cplusplus
extern "C" {
#endif 


#include "stm32f4xx_hal.h"


typedef struct {
    uint8_t width_mode, width_mode_last;    // 1 for gyro mode, 0 for normal mode
    uint8_t cap_state;                      // cap percent, 0 ~ 100
    float pitch_angle;
} DrawCtrl_DrawCtrlDataTypeDef;


void DrawCtrl_SetWidthMode(uint8_t mode);
void DrawCtrl_SetCapState(uint8_t state);
void DrawCtrl_SetPitchAngle(float angle);

void DrawCtrl_SetupAimLine(void);
void DrawCtrl_UpdateAimLine(void);
void DrawCtrl_SetupCrosshair(void);
void DrawCtrl_UpdateCrosshair(void);
void DrawCtrl_SetupWidthMark(void);
void DrawCtrl_UpdateWidthMark(void);
void DrawCtrl_SetupCapState(void);
void DrawCtrl_UpdateCapState(void);
void DrawCtrl_SetupPitchMeter(void);
void DrawCtrl_UpdatePitchMeter(void);
void DrawCtrl_SetupModeDisplay(void);
void DrawCtrl_UpdateModeDisplay(void);
void DrawCtrl_SetupErrorDisplay(void);
void DrawCtrl_UpdateErrorDisplay(void);
void DrawCtrl_SetupAllString(void);
void DrawCtrl_Setup(void);
void DrawCtrl_Update(void);


#ifdef __cplusplus
}
#endif

#endif
