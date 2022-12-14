/*
 *  Project      : Infantry_Momentum
 *
 *  file         : gim_miniPC_ctrl.h
 *  Description  : This file contains MiniPC control function
 *  LastEditors  : ����ؼ���ᶯ��
 *  Date         : 2021-05-04 20:53:31
 *  LastEditTime : 2021-05-16 20:01:45
 */

#ifndef GIM_MINIPC_CTRL_H
#define GIM_MINIPC_CTRL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "configure.h"

#if __FN_IF_ENABLE(__FN_CTRL_MINIPC)

#include "minipc_periph.h"
#include "math_alg.h"
#include "filter_alg.h"
#include "kalman_alg.h"

typedef enum {
    MiniPC_TARGET_LOST = 0u,
    MiniPC_TARGET_FOLLOWING = 1u
} MiniPC_TargetFollowModeEnum;

typedef enum {
    MiniPC_ARMOR = 0u,
    MiniPC_BIG_BUFF = 1u,
    MiniPC_SMALL_BUFF = 2u
} MiniPC_AutoAimModeEnum;

typedef struct {
    uint8_t enable_aim_output;

    float distance;
    float yaw_angle;
    float pitch_angle;

    float yaw_ref_filtered, pitch_ref_filtered;
    float yaw_cvkf_filtered;
    float distance_filtered;

    Filter_LowPassParamTypeDef yaw_fil_param;
    Filter_LowPassTypeDef yaw_fil;

    Filter_LowPassParamTypeDef pitch_fil_param;
    Filter_LowPassTypeDef pitch_fil;

    Filter_LowPassParamTypeDef yaw_cvkf_fil_param;
    Filter_LowPassTypeDef yaw_cvkf_fil;

    Filter_LowPassParamTypeDef distance_fil_param;
    Filter_LowPassTypeDef distance_fil;

    Kalman_CVKalmanInitDataTypeDef cvkf_data_yaw, cvkf_data_pitch;  // For CVKF:
    Kalman_CVKalmanTypeDef cvkf_yaw, cvkf_pitch;
    Kalman_CVKalmanControlTypeDef cvkf_control;  // For Debug: Start Which Function

    uint32_t get_target_time;
    MiniPC_AutoAimModeEnum aim_mode;
    MiniPC_TargetFollowModeEnum target_state;
} MiniPC_MiniPCContrlTypeDef;

MiniPC_MiniPCContrlTypeDef* MiniPC_GetMiniPCControlDataPtr(void);
MiniPC_MiniPCContrlTypeDef* MiniPC_GetMiniPCDecodeDataPtr(void);
void MiniPC_InitControl(void);
void MiniPC_ChangeAimMode(MiniPC_AutoAimModeEnum mode);
void MiniPC_CalcAutoAim(void);
void MiniPC_UpdateAutoAim(void);
void MiniPC_SetFollowMode(MiniPC_TargetFollowModeEnum mode);
void MiniPC_SetTargetFollowMode(void);
void MiniPC_KalmanPrediction(void);
void MiniPC_UpdateControlData(void);
void MiniPC_SetGimbalRef(void);

#endif

#ifdef __cplusplus
}
#endif

#endif
