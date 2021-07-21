/*
 *  Project      : Infantry_Momentum
 * 
 *  file         : kalman_alg.h
 *  Description  : This file contains the kalman filter algorithm
 *  LastEditors  : ???????
 *  Date         : 2021-05-04 20:53:31
 *  LastEditTime : 2021-05-16 20:05:58
 */

#ifndef KALMAN_ALG_H
#define KALMAN_ALG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "math_alg.h"

typedef struct {
    // KF Filter Parameters:
    mat KF_A, KF_C;
    mat XLast, Xpre, Xopt;
    mat PLast, Ppre, Popt;
    mat Q, R;
    mat Kf;

    // Cycle Time
    //float cvkf_t;
    float angle, angle_p_err, max_speed, min_speed;
    int switch_mode,
        measure_mode,
        targer_change;
} Kalman_CVKalmanTypeDef;

typedef struct {
    float KF_A[4], KF_C[2];
    float XLast[2], Xpre[2], Xopt[2];
    float PLast[4], Ppre[4], Popt[4];
    float Q[4], R[1];
    float Kf[2];
    //float cvkf_t;
} Kalman_CVKalmanInitDataTypeDef;

typedef struct {
    int total;         //???:????????
    int basicprocess;  //CVKF??????(????)
    int predict;       //??????????????
    int limit;         //?????????????
    int jumpjudge;     //????????
    int offset;
    int output;                 //??????????
    int lowfilter;              //?????????????
    int dead_domain_delta_ref;  //?????????????

} Kalman_CVKalmanControlTypeDef;

//CVKF Inner Function:
void Kalman_CVKalmanInitYawParam(Kalman_CVKalmanInitDataTypeDef* cvkf_data, float KF_T, float init_angle_yaw, float init_angle_speed);
//void Kalman_CVKalmanInitYawParam(Kalman_CVKalmanInitDataTypeDef *cvkf_data, float KF_T, float init_angle_yaw);
void Kalman_CVKalmanInitPitchParam(Kalman_CVKalmanInitDataTypeDef* cvkf_data, float KF_T, float init_angle_pitch, float init_angle_speed);
//void Kalman_CVKalmanInitPitchParam(Kalman_CVKalmanInitDataTypeDef *cvkf_data, float KF_T, float init_angle_pitch);
void Kalman_CVInitSetYaw(Kalman_CVKalmanInitDataTypeDef* cvkf_data, float init_angle_yaw, float init_angle_speed);
void Kalman_CVInitSetPitch(Kalman_CVKalmanInitDataTypeDef* cvkf_data, float init_angle_pitch, float init_angle_speed);
void Kalman_CVKalmanInit(Kalman_CVKalmanTypeDef* cvkf, Kalman_CVKalmanInitDataTypeDef* cvkf_data);
void Kalman_CV_Limit_Speed(Kalman_CVKalmanTypeDef* cvkf);
float Kalman_CV_CalInitSpeed(float delta_err_angle);

void Kalman_TurnOffCVKF(Kalman_CVKalmanTypeDef* cvkf);
void Kalman_TurnOnCVKF(Kalman_CVKalmanTypeDef* cvkf);
void Kalman_TurnOnMeasureUpdate(Kalman_CVKalmanTypeDef* cvkf);
void Kalman_CalcPredict(Kalman_CVKalmanTypeDef* cvkf);
void Kalman_CalcKFGain(Kalman_CVKalmanTypeDef* cvkf);
void Kalman_CalcCorrect(Kalman_CVKalmanTypeDef* cvkf, float angle);
void Kalman_Update(Kalman_CVKalmanTypeDef* cvkf);
void Kalman_MeasurementCalc(Kalman_CVKalmanTypeDef* cvkf, float angle);
void Kalman_NonMeasurementCalc(Kalman_CVKalmanTypeDef* cvkf);
float Kalman_Predict_nT(Kalman_CVKalmanTypeDef* cvkf, int nT);

float Kalman_JudgeChange(Kalman_CVKalmanTypeDef* cvkf, float m_angle);

#ifdef __cplusplus
}
#endif

#endif
