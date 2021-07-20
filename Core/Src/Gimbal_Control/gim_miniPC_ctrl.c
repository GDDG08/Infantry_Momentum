/*
 *  Project      : Infantry_Momentum
 * 
 *  file         : gim_miniPC_ctrl.c
 *  Description  : This file contains MiniPC control function
 *  LastEditors  : ???????
 *  Date         : 2021-05-04 20:53:31
 *  LastEditTime : 2021-05-16 20:32:45
 */

#include "gim_miniPC_ctrl.h"

#if __FN_IF_ENABLE(__FN_CTRL_MINIPC)

#include "buscomm_ctrl.h"
#include "const_lib.h"
#include "gim_gimbal_ctrl.h"
#include "gim_shoot_ctrl.h"
#include "math_alg.h"

MiniPC_MiniPCContrlTypeDef MiniPC_MiniPCContrlData;

#if (__FN_INFANTRY_TYPE == 4)
int CVKF_NT_YAW = 100;
#endif
#if (__FN_INFANTRY_TYPE == 3)
int CVKF_NT_YAW = 100;
#endif
#if (__FN_INFANTRY_TYPE == 5)
int CVKF_NT_YAW = 100;
#endif

int CVKF_NT_PITCH = 7;

float before_cvkf_yaw = 0.0f;
float before_cvkf_pitch = 0.0f;
float after_predict_yaw = 0.0f;
float after_predict_pitch = 0.0f;

float autoaim_pitch_offset = -3.0f;
float autoaim_yaw_offset = 0.0f;
float autoaim_pitch_dead = 0.05f;
float autoaim_yaw_dead = 0.05f;
float autoaim_pitch_limit = 5.0f;
float autoaim_yaw_limit = 10.0f;

float energy_yaw_offset = 0.9f;
float energy_pitch_offset = 0.3f;

/**
  * @brief      Gets the pointer to the MiniPC data object
  * @param      NULL
  * @retval     Pointer to MiniPC data object
  */
MiniPC_MiniPCContrlTypeDef* MiniPC_GetMiniPCControlDataPtr() {
    return &MiniPC_MiniPCContrlData;
}

/**
* @brief      Init minipc data
* @param      NULL
* @retval     NULL
*/
void MiniPC_InitControl() {
    MiniPC_MiniPCContrlTypeDef* minipc = MiniPC_GetMiniPCControlDataPtr();

    minipc->enable_aim_output = 1;

    Filter_LowPassInit(0.5, &minipc->yaw_fil_param);
    Filter_LowPassInit(0.5, &minipc->pitch_fil_param);

    //CVKF Init Variables:
    minipc->cvkf_control.total = 1;
    minipc->cvkf_control.basicprocess = 1;
    minipc->cvkf_control.jumpjudge = 0;  //no function
    minipc->cvkf_control.limit = 1;
    minipc->cvkf_control.output = 1;
    minipc->cvkf_control.predict = 1;
    minipc->cvkf_control.lowfilter = 1;
    minipc->cvkf_control.dead_domain_delta_ref = 1;
    minipc->cvkf_control.offset = 1;

    //CVKF for Yaw Angle:
    Kalman_CVKalmanInitYawParam(&minipc->cvkf_data_yaw, 1 / 1000.0f, 0.0f, 0.0f);
    Kalman_CVKalmanInit(&minipc->cvkf_yaw, &minipc->cvkf_data_yaw);
    //CVKF for Pitch Angle:
    Kalman_CVKalmanInitPitchParam(&minipc->cvkf_data_pitch, 1 / 1000.0f, 0.0f, 0.0f);
    Kalman_CVKalmanInit(&minipc->cvkf_pitch, &minipc->cvkf_data_pitch);
}

/**
* @brief      Change aiming mode
* @param      mode: MiniPC aim mode enum
* @retval     NULL
*/
void MiniPC_ChangeAimMode(MiniPC_AutoAimModeEnum mode) {
    MiniPC_MiniPCContrlTypeDef* minipc = MiniPC_GetMiniPCControlDataPtr();
    MiniPC_MiniPCDataTypeDef* minipc_data = MiniPC_GetMiniPCDataPtr();

    minipc->aim_mode = mode;
    minipc_data->mode = minipc->aim_mode;
}

/**
* @brief      MiniPC auto aim decode control
* @param      NULL
* @retval     NULL
*/
void MiniPC_CalcAutoAim() {
    Gimbal_GimbalTypeDef* gimbal = Gimbal_GetGimbalControlPtr();
    MiniPC_MiniPCContrlTypeDef* minipc = MiniPC_GetMiniPCControlDataPtr();

    MiniPC_SetTargetFollowMode();
    MiniPC_SetGimbalRef();
}

/**
* @brief      MiniPC auto aim decode control
* @param      NULL
* @retval     NULL
*/
void MiniPC_UpdateAutoAim() {
    MiniPC_UpdateControlData();
}

/**
* @brief      Set gimbal following mode
* @param      mode: MiniPC target follow mode enum
* @retval     NULL
*/
void MiniPC_SetFollowMode(MiniPC_TargetFollowModeEnum mode) {
    MiniPC_MiniPCContrlTypeDef* minipc = MiniPC_GetMiniPCControlDataPtr();
    minipc->target_state = mode;
}

/**
* @brief      Set the state of the target being recognized 
* @param      NULL
* @retval     NULL
*/
void MiniPC_SetTargetFollowMode() {
    MiniPC_MiniPCContrlTypeDef* minipc = MiniPC_GetMiniPCControlDataPtr();

    uint32_t now = HAL_GetTick();
    if (abs((now - minipc->get_target_time)) <= Const_MiniPC_Follow_Target_Time) {
        MiniPC_SetFollowMode(MiniPC_TARGET_FOLLOWING);
    } else if (abs((now - minipc->get_target_time)) >= Const_MiniPC_Lost_Target_Time) {
        MiniPC_SetFollowMode(MiniPC_TARGET_LOST);
    }
}

/**
* @brief      Kalman prediction
* @param      NULL
* @retval     NULL
*/

void MiniPC_KalmanPrediction() {
    static MiniPC_TargetFollowModeEnum last_target_state = MiniPC_TARGET_LOST;
    MiniPC_MiniPCContrlTypeDef* minipc = MiniPC_GetMiniPCControlDataPtr();
    IMU_IMUDataTypeDef* imu = IMU_GetIMUDataPtr();

    float angle_yaw = 0.0f;    //imu->angle.yaw - minipc->yaw_angle;
    float angle_pitch = 0.0f;  //imu->angle.pitch + minipc->pitch_angle;

    //********
    if (minipc->cvkf_control.lowfilter == 1) {
        angle_yaw = imu->angle.yaw - minipc->yaw_ref_filtered;
        angle_pitch = imu->angle.pitch + minipc->pitch_ref_filtered;
    } else {
        angle_yaw = imu->angle.yaw - minipc->yaw_angle;
        angle_pitch = imu->angle.pitch + minipc->pitch_angle;
    }
    //********

    //********
    if (minipc->target_state == MiniPC_TARGET_FOLLOWING && (last_target_state == MiniPC_TARGET_LOST)) {
        //Get New Target: Init CVKF Yaw
        angle_yaw = imu->angle.yaw - minipc->yaw_angle;
        angle_pitch = imu->angle.pitch + minipc->pitch_angle;
        //float angle_yaw = imu->angle.yaw - minipc->yaw_angle;//????
        Kalman_CVInitSetYaw(&minipc->cvkf_data_yaw, angle_yaw, Kalman_CV_CalInitSpeed(-minipc->yaw_angle));
        Kalman_CVKalmanInit(&minipc->cvkf_yaw, &minipc->cvkf_data_yaw);
        Kalman_TurnOnCVKF(&minipc->cvkf_yaw);  //Start Filtering
        //Get New Target: Init CVKF Pitch
        Kalman_CVInitSetPitch(&minipc->cvkf_data_pitch, angle_pitch, 0.0f);  //Kalman_CV_CalInitSpeed(minipc->pitch_angle));
        Kalman_CVKalmanInit(&minipc->cvkf_pitch, &minipc->cvkf_data_pitch);
        Kalman_TurnOnCVKF(&minipc->cvkf_pitch);  //Start Filtering
        //ReStart LowFilter for income Speed:

        minipc->yaw_fil.filted_last_val = minipc->yaw_angle;
        minipc->pitch_fil.filted_last_val = minipc->pitch_angle;
        minipc->yaw_fil.filted_val = minipc->yaw_angle;
        minipc->pitch_fil.filted_val = minipc->pitch_angle;
    }

    else if ((minipc->target_state == MiniPC_TARGET_FOLLOWING) && (last_target_state == MiniPC_TARGET_FOLLOWING)) {
        // Always get the new Measurement For CVKF
        if (minipc->cvkf_yaw.measure_mode == 1) {
            before_cvkf_yaw = angle_yaw;
            Kalman_MeasurementCalc(&minipc->cvkf_yaw, angle_yaw);
        } else {
            //Using CVKF Without Measurements For Tracking:
            Kalman_NonMeasurementCalc(&minipc->cvkf_yaw);
        }

        if (minipc->cvkf_pitch.measure_mode == 1) {
            before_cvkf_pitch = angle_pitch;
            Kalman_MeasurementCalc(&minipc->cvkf_pitch, angle_pitch);
        } else {
            //Using CVKF Without Measurements For Tracking:
            Kalman_NonMeasurementCalc(&minipc->cvkf_pitch);
        }

    }

    else {
        //No Targets: Close CVKF
        Kalman_TurnOffCVKF(&minipc->cvkf_yaw);
        Kalman_CVInitSetYaw(&minipc->cvkf_data_yaw, 0.0f, 0.0f);
        Kalman_TurnOffCVKF(&minipc->cvkf_pitch);
        Kalman_CVInitSetPitch(&minipc->cvkf_data_pitch, 0.0f, 0.0f);
    }
    //*******

    //********
    last_target_state = minipc->target_state;
    //********
}

/**
* @brief      Update minipc data
* @param      NULL
* @retval     NULL
*/
void MiniPC_UpdateControlData() {
    MiniPC_MiniPCContrlTypeDef* minipc = MiniPC_GetMiniPCControlDataPtr();
    MiniPC_MiniPCDataTypeDef* minipc_data = MiniPC_GetMiniPCDataPtr();
    Gimbal_GimbalTypeDef* gimbal = Gimbal_GetGimbalControlPtr();

    minipc->distance = minipc_data->distance / 1000.f;  // mm to m

    if (minipc_data->is_get_target == 1)
        minipc->get_target_time = HAL_GetTick();

    if (minipc->cvkf_control.limit == 1) {
        if (minipc_data->yaw_angle > autoaim_yaw_limit)
            minipc->yaw_angle = autoaim_yaw_limit;
        else if (minipc_data->yaw_angle < -autoaim_yaw_limit)
            minipc->yaw_angle = -autoaim_yaw_limit;
        else
            minipc->yaw_angle = minipc_data->yaw_angle;

        if (minipc_data->pitch_angle > autoaim_pitch_limit)
            minipc->pitch_angle = autoaim_pitch_limit;
        else if (minipc_data->pitch_angle < -autoaim_pitch_limit)
            minipc->pitch_angle = -autoaim_pitch_limit;
        else
            minipc->pitch_angle = minipc_data->pitch_angle;
    }else {
        minipc->yaw_angle = minipc_data->yaw_angle;
        minipc->pitch_angle = minipc_data->pitch_angle;
    }

    minipc->yaw_ref_filtered = Filter_LowPass(minipc->yaw_angle, &minipc->yaw_fil_param, &minipc->yaw_fil);
    minipc->pitch_ref_filtered = Filter_LowPass(minipc->pitch_angle, &minipc->pitch_fil_param, &minipc->pitch_fil);

    if (minipc->cvkf_yaw.switch_mode == 1 && minipc->cvkf_pitch.switch_mode == 1) {
        Kalman_TurnOnMeasureUpdate(&minipc->cvkf_yaw);
        Kalman_TurnOnMeasureUpdate(&minipc->cvkf_pitch);
    }
}

/**
* @brief      Set gimbal autoaim reference
* @param      NULL
* @retval     NULL
*/

void MiniPC_SetAutoAimRef() {
    MiniPC_MiniPCContrlTypeDef* minipc = MiniPC_GetMiniPCControlDataPtr();
    MiniPC_MiniPCDataTypeDef* minipc_data = MiniPC_GetMiniPCDataPtr();
    IMU_IMUDataTypeDef* imu = IMU_GetIMUDataPtr();
    Gimbal_GimbalTypeDef* gimbal = Gimbal_GetGimbalControlPtr();

    float cvkf_yaw_angle = 0.0f;
    float cvkf_pitch_angle = 0.0f;

    if ((minipc->cvkf_control.output == 1) && (minipc->cvkf_control.total == 1) && (minipc->cvkf_control.basicprocess == 1)) {
        MiniPC_KalmanPrediction();

        after_predict_yaw = Kalman_Predict_nT(&minipc->cvkf_yaw, CVKF_NT_YAW);
        after_predict_pitch = Kalman_Predict_nT(&minipc->cvkf_pitch, CVKF_NT_PITCH);

        if (minipc->cvkf_control.predict == 1) {
            cvkf_yaw_angle = after_predict_yaw;
            cvkf_pitch_angle = after_predict_pitch;
        } else {
            cvkf_yaw_angle = minipc->cvkf_yaw.angle;
            cvkf_pitch_angle = minipc->cvkf_pitch.angle;
        }

        static float ref_cvkf_yaw_angle = 0.0f;
        static float ref_cvkf_pitch_angle = 0.0f;

        if (minipc->cvkf_control.dead_domain_delta_ref == 1) {
            if (fabs(ref_cvkf_yaw_angle - cvkf_yaw_angle) > autoaim_yaw_dead) {
                ref_cvkf_yaw_angle = cvkf_yaw_angle;
            }
            if (fabs(ref_cvkf_pitch_angle - cvkf_pitch_angle) > autoaim_pitch_dead) {
                ref_cvkf_pitch_angle = cvkf_pitch_angle;
            }
        } else {
            ref_cvkf_yaw_angle = cvkf_yaw_angle;
            ref_cvkf_pitch_angle = cvkf_pitch_angle;
        }

        if (minipc->cvkf_control.offset == 1) {
            float delta_predict = after_predict_yaw - minipc->cvkf_yaw.angle;

            if (fabs(delta_predict) < 0.5f)
                autoaim_yaw_offset = 0;
            else if (delta_predict >= 3.0f)
                autoaim_yaw_offset = 4.0f;
            else if (delta_predict >= 1.0f)
                autoaim_yaw_offset = (delta_predict - 1.0f) * 4.0f / 2.5f;
            else if (delta_predict <= -3.0f)
                autoaim_yaw_offset = -4.0;
            else if (delta_predict <= -1.0f)
                autoaim_yaw_offset = (delta_predict + 1.0f) * 4.0f / 2.5f;
            else
                autoaim_yaw_offset = 0;
        }

        Gimbal_SetYawAutoRef(ref_cvkf_yaw_angle + autoaim_yaw_offset);
        Gimbal_SetPitchAutoRef(ref_cvkf_pitch_angle + autoaim_pitch_offset);
    }
}

/**
* @brief      Set gimbal reference
* @param      NULL
* @retval     NULL
*/
void MiniPC_SetGimbalRef() {
    MiniPC_MiniPCContrlTypeDef* minipc = MiniPC_GetMiniPCControlDataPtr();
    MiniPC_MiniPCDataTypeDef* minipc_data = MiniPC_GetMiniPCDataPtr();
    IMU_IMUDataTypeDef* imu = IMU_GetIMUDataPtr();
    Gimbal_GimbalTypeDef* gimbal = Gimbal_GetGimbalControlPtr();

    if ((minipc->enable_aim_output) && (minipc->target_state == MiniPC_TARGET_FOLLOWING) && (gimbal->mode.present_mode == Gimbal_ARMOR)) {
        MiniPC_SetAutoAimRef();
    } else if ((minipc->enable_aim_output) && (minipc->target_state == MiniPC_TARGET_FOLLOWING) && (gimbal->mode.present_mode == Gimbal_BIG_ENERGY)) {
        Gimbal_SetYawAutoRef(imu->angle.yaw - minipc->yaw_ref_filtered + energy_yaw_offset);
        Gimbal_SetPitchAutoRef(imu->angle.pitch + minipc->pitch_ref_filtered + energy_pitch_offset);
    } else if ((minipc->enable_aim_output) && (minipc->target_state == MiniPC_TARGET_FOLLOWING) && (gimbal->mode.present_mode == Gimbal_SMALL_ENERGY)) {
        Gimbal_SetYawAutoRef(imu->angle.yaw - minipc->yaw_ref_filtered + energy_yaw_offset);
        Gimbal_SetPitchAutoRef(imu->angle.pitch + minipc->pitch_ref_filtered + energy_pitch_offset);
    }
    else
        return;
}

#endif
