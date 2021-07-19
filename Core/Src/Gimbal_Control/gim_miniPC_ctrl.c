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
#include "math_alg.h"

MiniPC_MiniPCContrlTypeDef MiniPC_MiniPCContrlData;

//CVKF Predict nT For Aiming
#if (__FN_INFANTRY_TYPE == 4)
int CVKF_NT_YAW = 0;
#endif
#if (__FN_INFANTRY_TYPE == 3)
int CVKF_NT_YAW = 80;
#endif
#if (__FN_INFANTRY_TYPE == 5)
int CVKF_NT_YAW = 0;
#endif
int CVKF_NT_PITCH = 20;
float before_cvkf_yaw = 0.0f;
float before_cvkf_pitch = 0.0f;
float after_predict_yaw = 0.0f;
float after_predict_pitch = 0.0f;

float debug_pitch_offset = -2.0f;
float debug_yaw_offset = 0.3f;

float debug_yaw_dead = 0.1f;
float debug_pitch_dead = 0.05f;

float yaw_angle_speed = 0.0f;
float pitch_angle_speed = 0.0f;

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
    Filter_LowPassInit(0.3, &minipc->pitch_fil_param);

    Filter_LowPassInit(0.1, &minipc->yaw_aim_param);
    Filter_LowPassInit(0.1, &minipc->pitch_aim_param);

    //CVKF Init Variables:
    minipc->cvkf_control.total = 1;
    minipc->cvkf_control.basicprocess = 1;
    minipc->cvkf_control.jumpjudge = 0;  //NO FUNCTION
    minipc->cvkf_control.limit = 0;      //No FUNCTION
    minipc->cvkf_control.output = 1;
    minipc->cvkf_control.predict = 1;
    minipc->cvkf_control.lowfilter = 1;  //Had FUNCTION
    minipc->cvkf_control.dead_domain_delta_ref = 0;
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

    // Set following mode
    MiniPC_SetTargetFollowMode();

    // Gimbal Trace Forecast
    if ((minipc->cvkf_control.total == 1) && (minipc->cvkf_control.basicprocess == 1))
        MiniPC_KalmanPrediction();

    MiniPC_SetGimbalRef();
}

/**
* @brief      MiniPC auto aim decode control
* @param      NULL
* @retval     NULL
*/
void MiniPC_UpdateAutoAim() {
    // Update
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
* @brief      Add pitch axis trajectory compensation
* @param      angle_pitch: Set pitch axis angle
* @retval     float: Pitch axis angle after compensation
*/
float MiniPC_AddPitchOffset(float angle_pitch) {
    float offset = 0.0f;
    if (angle_pitch > 0) {
        offset = 10.0f;
    } else if (angle_pitch > -10.0f) {
        offset = 5.0f;
    } else {
        offset = 0.1f;
    }
    return offset;
}

/**
* @brief      Kalman prediction
* @param      NULL
* @retval     NULL
*/
void MiniPC_KalmanPrediction() {
    //Data Preprocessing:
    static MiniPC_TargetFollowModeEnum last_target_state = MiniPC_TARGET_LOST;
    MiniPC_MiniPCContrlTypeDef* minipc = MiniPC_GetMiniPCControlDataPtr();
    IMU_IMUDataTypeDef* imu = IMU_GetIMUDataPtr();

    float angle_yaw = 0.0f;    //imu->angle.yaw - minipc->yaw_angle;
    float angle_pitch = 0.0f;  //imu->angle.pitch + minipc->pitch_angle;

    //********
    //???????????:
    if (minipc->cvkf_control.lowfilter == 1) {
        //angle_yaw = imu->angle.yaw - minipc->yaw_ref_filtered;//????
        //angle_pitch = imu->angle.pitch + minipc->pitch_ref_filtered;//????
        angle_yaw = imu->angle.yaw - minipc->yaw_ref_filtered;
        angle_pitch = imu->angle.pitch + minipc->pitch_ref_filtered;
    } else {
        angle_yaw = imu->angle.yaw - minipc->yaw_angle;
        angle_pitch = imu->angle.pitch + minipc->pitch_angle;
    }
    //********

    //********
    if (minipc->target_state == MiniPC_TARGET_FOLLOWING && last_target_state == MiniPC_TARGET_LOST) {
        //Get New Target: Init CVKF Yaw
        angle_yaw = imu->angle.yaw - minipc->yaw_angle;
        angle_pitch = imu->angle.pitch + minipc->pitch_angle;
        //float angle_yaw = imu->angle.yaw - minipc->yaw_angle;//????
        Kalman_CVInitSetYaw(&minipc->cvkf_data_yaw, angle_yaw, Kalman_CV_CalInitSpeed(-minipc->yaw_angle));
        Kalman_CVKalmanInit(&minipc->cvkf_yaw, &minipc->cvkf_data_yaw);
        Kalman_TurnOnCVKF(&minipc->cvkf_yaw);  //Start Filtering
                                               //Get New Target: Init CVKF Pitch
                                               //float angle_pitch = imu->angle.pitch + minipc->pitch_angle;//????
        Kalman_CVInitSetPitch(&minipc->cvkf_data_pitch, angle_pitch, 0.0f);
        Kalman_CVKalmanInit(&minipc->cvkf_pitch, &minipc->cvkf_data_pitch);
        Kalman_TurnOnCVKF(&minipc->cvkf_pitch);  //Start Filtering

        //ReStart LowFilter for income Speed:
        //		minipc->yaw_fil.filted_last_val = minipc->yaw_angle;
        //		minipc->pitch_fil.filted_last_val = minipc->pitch_angle;
        //		minipc->yaw_fil.filted_val = minipc->yaw_angle;
        //		minipc->pitch_fil.filted_val = minipc->pitch_angle;
        //		//Start Low Filter For Angle Speed Preparing For Auto Aim Energy:
        //		minipc->pitch_aim.filted_last_val = minipc->cvkf_pitch.XLast.pData[1];
        //		minipc->pitch_aim.filted_val = minipc->cvkf_pitch.XLast.pData[1];
        //		minipc->yaw_aim.filted_last_val = minipc->cvkf_yaw.XLast.pData[1];
        //		minipc->yaw_aim.filted_val = minipc->cvkf_yaw.XLast.pData[1];

    } else if ((minipc->target_state == MiniPC_TARGET_FOLLOWING) && (last_target_state == MiniPC_TARGET_FOLLOWING)) {
        // Always get the new Measurement For CVKF
        if (minipc->cvkf_yaw.measure_mode == 1) {
            //float _yaw_angle = imu->angle.yaw - minipc->yaw_angle;
            before_cvkf_yaw = angle_yaw;
            {
                Kalman_MeasurementCalc(&minipc->cvkf_yaw, angle_yaw);
            }
        } else {
            //Using CVKF Without Measurements For Tracking:
            Kalman_NonMeasurementCalc(&minipc->cvkf_yaw);
        }

        if (minipc->cvkf_pitch.measure_mode == 1) {
            //float _pitch_angle = imu->angle.pitch + minipc->pitch_angle ;
            before_cvkf_pitch = angle_pitch;
            {
                Kalman_MeasurementCalc(&minipc->cvkf_pitch, angle_pitch);
            }
        } else {
            //Using CVKF Without Measurements For Tracking:
            Kalman_NonMeasurementCalc(&minipc->cvkf_pitch);
        }

    } else {
        //No Targets: Close CVKF
        Kalman_TurnOffCVKF(&minipc->cvkf_yaw);
        //Kalman_CVInitSetYaw(&minipc->cvkf_data_yaw, 0.0f ,0.0f);
        Kalman_TurnOffCVKF(&minipc->cvkf_pitch);
        //Kalman_CVInitSetPitch(&minipc->cvkf_data_pitch, 0.0f ,0.0f);
    }
    //*******

    //********
    //State Update:
    last_target_state = minipc->target_state;
    //********
}

/**
* @brief      Update minipc data
* @param      NULL
* @retval     NULL
*/
void +MiniPC_UpdateControlData() {
    MiniPC_MiniPCContrlTypeDef* minipc = MiniPC_GetMiniPCControlDataPtr();
    MiniPC_MiniPCDataTypeDef* minipc_data = MiniPC_GetMiniPCDataPtr();

    //Data Preprocessing:
    //TODO:
    minipc->distance = minipc_data->distance;

    if (minipc_data->is_get_target == 1) {
        minipc->get_target_time = HAL_GetTick();
        //Update data when get target
        //Otherwise keep last data
        minipc->yaw_angle = minipc_data->yaw_angle;
        minipc->pitch_angle = minipc_data->pitch_angle;
    }

    minipc->yaw_ref_filtered = Filter_LowPass(minipc->yaw_angle, &minipc->yaw_fil_param, &minipc->yaw_fil);
    minipc->pitch_ref_filtered = Filter_LowPass(minipc->pitch_angle, &minipc->pitch_fil_param, &minipc->pitch_fil);
    //		minipc->yaw_angle = minipc_data->yaw_angle;
    //		minipc->pitch_angle = minipc_data->pitch_angle;

    if (minipc->cvkf_yaw.switch_mode == 1 && minipc->cvkf_pitch.switch_mode == 1) {
        //TODO: CVKF with measurements.
    }
    //Update cvkf-mode when get new measurements!
    Kalman_TurnOnMeasureUpdate(&minipc->cvkf_yaw);
    Kalman_TurnOnMeasureUpdate(&minipc->cvkf_pitch);
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

    //    float yaw_ref = 0.0f, pitch_ref = 0.0f;
    float cvkf_yaw_angle = 0.0f;
    float cvkf_pitch_angle = 0.0f;

    if ((minipc->enable_aim_output) && (minipc->target_state == MiniPC_TARGET_FOLLOWING) && (gimbal->mode.present_mode == Gimbal_ARMOR)) {
        if ((minipc->cvkf_control.output == 1) && (minipc->cvkf_control.total == 1) && (minipc->cvkf_control.basicprocess == 1)) {
            //FOR Prediction：
            after_predict_yaw = Kalman_Predict_nT(&minipc->cvkf_yaw, CVKF_NT_YAW);
            after_predict_pitch = Kalman_Predict_nT(&minipc->cvkf_pitch, CVKF_NT_PITCH);

            if (minipc->cvkf_control.predict == 1) {
                //???????????:
                //int pitch_predict = fabs(pitch_ref/);
                //cvkf_yaw_angle  = Kalman_Predict_nT(&minipc->cvkf_yaw, CVKF_NT_YAW + yaw_predict);
                //cvkf_pitch_angle = Kalman_Predict_nT(&minipc->cvkf_pitch, CVKF_NT_PITCH);

                //Set REF with Prediction:
                cvkf_yaw_angle = after_predict_yaw;
                cvkf_pitch_angle = after_predict_pitch;
            } else {
                //Set REF without Prediction
                cvkf_yaw_angle = minipc->cvkf_yaw.angle;
                cvkf_pitch_angle = minipc->cvkf_pitch.angle;
            }
            //				{
            //				// TODO:ADD OFFSET For _yaw_angle
            //				float ref_pffset_yaw =  after_predict_yaw - minipc->cvkf_yaw.angle;
            //				//根据预测值的正负设置额外的偏角
            //				if(ref_pffset_yaw > 1.0f){
            //					cvkf_yaw_angle += 2.0f;
            //				}else if(ref_pffset_yaw < -1.0f){
            //					cvkf_yaw_angle += -2.0f;
            //				}else{
            //					//cvkf_yaw_angle += 0;
            //				}

            //			}
            //Set REF Dead Region :
            static float ref_cvkf_yaw_angle = 0.0f;
            static float ref_cvkf_pitch_angle = 0.0f;
            //For Auto Dynamic Aiming OFFSET：
            float yaw_auto_aim_offset = 0.0f;
            float pitch_auto_aim_offset = 0.0f;

            if (minipc->cvkf_control.dead_domain_delta_ref == 1) {
                //Delta Angle Bigger than dead domain: SET REF
                if (fabs(ref_cvkf_yaw_angle - cvkf_yaw_angle) > debug_yaw_dead) {
                    ref_cvkf_yaw_angle = cvkf_yaw_angle;
                }
                //Delta Angle Bigger than dead domain: SET REF
                if (fabs(ref_cvkf_pitch_angle - cvkf_pitch_angle) > debug_pitch_dead) {
                    ref_cvkf_pitch_angle = cvkf_pitch_angle;
                }
            } else {
                ref_cvkf_yaw_angle = cvkf_yaw_angle;
                ref_cvkf_pitch_angle = cvkf_pitch_angle;
            }
            // TRY TODO: Using LowPassFilter After CVKF:
            //			if (minipc->cvkf_control.offset == 1) {
            //					float delta_predict = after_predict_yaw - minipc->cvkf_yaw.angle;
            //					if(fabs(delta_predict)<0.5f){
            //						debug_yaw_offset = 0;
            //					}else if(delta_predict >= 2.5f){
            //						debug_yaw_offset = 5.0f;
            //					}else if(delta_predict >= 0.5f){
            //						debug_yaw_offset = (delta_predict-0.5f)*5.0f/2.f;
            //					}else if(delta_predict < -2.5f){
            //						debug_yaw_offset = -5.0;
            //					}else if(delta_predict < -0.5f){
            //						debug_yaw_offset = (delta_predict + 0.5f)*5.0/2.0f;
            //					}else{
            //						debug_yaw_offset = 0;
            //					}
            //			}
            if (minipc->cvkf_control.offset == 1) {
                //****** For Aiming Sentry:
                //					float delta_predict = after_predict_yaw - minipc->cvkf_yaw.angle;
                //					if(fabs(delta_predict)<0.5f){
                //						debug_yaw_offset = 0;
                //					}else if(delta_predict >= 3.0f){
                //						debug_yaw_offset = 4.0f;
                //					}else if(delta_predict >= 1.0f){
                //						debug_yaw_offset = (delta_predict-1.0f)*4.0f/2.5f;
                //					}else if(delta_predict <= -3.0f){
                //						debug_yaw_offset = -4.0;
                //					}else if(delta_predict <= -1.0f){
                //						debug_yaw_offset = (delta_predict)*2.0f*4.0f/2.5f;
                //					}else{
                //						debug_yaw_offset = 0;
                //					}
                //******
                //****** For Aiming Energy:
                yaw_angle_speed = Filter_LowPass(minipc->cvkf_yaw.XLast.pData[1], &minipc->yaw_aim_param, &minipc->yaw_aim);
                pitch_angle_speed = Filter_LowPass(minipc->cvkf_pitch.XLast.pData[1], &minipc->pitch_aim_param, &minipc->pitch_aim);

                if (yaw_angle_speed >= 1.0f) {
                    yaw_auto_aim_offset = 0.4f;
                } else if (yaw_angle_speed <= -1.0f) {
                    yaw_auto_aim_offset = -0.4f;
                } else {
                    yaw_auto_aim_offset = 0.0f;
                }

                if (pitch_angle_speed >= 1.0f) {
                    pitch_auto_aim_offset = 0.3f;
                } else if (pitch_angle_speed <= -1.0f) {
                    pitch_auto_aim_offset = 0.1f;
                } else {
                    pitch_auto_aim_offset = 0.0f;
                }
            }
            // Set Gimbal Angle:
            Gimbal_SetYawAutoRef(ref_cvkf_yaw_angle + debug_yaw_offset + yaw_auto_aim_offset);
            Gimbal_SetPitchAutoRef(ref_cvkf_pitch_angle + debug_pitch_offset + pitch_auto_aim_offset);
        } else {
            // Set_Gambal_Angle:
            Gimbal_SetYawAutoRef(imu->angle.yaw - minipc->yaw_ref_filtered);
            Gimbal_SetPitchAutoRef(imu->angle.pitch + minipc->pitch_ref_filtered);
        }
    }
}

#endif
