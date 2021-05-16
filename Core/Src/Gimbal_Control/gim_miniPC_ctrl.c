 /*
 *  Project      : Infantry_Momentum
 * 
 *  file         : gim_miniPC_ctrl.c
 *  Description  : This file contains MiniPC control function
 *  LastEditors  : 动情丶卜灬动心
 *  Date         : 2021-05-04 20:53:31
 *  LastEditTime : 2021-05-16 00:26:31
 */

#include "gim_miniPC_ctrl.h"

#if __FN_IF_ENABLE(__FN_CTRL_MINIPC)

#include "buscomm_ctrl.h"
#include "const_lib.h"
#include "gim_gimbal_ctrl.h"

MiniPC_MiniPCContrlTypeDef MiniPC_MiniPCContrlData;

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
    MiniPC_MiniPCContrlTypeDef *minipc = MiniPC_GetMiniPCControlDataPtr();

    minipc->enable_aim_output = 1;
    Filter_LowPassInit(-1, &minipc->yaw_fil_param);
    Filter_LowPassInit(-1, &minipc->pitch_fil_param);
}


/**
* @brief      Change aiming mode
* @param      mode: MiniPC aim mode enum
* @retval     NULL
*/
void MiniPC_ChangeAimMode(MiniPC_AutoAimModeEnum mode) {
    MiniPC_MiniPCContrlTypeDef *minipc = MiniPC_GetMiniPCControlDataPtr();
    MiniPC_MiniPCDataTypeDef *minipc_data = MiniPC_GetMiniPCDataPtr();
  
    minipc->aim_mode = mode;
    minipc_data->mode = minipc->aim_mode;
}


/**
* @brief      MiniPC auto aim decode control
* @param      NULL
* @retval     NULL
*/
void MiniPC_CalcAutoAim() {
	Gimbal_GimbalTypeDef *gimbal = Gimbal_GetGimbalControlPtr();

    // Set following mode
	MiniPC_SetTargetFollowMode();
    
	// Gimbal Trace Forecast
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
    MiniPC_MiniPCContrlTypeDef *minipc = MiniPC_GetMiniPCControlDataPtr();

    minipc->target_state = mode;
}


/**
* @brief      Set the state of the target being recognized 
* @param      NULL
* @retval     NULL
*/
void MiniPC_SetTargetFollowMode() {
    MiniPC_MiniPCContrlTypeDef *minipc = MiniPC_GetMiniPCControlDataPtr();

    uint32_t now = HAL_GetTick();
    if (now - minipc->get_target_time <= Const_MiniPC_Follow_Target_Time) {
        MiniPC_SetFollowMode(MiniPC_TARGET_FOLLOWING);
    }
    else if (now - minipc->get_target_time >= Const_MiniPC_Lost_Target_Time) {
        MiniPC_SetFollowMode(MiniPC_TARGET_LOST);
    }
}


/**
* @brief      Kalman prediction
* @param      NULL
* @retval     NULL
*/
void MiniPC_KalmanPrediction() {
    return;
}


/**
* @brief      Update minipc data
* @param      NULL
* @retval     NULL
*/
void MiniPC_UpdateControlData() {
    MiniPC_MiniPCContrlTypeDef *minipc = MiniPC_GetMiniPCControlDataPtr();
    MiniPC_MiniPCDataTypeDef *minipc_data = MiniPC_GetMiniPCDataPtr();

    minipc->distance    = minipc_data->distance;

    if (minipc_data->is_get_target == 1) {
        minipc->get_target_time = HAL_GetTick();
    }
    minipc->yaw_angle   = minipc_data->yaw_angle;
    minipc->pitch_angle = minipc_data->pitch_angle;
}


/**
* @brief      Set gimbal reference
* @param      NULL
* @retval     NULL
*/
void MiniPC_SetGimbalRef() {
    MiniPC_MiniPCContrlTypeDef *minipc = MiniPC_GetMiniPCControlDataPtr();
    MiniPC_MiniPCDataTypeDef *minipc_data = MiniPC_GetMiniPCDataPtr();
	IMU_IMUDataTypeDef *imu = IMU_GetIMUDataPtr();
    Gimbal_GimbalTypeDef *gimbal = Gimbal_GetGimbalControlPtr();
    
    float yaw_ref = 0, pitch_ref = 0;
    if (minipc_data->state == MiniPC_CONNECTED) {
        yaw_ref = minipc->yaw_angle;
        pitch_ref = minipc->pitch_angle;
    }
    
    minipc->yaw_ref_filtered   = Filter_LowPass(yaw_ref,   &minipc->yaw_fil_param,   &minipc->yaw_fil);
    minipc->pitch_ref_filtered = Filter_LowPass(pitch_ref, &minipc->pitch_fil_param, &minipc->pitch_fil);
    
    if ((minipc->enable_aim_output) && (minipc->target_state ==  MiniPC_TARGET_FOLLOWING) && (gimbal->mode.present_mode == Gimbal_ARMOR)) {
        Gimbal_SetYawAutoRef(minipc->yaw_ref_filtered);
        Gimbal_SetPitchAutoRef(minipc->pitch_ref_filtered);
    }
}

#endif
