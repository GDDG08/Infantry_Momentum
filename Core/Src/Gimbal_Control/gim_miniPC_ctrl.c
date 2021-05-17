 /*
 *  Project      : Infantry_Momentum
 * 
 *  file         : gim_miniPC_ctrl.c
 *  Description  : This file contains MiniPC control function
 *  LastEditors  : 动情丶卜灬动心
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
int CVKF_NT_YAW = 0;
int CVKF_NT_PITCH = 0;


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

	//      CVKF Init Variables:
	minipc->cvkf_control.total = 1;
	minipc->cvkf_control.basicprocess = 1;
	minipc->cvkf_control.jumpjudge = 0;
	minipc->cvkf_control.limit = 0;
	minipc->cvkf_control.output = 0;
	minipc->cvkf_control.predict = 0;

	//CVKF for Yaw Angle:
	Kalman_CVKalmanInitYawParam(&minipc->cvkf_data_yaw, 1/1000.0f ,0.0f);
	Kalman_CVKalmanInit(&minipc->cvkf_yaw, &minipc->cvkf_data_yaw);
	//CVKF for Pitch Angle:
	Kalman_CVKalmanInitPitchParam(&minipc->cvkf_data_pitch, 1/1000.0f, 0.0f);
	Kalman_CVKalmanInit(&minipc->cvkf_pitch, &minipc->cvkf_data_pitch);

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
	MiniPC_MiniPCContrlTypeDef *minipc = MiniPC_GetMiniPCControlDataPtr();
    
    // Set following mode
	MiniPC_SetTargetFollowMode();
    
	// Gimbal Trace Forecast
	if((minipc->cvkf_control.total == 1) && (minipc->cvkf_control.basicprocess==1))
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
    if (abs(now - minipc->get_target_time) <= Const_MiniPC_Follow_Target_Time) {
        MiniPC_SetFollowMode(MiniPC_TARGET_FOLLOWING);
    }
    else if (abs(now - minipc->get_target_time) >= Const_MiniPC_Lost_Target_Time) {
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
	if (angle_pitch > 0){
		offset = 10.0f;
	}
    else if (angle_pitch > -10.0f) {
		offset = 5.0f;
	}
    else {
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
	MiniPC_MiniPCContrlTypeDef *minipc = MiniPC_GetMiniPCControlDataPtr();
	IMU_IMUDataTypeDef *imu = IMU_GetIMUDataPtr();

	if (minipc->target_state == MiniPC_TARGET_FOLLOWING
		&& last_target_state == MiniPC_TARGET_LOST) {
	//Get New Target: Init CVKF Yaw
		float angle_yaw = imu->angle.yaw - minipc->yaw_angle;//绝对坐标
		Kalman_CVInitSetYaw(&minipc->cvkf_data_yaw, angle_yaw);
		Kalman_CVKalmanInit(&minipc->cvkf_yaw, &minipc->cvkf_data_yaw);
		Kalman_TurnOnCVKF(&minipc->cvkf_yaw);   //Start Filtering
	//Get New Target: Init CVKF Pitch
		float angle_pitch = imu->angle.pitch + minipc->pitch_angle;//绝对坐标
		Kalman_CVInitSetPitch(&minipc->cvkf_data_pitch, angle_pitch);
		Kalman_CVKalmanInit(&minipc->cvkf_pitch, &minipc->cvkf_data_pitch);
		Kalman_TurnOnCVKF(&minipc->cvkf_pitch);//Start Filtering
	}
    else if ((minipc->target_state == MiniPC_TARGET_FOLLOWING) 
			&& (last_target_state == MiniPC_TARGET_FOLLOWING)) {
		// Always get the new Measurement For CVKF
		if (minipc->cvkf_yaw.measure_mode == 1) {
			
			float _yaw_angle = imu->angle.yaw - minipc->yaw_angle;
			if (minipc->cvkf_control.jumpjudge == 1)
				_yaw_angle = Kalman_JudgeChange(&minipc->cvkf_yaw, _yaw_angle);
			if ((minipc->cvkf_yaw.targer_change == 1) && (minipc->cvkf_control.jumpjudge == 1)) {
				Kalman_CVInitSetYaw(&minipc->cvkf_data_yaw, _yaw_angle);
				Kalman_CVKalmanInit(&minipc->cvkf_yaw, &minipc->cvkf_data_yaw);
				minipc->cvkf_yaw.targer_change = 0;
			}
            else {
					Kalman_MeasurementCalc(&minipc->cvkf_yaw, _yaw_angle);
			}
		}
        else {
			//Using CVKF Without Measurements For Tracking:	
			Kalman_NonMeasurementCalc(&minipc->cvkf_yaw);
		}
		
		if (minipc->cvkf_pitch.measure_mode == 1) {
			
			float _pitch_angle = imu->angle.pitch + minipc->pitch_angle;
			if (minipc->cvkf_control.jumpjudge == 1)
				_pitch_angle = Kalman_JudgeChange(&minipc->cvkf_pitch, _pitch_angle);
			
			if ((minipc->cvkf_pitch.targer_change == 1) && (minipc->cvkf_control.jumpjudge == 1)) {
				Kalman_CVInitSetPitch(&minipc->cvkf_data_pitch, _pitch_angle);
				Kalman_CVKalmanInit(&minipc->cvkf_pitch, &minipc->cvkf_data_pitch);
				minipc->cvkf_pitch.targer_change = 0;
			}
            else {
				Kalman_MeasurementCalc(&minipc->cvkf_pitch, _pitch_angle);
			}
		}
        else {
			//Using CVKF Without Measurements For Tracking:	
			Kalman_NonMeasurementCalc(&minipc->cvkf_pitch);
		}

	}
    else {
		//No Targets: Close CVKF
		Kalman_TurnOffCVKF(&minipc->cvkf_yaw);
		Kalman_TurnOffCVKF(&minipc->cvkf_pitch);
	}
	//State Update:
	last_target_state = minipc->target_state;
}


/**
* @brief      Update minipc data
* @param      NULL
* @retval     NULL
*/
void MiniPC_UpdateControlData() {
    MiniPC_MiniPCContrlTypeDef *minipc = MiniPC_GetMiniPCControlDataPtr();
    MiniPC_MiniPCDataTypeDef *minipc_data = MiniPC_GetMiniPCDataPtr();

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

//		minipc->yaw_angle = minipc_data->yaw_angle;
//		minipc->pitch_angle = minipc_data->pitch_angle;
		
	if (minipc->cvkf_yaw.switch_mode == 1 && minipc->cvkf_pitch.switch_mode == 1){
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
    MiniPC_MiniPCContrlTypeDef *minipc = MiniPC_GetMiniPCControlDataPtr();
    MiniPC_MiniPCDataTypeDef *minipc_data = MiniPC_GetMiniPCDataPtr();
	IMU_IMUDataTypeDef *imu = IMU_GetIMUDataPtr();
    Gimbal_GimbalTypeDef *gimbal = Gimbal_GetGimbalControlPtr();
    
    float yaw_ref = 0.0f, pitch_ref = 0.0f;
	float cvkf_yaw_angle = 0.0f;
	float cvkf_pitch_angle = 0.0f;
	// if(minipc->cvkf_control.total != 1) {
//	if (minipc_data->state == MiniPC_CONNECTED) {
        yaw_ref = minipc->yaw_angle;
        pitch_ref = minipc->pitch_angle;
//	}
	minipc->yaw_ref_filtered   = Filter_LowPass(yaw_ref,   &minipc->yaw_fil_param,   &minipc->yaw_fil);
	minipc->pitch_ref_filtered = Filter_LowPass(pitch_ref, &minipc->pitch_fil_param, &minipc->pitch_fil);
//	minipc->yaw_ref_filtered   = minipc->yaw_angle;
//	minipc->pitch_ref_filtered = minipc->pitch_angle;
	//}
		
    if ((minipc->enable_aim_output) 
		&& (minipc->target_state ==  MiniPC_TARGET_FOLLOWING) 
		&& (gimbal->mode.present_mode == Gimbal_ARMOR)) {
        if ((minipc->cvkf_control.output == 1) && (minipc->cvkf_control.total == 1)
            && (minipc->cvkf_control.basicprocess == 1)) {
			if (minipc->cvkf_control.predict == 1) {
				//根据误差角进行提前预测:
				int yaw_predict = fabs(yaw_ref/50*1000);	//云台相应速度50degree/s
				//int pitch_predict = fabs(pitch_ref/);
				cvkf_yaw_angle   = Kalman_Predict_nT(&minipc->cvkf_yaw, CVKF_NT_YAW + yaw_predict);
				cvkf_pitch_angle = Kalman_Predict_nT(&minipc->cvkf_pitch, CVKF_NT_PITCH);
			}
            else {
				cvkf_yaw_angle   = minipc->cvkf_yaw.angle;
				cvkf_pitch_angle = minipc->cvkf_pitch.angle;
			}
			if (minipc->cvkf_control.offset == 1) {
				// TODO:ADD Pitch OFFSET After CVKF
			}
			// TRY TODO: Using LowPassFilter After CVKF:
			// Set Gimbal Angle:
			Gimbal_SetYawAutoRef(cvkf_yaw_angle);
			Gimbal_SetPitchAutoRef(cvkf_pitch_angle);
	    }
        else {				
	    	// Set_Gambal_Angle:
	    	Gimbal_SetYawAutoRef(imu->angle.yaw - minipc->yaw_ref_filtered);
	    	Gimbal_SetPitchAutoRef(imu->angle.pitch + minipc->pitch_ref_filtered);
	    }   
    }
}

#endif
