/*
 *  Project      : Infantry_Momentum
 * 
 *  file         : kalman_alg.c
 *  Description  : This file contains the kalman filter algorithm
 *  LastEditors  : ����ؼ���ᶯ��
 *  Date         : 2021-05-04 20:53:31
 *  LastEditTime : 2021-05-16 20:40:21
 */

#include "kalman_alg.h"

/**
  * @brief      Initialization of CV Kalman filter yaw parameters
  * @param      cvkf_data: Initialized Kalman filter structure
  * @param      KF_T: Kalman 
  * @param      init_angle_yaw: Initialized yaw axis angle
  * @retval     NULL
  */
void Kalman_CVKalmanInitYawParam(Kalman_CVKalmanInitDataTypeDef *cvkf_data, const float KF_T, const float init_angle_yaw) {
	static float KF_A[4]  = {1.0f, 0.0f,
							 0.0f, 1.0f};
		         KF_A[1]  = KF_T;
	static float KF_C[2]  = {1.0f, 0.0f};
	static float XLast[2] = {0.0f};
		         XLast[0] = init_angle_yaw;
	static float Xpre[2]  = {0.0f};
		         XLast[0] = init_angle_yaw;
	static float Xopt[2]  = {0.0f};
		         XLast[0] = init_angle_yaw;

	static float PLast[4] = {1.0f, 0.0f,
							 0.0f, 1.0f};
	static float Ppre[4]  = {0.0f};
	static float Popt[4]  = {0.0f};
	static float Q[4]     = {1.0f, 0.0f,
						     0.0f, 1.0f};
	static float R[1]     = {1.0f};
	static float Kf[2]    = {0.0f};
	
	for (int i = 0; i < 4; i++) {
		cvkf_data->KF_A[i] = KF_A[i];
		if (i < 2) {
			cvkf_data->KF_C[i]  = KF_C[i];
			cvkf_data->XLast[i] = XLast[i];
			cvkf_data->Xpre[i]  = Xpre[i];
			cvkf_data->Xopt[i]  = Xopt[i];
			
			cvkf_data->Kf[i] = Kf[i];
		}
		cvkf_data->PLast[i] = PLast[i];
		cvkf_data->Ppre[i]  = Ppre[i];
		cvkf_data->Popt[i]  = Popt[i];
		cvkf_data->Q[i]     = Q[i];
	}
	cvkf_data->R[0] = R[0];
}


/**
  * @brief      Initialization of CV Kalman filter pitch parameters
  * @param      cvkf_data: Initialized Kalman filter structure
  * @param      KF_T: Kalman 
  * @param      init_angle_pitch: Initialized pitch axis angle
  * @retval     NULL
  */
void Kalman_CVKalmanInitPitchParam(Kalman_CVKalmanInitDataTypeDef *cvkf_data, const float KF_T, const float init_angle_pitch) {
	static float KF_A[4]  = {1.0f, 0.0f,
			          0.0f, 1.0f};
		         KF_A[1]  = KF_T;
	static float KF_C[2]  = {1.0f, 0.0f};
	static float XLast[2] = {0.0f};
		         XLast[0] = init_angle_pitch;
	static float Xpre[2]  = {0.0f};
		         XLast[0] = init_angle_pitch;
	static float Xopt[2]  = {0.0f};
		         XLast[0] = init_angle_pitch;
	static float PLast[4] = {1.0f, 0.0f,
					         0.0f, 1.0f};
	static float Ppre[4]  = {0.0f};
	static float Popt[4]  = {0.0f};
	static float Q[4]     = {1.0f, 0.0f,
				             0.0f, 1.0f};
	static float R[1]    = {1.0f};
	static float Kf[2]   = {0.0f};

	for (int i = 0; i < 4; i++) {
		cvkf_data->KF_A[i] = KF_A[i];
		if (i < 2) {
			cvkf_data->KF_C[i]  = KF_C[i];
			cvkf_data->XLast[i] = XLast[i];
			cvkf_data->Xpre[i]  = Xpre[i];
			cvkf_data->Xopt[i]  = Xopt[i];
			
			cvkf_data->Kf[i] = Kf[i];
		}
		cvkf_data->PLast[i] = PLast[i];
		cvkf_data->Ppre[i]  = Ppre[i];
		cvkf_data->Popt[i]  = Popt[i];
		cvkf_data->Q[i]     = Q[i];
	}
	cvkf_data->R[0] = R[0];
}


/**
  * @brief      Initialization of CV Kalman filter yaw 
  * @param      cvkf_data: Initialized Kalman filter structure
  * @param      init_angle_yaw: Initialized yaw angle
  * @retval     NULL
  */
void Kalman_CVInitSetYaw(Kalman_CVKalmanInitDataTypeDef *cvkf_data, const float init_angle_yaw) {
	static float XLast[2] = {0.0f};
		         XLast[0] = init_angle_yaw;
	static float Xpre[2]  = {0.0f};
		         XLast[0] = init_angle_yaw;
	static float Xopt[2]  = {0.0f};
		         XLast[0] = init_angle_yaw;
	static float PLast[4] = {1.0f, 0.0f,
							 0.0f, 1.0f};	
	for (int i = 0; i < 4; i++) {
		if (i < 2) {
			cvkf_data->XLast[i] = XLast[i];
			cvkf_data->Xpre[i]  = Xpre[i];
			cvkf_data->Xopt[i]  = Xopt[i];
		}
		cvkf_data->PLast[i] = PLast[i];
	}
}


/**
  * @brief      Initialization of CV Kalman filter pitch 
  * @param      cvkf_data: Initialized Kalman filter structure
  * @param      init_angle_pitch: Initialized pitch angle
  * @retval     NULL
  */
void Kalman_CVInitSetPitch(Kalman_CVKalmanInitDataTypeDef *cvkf_data, const float init_angle_pitch) {
	static float XLast[2] = {0.0f};
		         XLast[0] = init_angle_pitch;
	static float Xpre[2]  = {0.0f};
		         XLast[0] = init_angle_pitch;
	static float Xopt[2]  = {0.0f};
		         XLast[0] = init_angle_pitch;
	static float PLast[4] = {1.0f, 0.0f,
							 0.0f, 1.0f};	
	for (int i = 0; i < 4; i++) {
		if (i < 2) {
			cvkf_data->XLast[i] = XLast[i];
			cvkf_data->Xpre[i]  = Xpre[i];
			cvkf_data->Xopt[i]  = Xopt[i];
		}
		cvkf_data->PLast[i] = PLast[i];
	}
}


/**
  * @brief      Initialize cvkf structure and allocate memory space
  * @param      cvkf: Kalman filter structure
  * @param      cvkf_data: Initialized Kalman filter structure
  * @retval     NULL
  */
void Kalman_CVKalmanInit(Kalman_CVKalmanTypeDef *cvkf, Kalman_CVKalmanInitDataTypeDef *cvkf_data) {
	mat_init(&cvkf->KF_A,	2,2,(float *)cvkf_data->KF_A);
	mat_init(&cvkf->KF_C,	1,2,(float *)cvkf_data->KF_C);
	mat_init(&cvkf->XLast,  2,1,(float *)cvkf_data->XLast);
	mat_init(&cvkf->Xpre,	2,1,(float *)cvkf_data->Xpre);
	mat_init(&cvkf->Xopt,	2,1,(float *)cvkf_data->Xopt);
	mat_init(&cvkf->PLast,  2,2,(float *)cvkf_data->PLast);
	mat_init(&cvkf->Ppre,	2,2,(float *)cvkf_data->Ppre);
	mat_init(&cvkf->Popt,	2,2,(float *)cvkf_data->Popt);
	mat_init(&cvkf->Q,		2,2,(float *)cvkf_data->Q);
	mat_init(&cvkf->R,		1,1,(float *)cvkf_data->Q);
	mat_init(&cvkf->Kf,		2,1,(float *)cvkf_data->Kf);
	// cvkf->cvkf_t = cvkf_data->cvkf_t;
	cvkf->angle = cvkf_data->Xopt[0];
	cvkf->angle_p_err   = 0.0f;
	cvkf->switch_mode   = 1;	        // Default: OPEN
	cvkf->measure_mode  = 0;            // Default: None Update Measurement
	cvkf->targer_change = 0;            // Default: Target Follow no change
}


/**
  * @brief      Turn off Kalman filter
  * @param      cvkf: Kalman filter structure
  * @retval     NULL
  */
void Kalman_TurnOffCVKF(Kalman_CVKalmanTypeDef *cvkf) {
	cvkf->switch_mode = 0;
}


/**
  * @brief      Turn on Kalman filter
  * @param      cvkf: Kalman filter structure
  * @retval     NULL
  */
void Kalman_TurnOnCVKF(Kalman_CVKalmanTypeDef *cvkf) {
	cvkf->switch_mode = 1;
}


/**
  * @brief      Turn on Kalman filter update
  * @param      cvkf: Kalman filter structure
  * @retval     NULL
  */
void Kalman_TurnOnMeasureUpdate(Kalman_CVKalmanTypeDef *cvkf) {
	cvkf->measure_mode = 1;
}


/**
  * @brief      The model is used for prediction and variance iteration
  * @param      cvkf: Kalman filter structure
  * @retval     NULL
  */
void Kalman_CalcPredict(Kalman_CVKalmanTypeDef *cvkf) {
	static float _temp1[4] = {0.0f};
	static float _temp2[4] = {0.0f};
	mat _t1, _t2;
	mat_init(&_t1, 2, 2, _temp1);
	mat_init(&_t2, 2, 2, _temp2);

	// X_pre = A*X_last
	mat_mult(&cvkf->KF_A, &cvkf->XLast, &cvkf->Xpre);

	// PPre = A*PLast*A'+ Q;
	mat_mult(&cvkf->KF_A, &cvkf->PLast, &cvkf->Ppre);   // Ppre = A*Plast
	mat_trans(&cvkf->KF_A, &_t1);	                    // _temp = A'
	mat_mult(&cvkf->Ppre, &_t1, &_t2);                  // Ppre = Ppre*_temp
	mat_add(&_t2, &cvkf->Q, &cvkf->Ppre);               // PPre = PPre+ Q
     
	// Set Best Angle:
	cvkf->angle = cvkf->Xpre.pData[0];
}


/**
  * @brief      Calculation of Kalman filter gain
  * @param      cvkf: Kalman filter structure
  * @retval     NULL
  */
void Kalman_CalcKFGain(Kalman_CVKalmanTypeDef *cvkf) {
	static float _temp1[2] = {0.0f};
	static float _temp2[2] = {0.0f};
	static float _temp3[1] = {0.0f};
	static float _temp4[1] = {0.0f};
	static float _temp5[2] = {0.0f};
	mat _t1, _t2, _t3, _t4, _t5;
	mat_init(&_t1, 1, 2, _temp1);
	mat_init(&_t2, 2, 1, _temp2);
	mat_init(&_t3, 1, 1, _temp3);
	mat_init(&_t4, 1, 1, _temp4);
	mat_init(&_t5, 2, 1, _temp5);

	// Kf = PPre*C'/(C*PPre*C'+ R);
	mat_mult(&cvkf->KF_C, &cvkf->Ppre, &_t1);   // PLast = C*Ppre
	mat_trans(&cvkf->KF_C, &_t2);	            // _temp = C'
	mat_mult(&_t1, &_t2, &_t3);                 // Kf = C*Ppre*C'
	mat_add(&_t3, &cvkf->R, &_t4);              // Kf = (C*PPre*C'+ R);
	mat_inv(&_t4, &_t3);                        // Kf = inv(Kf)
	mat_mult(&cvkf->Ppre, &_t2, &_t5);          // _t1 = PPre*C'
	mat_mult(&_t5, &_t3, &cvkf->Kf);            // Kf = PPre*C'*inv(C*PPre*C'+ R)
	
}


/**
  * @brief      Using measurement data to update and predict optimal state estimation
  * @param      cvkf: Kalman filter structure
  * @param      angle: Measured angle
  * @retval     NULL
  */
void Kalman_CalcCorrect(Kalman_CVKalmanTypeDef * cvkf, const float angle) {
	// Init Measurement Mat:
	static float _ym[1]    = {0.0f};
                 _ym[0]    = angle;
	static float _temp1[1] = {0.0f};
	static float _temp2[1] = {0.0f};
	static float _temp3[2] = {0.0f};
	mat Ym, _t1, _t2, _t3;
	mat_init(&Ym, 1, 1, _ym);
	mat_init(&_t1, 1, 1, _temp1);
	mat_init(&_t2, 1, 1, _temp2);
	mat_init(&_t3, 1, 1, _temp3);

	// XOpt = XPre + Kf*(Ym - C*XPre):
	mat_mult(&cvkf->KF_C, &cvkf->Xpre, &_t1);
	mat_sub(&Ym, &_t1, &_t2);
	mat_mult(&cvkf->Kf, &_t2, &_t3);
	mat_add(&cvkf->XLast, &_t3, &cvkf->Xopt);
	
	// Init Eye:
	static float _eye[] = {1, 0, 0, 1};
	static float _temp4[4] = {0.0f};
	mat Eye, _t4;
	mat_init(&Eye, 2, 2, _eye);
	mat_init(&_t4, 2, 2, _temp4);

	//POpt = (eye(length(XOpt))-Kf*C)*PPre:
	mat_mult(&cvkf->Kf, &cvkf->KF_C, &cvkf->Popt);
	mat_sub(&Eye, &cvkf->Popt, &_t4);
	mat_mult(&_t4, &cvkf->Ppre, &cvkf->Popt);
    
	//Set Best Angle:
	cvkf->angle = cvkf->Xopt.pData[0];
}


/**
  * @brief      The last state is set to the optimal value to prepare for the next iteration
  * @param      cvkf: Kalman filter structure
  * @retval     NULL
  */
void Kalman_Update(Kalman_CVKalmanTypeDef *cvkf) {
	// Calculate the variance corresponding to the angle state
	cvkf->angle_p_err = fabs(cvkf->PLast.pData[0] - cvkf->Popt.pData[0]);
	
	int count = cvkf->PLast.numCols*cvkf->PLast.numRows;
	for(int i = 0; i < count; i++) {
		cvkf->PLast.pData[i] = cvkf->Popt.pData[i];
	}
	count = cvkf->XLast.numCols * cvkf->XLast.numRows;
	for(int i = 0; i < count; i++) {
		cvkf->XLast.pData[i] = cvkf->Xopt.pData[i];
	}
}


/**
  * @brief      KF filtering using measured data normally
  * @param      cvkf: Kalman filter structure
  * @param      angle: Measured angle
  * @retval     NULL
  */
void Kalman_MeasurementCalc(Kalman_CVKalmanTypeDef *cvkf, const float angle) {
	Kalman_CalcPredict(cvkf);
	Kalman_CalcKFGain(cvkf);
	Kalman_CalcCorrect(cvkf, angle);
	Kalman_Update(cvkf);
	cvkf->measure_mode = 0;         // After Correction
}


/**
  * @brief      KF filtering without measured data normally
  * @param      cvkf: Kalman filter structure
  * @param      angle: Measured angle
  * @retval     NULL
  */
void Kalman_NonMeasurementCalc(Kalman_CVKalmanTypeDef * cvkf) {
	Kalman_CalcPredict(cvkf);
}



/**
  * @brief      Forecast N cycles without variance iteration
  * @param      cvkf: Kalman filter structure
  * @param      nT: Forecast period
  * @retval     Mat: [angle_yaw omega_yaw angle_pitch omega_pitch]
  */
float Kalman_Predict_nT(Kalman_CVKalmanTypeDef * cvkf, int nT) {
	if (nT == 0)	// None Prediction
		return cvkf->angle;
	float _temp1[2] = {0.0f};
	float _temp2[2] = {0.0f};
	mat _t1, _t2;
	mat_init(&_t1, 2, 1, _temp1);
	mat_init(&_t2, 2, 1, _temp2);
	
	mat_mult(&cvkf->KF_A, &cvkf->XLast, &_t1);
	for (int i = 0; i < nT-1 ; i++) {
		// X_pre = A * X_last
		mat_mult(&cvkf->KF_A, &_t1, &_t2);        // t2 = A * t1
		_t1.pData[0] = _t2.pData[0];
		_t1.pData[1] = _t2.pData[1];
	}
	return (float)_t1.pData[0]; 
}


/**
  * @brief      Forecast N cycles without variance iteration
  * @param      cvkf: Kalman filter structure
  * @param      m_angle: Forecast period
  * @retval     last_target: last terget angle
  */
float Kalman_JudgeChange(Kalman_CVKalmanTypeDef *cvkf, const float m_angle) {
	int nT = 10;
	static uint32_t target_change_time = 0;
	static float last_target = 0.0f;
	float angle_predict = Kalman_Predict_nT(cvkf, nT);
	// Whether Large Gab Change Appear?
	if (fabs(m_angle - cvkf->angle) >= fabs(angle_predict - cvkf->angle) + 1.5f) {
		uint32_t change_now = HAL_GetTick();
		if ((int)(change_now - target_change_time) >= 500 || (int)(change_now - target_change_time)<= -500) {
			cvkf->targer_change = 1;
			last_target = m_angle;
		}
	} else {
		target_change_time = HAL_GetTick();
		last_target = m_angle;
		cvkf->targer_change = 0;
	}
	
	return last_target;
}




