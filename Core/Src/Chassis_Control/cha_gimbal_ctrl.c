/*
 *  Project      : Infantry_Momentum
 *
 *  file         : cha_gimbal_ctrl.c
 *  Description  : This file contains Gimbal Yaw control function
 *  LastEditors  : ����ؼ���ᶯ��
 *  Date         : 2021-05-04 20:53:31
 *  LastEditTime : 2021-05-08 08:27:37
 */

#include "cha_gimbal_ctrl.h"

#if __FN_IF_ENABLE(__FN_CTRL_GIMBAL_YAW_CHA)

#include "buscomm_ctrl.h"
#include "const_lib.h"

Motor_MotorParamTypeDef GimbalYaw_gimbalYawMotorParamBigEnergy;
Motor_MotorParamTypeDef GimbalYaw_gimbalYawMotorParamSmallEnergy;
Motor_MotorParamTypeDef GimbalYaw_gimbalYawMotorParamArmor;
Motor_MotorParamTypeDef GimbalYaw_gimbalYawMotorParamIMUDebug;
Motor_MotorParamTypeDef GimbalYaw_gimbalYawMotorParamNoAuto;

GimbalYaw_GimbalYawTypeDef GimbalYaw_gimbalYawControlData;

/**
 * @brief      Gimbal yaw control initialization
 * @param      NULL
 * @retval     NULL
 */
void GimbalYaw_InitGimbalYaw() {
    GimbalYaw_GimbalYawTypeDef* gimbalyaw = GimbalYaw_GetGimbalYawPtr();

    gimbalyaw->control_state = 1;
    gimbalyaw->output_state = 1;
    gimbalyaw->mode_changed = 0;
    gimbalyaw->yaw_ref = 0;
    gimbalyaw->yaw_count = 0;
    gimbalyaw->mode = GimbalYaw_MODE_IMU_DEBUG;
    gimbalyaw->last_mode = GimbalYaw_MODE_IMU_DEBUG;

    Motor_gimbalMotorYaw.encoder.angle = 4000;

    // Initialization of motor parameters (including PID parameters)
    Const_SetGimbalYawMotorParam();
    Filter_LowPassInit(0.2, &gimbalyaw->ref_fil_param);
}

/**
 * @brief      Get the pointer of gimbal control object
 * @param      NULL
 * @retval     Pointer to gimbal control object
 */
GimbalYaw_GimbalYawTypeDef* GimbalYaw_GetGimbalYawPtr() {
    return &GimbalYaw_gimbalYawControlData;
}

/**
 * @brief      Set the gimbal control output calculation enabled state
 * @param      state: Enabled, 1 is enabled, 0 is disabled
 * @retval     NULL
 */
void GimbalYaw_SetGimbalYawControlState(uint8_t state) {
    GimbalYaw_GimbalYawTypeDef* gimbalYaw = GimbalYaw_GetGimbalYawPtr();

    gimbalYaw->control_state = state;
}

/**
 * @brief      Set gimbal control output enable status
 * @param      state: Enabled, 1 is enabled, 0 is disabled
 * @retval     NULL
 */
void GimbalYaw_SetGimbalYawOutputState(uint8_t state) {
    GimbalYaw_GimbalYawTypeDef* gimbalYaw = GimbalYaw_GetGimbalYawPtr();

    gimbalYaw->output_state = state;
}

/**
 * @brief      Set up the yaw mode of gimbal
 * @param      mode: gimbal yaw mode
 * @retval     NULL
 */
void GimbalYaw_SetMode(GimbalYaw_GimbalYawModeEnum mode) {
    GimbalYaw_GimbalYawTypeDef* gimbalyaw = GimbalYaw_GetGimbalYawPtr();

    gimbalyaw->last_mode = gimbalyaw->mode;
    gimbalyaw->mode = mode;

    if (gimbalyaw->last_mode != gimbalyaw->mode)
        gimbalyaw->mode_changed = 1;
}

/**
 * @brief      Set the motor encoder as yaw fb
 * @param      NULL
 * @retval     NULL
 */
void GimbalYaw_SetEncoderFdb() {
    GimbalYaw_GimbalYawTypeDef* gimbalyaw = GimbalYaw_GetGimbalYawPtr();

    gimbalyaw->yaw_position_fdb = Motor_gimbalMotorYaw.encoder.limited_angle - Const_YAW_MOTOR_INIT_OFFSET;
    gimbalyaw->yaw_speed_fdb = Motor_gimbalMotorYaw.encoder.speed;
}

/**
 * @brief      Set the target value of gimbal yaw
 * @param      yaw_ref: gimbal yaw target value
 * @retval     NULL
 */
void GimbalYaw_SetYawRef(float yaw_ref) {
    GimbalYaw_GimbalYawTypeDef* gimbalyaw = GimbalYaw_GetGimbalYawPtr();

    gimbalyaw->yaw_ref = yaw_ref;
}

/**
 * @brief      Setting IMU yaw position feedback
 * @param      imu_yaw_position_fdb: IMU Yaw Position feedback
 * @retval     NULL
 */
void GimbalYaw_SetIMUYawPositionFdb(float imu_yaw_position_fdb) {
    GimbalYaw_GimbalYawTypeDef* gimbalyaw = GimbalYaw_GetGimbalYawPtr();

    gimbalyaw->yaw_position_fdb = imu_yaw_position_fdb;
}

/**
 * @brief      Setting IMU yaw speed feedback
 * @param      imu_yaw_speed_fdb: IMU Yaw Speed feedback
 * @retval     NULL
 */
void GimbalYaw_SetIMUYawSpeedFdb(float imu_yaw_speed_fdb) {
    GimbalYaw_GimbalYawTypeDef* gimbalyaw = GimbalYaw_GetGimbalYawPtr();

    gimbalyaw->yaw_speed_fdb = imu_yaw_speed_fdb;
}

/**
 * @brief      Control function of gimbal yaw
 * @param      NULL
 * @retval     NULL
 */
void GimbalYaw_Control() {
    GimbalYaw_GimbalYawTypeDef* gimbalyaw = GimbalYaw_GetGimbalYawPtr();

    if (gimbalyaw->control_state != 1)
        return;

    // Clear PID when mode changes
    if (gimbalyaw->mode_changed == 1) {
        Motor_ResetMotorPID(&Motor_gimbalMotorYaw);
        gimbalyaw->mode_changed = 0;
    }

    // Set pid param
    Motor_MotorParamTypeDef* pparam;
    switch (gimbalyaw->mode) {
        case GimbalYaw_MODE_BIG_ENERGY:
            pparam = &GimbalYaw_gimbalYawMotorParamBigEnergy;
            break;
        case GimbalYaw_MODE_SMALL_ENERGY:
            pparam = &GimbalYaw_gimbalYawMotorParamSmallEnergy;
            break;
        case GimbalYaw_MODE_ARMOR:
            pparam = &GimbalYaw_gimbalYawMotorParamArmor;
            break;
        case GimbalYaw_MODE_IMU_DEBUG:
            pparam = &GimbalYaw_gimbalYawMotorParamIMUDebug;
            break;
        case GimbalYaw_MODE_NO_AUTO:
            pparam = &GimbalYaw_gimbalYawMotorParamNoAuto;
            break;
        default:
            break;
    }

    float yaw_ref;
    yaw_ref = Filter_LowPass(gimbalyaw->yaw_ref, &gimbalyaw->ref_fil_param, &gimbalyaw->ref_fil);

    Motor_SetMotorRef(&Motor_gimbalMotorYaw, yaw_ref);
    Motor_SetMotorFdb(&Motor_gimbalMotorYaw, 2, gimbalyaw->yaw_position_fdb);
    Motor_SetMotorFdb(&Motor_gimbalMotorYaw, 1, gimbalyaw->yaw_speed_fdb);
    Motor_CalcMotorOutput(&Motor_gimbalMotorYaw, pparam);
}

/**
 * @brief      Gimbal yaw output function
 * @param      NULL
 * @retval     NULL
 */
void GimbalYaw_Output() {
    GimbalYaw_GimbalYawTypeDef* gimbalyaw = GimbalYaw_GetGimbalYawPtr();

    if (gimbalyaw->output_state != 1)
        return;
    Motor_SendMotorGroupOutput(&Motor_gimbalMotors);
}

#endif
