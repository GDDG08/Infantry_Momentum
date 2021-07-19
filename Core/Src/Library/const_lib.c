/*
 *  Project      : Infantry_Momentum
 * 
 *  file         : const_lib.c
 *  Description  : This file contains all necessary constants
 *  LastEditors  : 动情丶卜灬动心
 *  Date         : 2021-05-04 20:53:31
 *  LastEditTime : 2021-05-16 19:18:58
 */

#include "const_lib.h"

#include "can.h"
#include "cha_chassis_ctrl.h"
#include "cha_gimbal_ctrl.h"
#include "gim_gimbal_ctrl.h"
#include "gim_shoot_ctrl.h"
#include "motor_periph.h"
#include "pid_alg.h"
#include "supercap_ctrl.h"
#include "usart.h"

/*      Super Cap Const         */

#if __FN_IF_ENABLE(__FN_SUPER_CAP)

/*          ADC Control related constants       */
const float Const_ADC_V_VGAIN = 11.0f;               // Voltage value division ratio 10:1
const float Const_ADC_V_C_HolzerGAIN = 5.0f;         // Gain of Hall current sensor（chassis referee）
const float Const_ADC_V_C_BuckOutResGAIN = 6.25f;    // Buck output current sensing gain（LT3790）
const float Const_ADC_V_C_BuckInputResGAIN = 25.0f;  // Buck input current sensing gain（LT3790）
const float Const_ADC_Cap_TotalEnergy = 2000.0f;     // Total capacitance energy
const float Const_ADC_CapValue = 6.0f;               // Minimum voltage of capacitor
const float Const_ADC_CurrentErrorVoltage = 0.0f;    // Current sensor error

/*          DAC Control related constants       */
const float Const_DAC_GAIN = 20.0f;        // DAC current set gain（LT3790）
const float Const_DAC_DetectRES = 0.004f;  // DAC current set resistor（LT3790）

/*          CAN Handle definition              */
// CAN_HandleTypeDef* Const_BusComm_CAN_HANDLER        = &hcan2;
UART_HandleTypeDef* Const_SuperCap_UART_HANDLER = &huart5;

/*          Super Cap control const             */
const float Cap_MinVoltage = 15.0f;          // Cap min voltage
const float Cap_ChargeReservedPower = 5.0f;  // Cap charge reserved power
const float Cap_AvailableVoltage = 21.0f;    // Cap restart voltage

#endif

/*      infantry chasiss const                  */

#if __FN_IF_ENABLE(__FN_INFANTRY_CHASSIS)

/*          Uart Handle definition              */
UART_HandleTypeDef* Const_Referee_UART_HANDLER = &huart6;
CAN_HandleTypeDef* Const_BusComm_CAN_HANDLER = &hcan2;
UART_HandleTypeDef* Const_SuperCap_UART_HANDLER = &huart3;

/*          Motor control constant              */
#if __FN_IF_ENABLE(__FN_INFANTRY_NEW_ONE)
const float Const_YAW_MOTOR_INIT_OFFSET = -147.0f;
#endif

#if __FN_IF_ENABLE(__FN_INFANTRY_NEW_TWO)
const float Const_YAW_MOTOR_INIT_OFFSET = -62.7f;
#endif

#if __FN_IF_ENABLE(__FN_INFANTRY_NEW_THREE)
const float Const_YAW_MOTOR_INIT_OFFSET = -150.0f;
#endif

/*               constant                       */
static const float Const_chassisMotorParam[4][3][4][5] = {
    {{{0, 0, 0, 0, 0}, {-1, -1}, {0, 0}, {-1, -1}}, {{75, 0, 0.5, 0, 13000}, {-1, -1}, {0, 0}, {-1, -1}}, {{0, 0, 0, 0, 0}, {-1, -1}, {0, 0}, {-1, -1}}},   // Chassis_chassisMotorParamStop
    {{{0, 0, 0, 0, 0}, {-1, -1}, {0, 0}, {-1, -1}}, {{75, 0, 0.5, 0, 11000}, {-1, -1}, {0, 0}, {-1, -1}}, {{0, 0, 0, 0, 0}, {-1, -1}, {0, 0}, {-1, -1}}},   // Chassis_chassisMotorParamNormal
    {{{0, 0, 0, 0, 0}, {-1, -1}, {0, 0}, {-1, -1}}, {{75, 0, 0.5, 0, 13000}, {-1, -1}, {0, 0}, {-1, -1}}, {{0, 0, 0, 0, 0}, {-1, -1}, {0, 0}, {-1, -1}}},   // Chassis_chassisMotorParamGyro
    {{{0, 0, 0, 0, 0}, {-1, -1}, {0, 0}, {-1, -1}}, {{0, 0, 0, 0, 0}, {-1, -1}, {0, 0}, {-1, -1}}, {{10, 0.01, 0, 1000, 400}, {-1, -1}, {0, 0}, {-1, -1}}}  // Chassis_followPIDParam
    //  {           Current  PID                          }   {                 SPEED PID                       }  {                    POSITION PID                 }
    //  {Kp, Ki, Kd, SumMax, OutMax}, {d_fil,delta_fil},{kf_1,kf_2}, {kf1_fil_param,kf2_fil_param};
    //              PID Group            PID Filter    Feedforward      Feedforward Filter
};

static const float Const_gimbalYawMotorParam[5][3][4][5] = {
    {{{1.1, 0.1, 0, 10000, 26000}, {-1, -1}, {0, 0}, {-1, -1}}, {{320, 0.8, 10, 10000, 30000}, {0.3, -1}, {0, 0}, {-1, -1}}, {{11, 0.02, 0, 500, 1000}, {-1, -1}, {50, 100}, {0.8, 0.7}}},  // GimbalYaw_gimbalYawMotorParamAimBigEnergy
    {{{1.1, 0.1, 0, 10000, 26000}, {-1, -1}, {0, 0}, {-1, -1}}, {{320, 0.8, 10, 10000, 30000}, {0.3, -1}, {0, 0}, {-1, -1}}, {{11, 0.02, 0, 500, 1000}, {-1, -1}, {50, 100}, {0.8, 0.7}}},  // GimbalYaw_gimbalYawMotorParamAimSmallEnergy
    {{{1.1, 0.1, 0, 10000, 26000}, {-1, -1}, {0, 0}, {-1, -1}}, {{320, 0.8, 10, 10000, 30000}, {0.3, -1}, {0, 0}, {-1, -1}}, {{11, 0.02, 0, 500, 1000}, {-1, -1}, {50, 100}, {0.8, 0.7}}},  // GimbalYaw_gimbalYawMotorParamArmor
    {{{1.1, 0.1, 0, 10000, 26000}, {-1, -1}, {0, 0}, {-1, -1}}, {{320, 0.8, 10, 10000, 30000}, {0.3, -1}, {0, 0}, {-1, -1}}, {{11, 0.02, 0, 500, 1000}, {-1, -1}, {50, 100}, {0.8, 0.7}}},  // GimbalYaw_gimbalYawMotorParamIMUDebug
    {{{1.1, 0.1, 0, 10000, 26000}, {-1, -1}, {0, 0}, {-1, -1}}, {{320, 0.8, 10, 10000, 30000}, {0.3, -1}, {0, 0}, {-1, -1}}, {{11, 0.02, 0, 500, 1000}, {-1, -1}, {50, 100}, {0.8, 0.7}}}   // GimbalYaw_gimbalYawMotorParamNoAuto
    //  {           Current  PID                          }   {                 SPEED PID                       }  {                    POSITION PID                 }
    //  {Kp, Ki, Kd, SumMax, OutMax}, {d_fil,delta_fil},{kf_1,kf_2}, {kf1_fil_param,kf2_fil_param};
    //              PID Group            PID Filter    Feedforward      Feedforward Filter
};

void Const_SetChasisMotorParam() {
    Motor_InitMotorParam(&Chassis_chassisMotorParamStop, Const_chassisMotorParam[0], PID_POSITION, PID_POSITION, PID_POSITION);
    Motor_InitMotorParam(&Chassis_chassisMotorParamNormal, Const_chassisMotorParam[1], PID_POSITION, PID_POSITION, PID_POSITION);
    Motor_InitMotorParam(&Chassis_chassisMotorParamGyro, Const_chassisMotorParam[2], PID_POSITION, PID_POSITION, PID_POSITION);
    PID_InitPIDParam(&Chassis_followPIDParam, Const_chassisMotorParam[3][2][0][0], Const_chassisMotorParam[3][2][0][1], Const_chassisMotorParam[3][2][0][2], Const_chassisMotorParam[3][2][0][3],
                     Const_chassisMotorParam[3][2][0][4], Const_chassisMotorParam[3][2][1][0], Const_chassisMotorParam[3][2][1][1], Const_chassisMotorParam[3][2][2][0],
                     Const_chassisMotorParam[3][2][2][1], Const_chassisMotorParam[3][2][3][0], Const_chassisMotorParam[3][2][3][1], PID_POSITION);
}

void Const_SetGimbalYawMotorParam() {
    Motor_InitMotorParam(&GimbalYaw_gimbalYawMotorParamBigEnergy, Const_gimbalYawMotorParam[0], PID_POSITION, PID_POSITION, PID_POSITION);
    Motor_InitMotorParam(&GimbalYaw_gimbalYawMotorParamSmallEnergy, Const_gimbalYawMotorParam[1], PID_POSITION, PID_POSITION, PID_POSITION);
    Motor_InitMotorParam(&GimbalYaw_gimbalYawMotorParamArmor, Const_gimbalYawMotorParam[2], PID_POSITION, PID_POSITION, PID_POSITION);
    Motor_InitMotorParam(&GimbalYaw_gimbalYawMotorParamIMUDebug, Const_gimbalYawMotorParam[3], PID_POSITION, PID_POSITION, PID_POSITION);
    Motor_InitMotorParam(&GimbalYaw_gimbalYawMotorParamNoAuto, Const_gimbalYawMotorParam[4], PID_POSITION, PID_POSITION, PID_POSITION);
}

/*          Chassis control filter const        */
const float Const_Chassis_MOVE_REF_TO_MOTOR_REF = 0.7f;
const float Const_Chassis_ROTATE_REF_TO_MOTOR_REF = 0.6f;

#endif

/*      infantry gimbal const       */

#if __FN_IF_ENABLE(__FN_INFANTRY_GIMBAL)

/*          Remote bessel_filter                */
Filter_Bessel_TypeDef Remote_forward_backFilter = {0, 0, 0};
Filter_Bessel_TypeDef Remote_right_leftFilter = {0, 0, 0};
Filter_Bessel_TypeDef Remote_mouse_y_Filter = {0, 0, 0};

/*          Remote control const                */
const float REMOTE_PITCH_ANGLE_TO_REF = 0.0015f;
const float REMOTE_YAW_ANGLE_TO_REF = 0.0015f;
const float MOUSE_PITCH_ANGLE_TO_FACT = 0.008f;
const float MOUSE_YAW_ANGLE_TO_FACT = 0.015f;
const float MOUSE_CHASSIS_ACCELERATE = 0.5;
const float MOUSE_CHASSIS_SLOWDOWN = 0.5;
const float MOUSE_CHASSIS_MAX_SPEED = 600;
const float MOUSE_CHASSIS_MAX_GYRO_SPEED = 400;

const uint32_t Const_MiniPC_Follow_Target_Time = 150;
const uint32_t Const_MiniPC_Lost_Target_Time = 150;
//const uint32_t Const_MiniPC_New_Target_Time           = 200;

/*          Uart Handle definition              */
UART_HandleTypeDef* Const_IMU_UART_HANDLER = &huart3;
UART_HandleTypeDef* Const_Remote_UART_HANDLER = &huart1;
UART_HandleTypeDef* Const_MiniPC_UART_HANDLER = &huart5;
CAN_HandleTypeDef* Const_BusComm_CAN_HANDLER = &hcan2;

/*          Gimbal pitch limit                  */
const float Const_PITCH_UMAXANGLE = 10.0f;
const float Const_PITCH_UMAXANGLE_GRYO = 8.0f;
const float Const_PITCH_DMAXANGLE = -31.5f;
const float Const_YAW_MAXANGLE = 40.0f;
const float Const_PITCH_MOTOR_INIT_OFFSET = -31.0f;
const float Const_SERVO_INIT_OFFSET = 0.05f;

#if __FN_IF_ENABLE(__FN_INFANTRY_NEW_ONE)
const float Const_YAW_MOTOR_INIT_OFFSET = 23.0f;
#endif

#if __FN_IF_ENABLE(__FN_INFANTRY_NEW_TWO)
const float Const_YAW_MOTOR_INIT_OFFSET = -60.0f;
#endif

#if __FN_IF_ENABLE(__FN_INFANTRY_NEW_THREE)
const float Const_YAW_MOTOR_INIT_OFFSET = -150.0f;
#endif

static const float Const_gimbalPitchMotorParam[5][3][4][5] = {
    {{{1.1, 0.1, 0, 8000, 30000}, {-1, -1}, {0, 0}, {-1, -1}}, {{190, 0.2, 20, 15000, 30000}, {0.4, -1}, {0, 0}, {-1, -1}}, {{25, 0.08, 0, 100, 500}, {-1, -1}, {100, 160}, {0.18, 0.3}}},  // GimbalYaw_gimbalYawMotorParamAimBigEnergy
    {{{1.1, 0.1, 0, 8000, 30000}, {-1, -1}, {0, 0}, {-1, -1}}, {{190, 0.2, 20, 15000, 30000}, {0.4, -1}, {0, 0}, {-1, -1}}, {{25, 0.08, 0, 100, 500}, {-1, -1}, {100, 160}, {0.18, 0.3}}},  // GimbalYaw_gimbalYawMotorParamAimSmallEnergy
    {{{1.1, 0.1, 0, 8000, 30000}, {-1, -1}, {0, 0}, {-1, -1}}, {{190, 0.2, 20, 15000, 30000}, {0.4, -1}, {0, 0}, {-1, -1}}, {{25, 0.08, 0, 100, 500}, {-1, -1}, {100, 160}, {0.18, 0.3}}},  // GimbalYaw_gimbalYawMotorParamArmor
    {{{1.1, 0.1, 0, 8000, 30000}, {-1, -1}, {0, 0}, {-1, -1}}, {{190, 0.2, 20, 15000, 30000}, {0.4, -1}, {0, 0}, {-1, -1}}, {{25, 0.08, 0, 100, 500}, {-1, -1}, {100, 160}, {0.18, 0.3}}},  // GimbalYaw_gimbalYawMotorParamIMUDebug
    {{{1.1, 0.1, 0, 8000, 30000}, {-1, -1}, {0, 0}, {-1, -1}}, {{190, 0.2, 20, 15000, 30000}, {0.4, -1}, {0, 0}, {-1, -1}}, {{25, 0.08, 0, 100, 500}, {-1, -1}, {100, 160}, {0.18, 0.3}}}   // GimbalYaw_gimbalYawMotorParamNoAuto
    //  {           Current  PID                          }   {                 SPEED PID                       }  {                    POSITION PID                 }
    //  {Kp, Ki, Kd, SumMax, OutMax},{d_fil,delta_fil},{kf_1,kf_2},{kf1_fil_param,kf2_fil_param};
    //              PID Group            PID Filter    Feedforward      Feedforward Filter

    // {{1.05,  0.1,  0,  8000, 26000}, {0.1, 0},  {0, 0}, {0, 0} }, {220, 0.4, 0, 15000, 30000, 0.1, 0, 0, 0, 0}, {11, 0, 0, 1000, 1000, 0.1, 0.55, 6, 1, 0.02}}
    // Position PID param
};

void Const_SetGimbalPitchMotorParam() {
    Motor_InitMotorParam(&GimbalPitch_gimbalPitchMotorParamBigEnergy, Const_gimbalPitchMotorParam[0], PID_POSITION, PID_POSITION, PID_POSITION);
    Motor_InitMotorParam(&GimbalPitch_gimbalPitchMotorParamSmallEnergy, Const_gimbalPitchMotorParam[1], PID_POSITION, PID_POSITION, PID_POSITION);
    Motor_InitMotorParam(&GimbalPitch_gimbalPitchMotorParamArmor, Const_gimbalPitchMotorParam[2], PID_POSITION, PID_POSITION, PID_POSITION);
    Motor_InitMotorParam(&GimbalPitch_gimbalPitchMotorParamIMUDebug, Const_gimbalPitchMotorParam[3], PID_POSITION, PID_POSITION, PID_POSITION);
    Motor_InitMotorParam(&GimbalPitch_gimbalPitchMotorParamNoAuto, Const_gimbalPitchMotorParam[4], PID_POSITION, PID_POSITION, PID_POSITION);
}

static const float Const_ShooterMotorParam[2][3][4][5] = {
    {{{0, 0, 0, 0, 0}, {-1, -1}, {0, 0}, {-1, -1}}, {{20, 0.05, 0.5, 10000, 20000}, {-1, -1}, {0, 0}, {-1, -1}}, {{0, 0, 0, 0, 0}, {-1, -1}, {0, 0}, {-1, -1}}},  // Left shooter motor
    {{{0, 0, 0, 0, 0}, {-1, -1}, {0, 0}, {-1, -1}}, {{20, 0.05, 0.5, 10000, 20000}, {-1, -1}, {0, 0}, {-1, -1}}, {{0, 0, 0, 0, 0}, {-1, -1}, {0, 0}, {-1, -1}}}   // Right shooter motor
};

static const float Const_FeederMotorParam[1][3][4][5] = {
    {{{0, 0, 0, 0, 0}, {-1, -1}, {0, 0}, {-1, -1}}, {{750, 0.01, 0, 10000, 20000}, {-1, -1}, {0, 0}, {-1, -1}}, {{8.35, 0, 0.11, 10000, 20000}, {-1, -1}, {0, 0}, {-1, -1}}}  // feeder motor
};

void Const_SetShooterPIDParam() {
    Motor_InitMotorParam(&Shooter_shooterLeftMotorParam, Const_ShooterMotorParam[0], PID_POSITION, PID_POSITION, PID_POSITION);
    Motor_InitMotorParam(&Shooter_shooterRightMotorParam, Const_ShooterMotorParam[1], PID_POSITION, PID_POSITION, PID_POSITION);
    Motor_InitMotorParam(&Shooter_feederMotorParam, Const_FeederMotorParam[0], PID_POSITION, PID_POSITION, PID_POSITION);
}

float Const_ShooterLockedCurrent = 3000.0f;
float Const_ShooterLockedSpeed = 20.0f;
float Const_ShooterLockedTime = 200.0f;
float Const_ShooterRelockedTime = 500.0f;
float Const_ShooterLockedReverseSpeed = 0.0f;

float Const_ShooterSlowSpeed = 150.0f;
float Const_ShooterFastSpeed = 230.0f;

float Const_Shooter15mpers = 280.0f;
float Const_Shooter18mpers = 300.0f;
float Const_Shooter30mpers = 442.0f;

float Const_FeederSlowSpeed = 50.0f;
float Const_FeederFastSpeed = 100.0f;
float Const_FeederWaitSpeed = 10.0f;

uint16_t Const_HeatCtrlFastLimit = 75;
uint16_t Const_HeatCtrlSlowLimit = 40;
uint16_t Const_HeatCtrlWaitLimit = 10;
uint16_t Const_HeatCtrlSingleCount = 10;
uint16_t Const_HeatCtrlStopLimit = 10;

#endif
