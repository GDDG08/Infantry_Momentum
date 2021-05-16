/*
 *  Project      : Infantry_Momentum
 * 
 *  file         : const_lib.h
 *  Description  : This file contains all necessary constants
 *  LastEditors  : 动情丶卜灬动心
 *  Date         : 2021-05-04 20:53:31
 *  LastEditTime : 2021-05-14 12:13:23
 */

#ifndef CONST_LIB_H
#define CONST_LIB_H

#ifdef __cplusplus
extern "C" {
#endif

#include "configure.h"
#include "filter_alg.h"

/*      Super Cap Const         */

#if __FN_IF_ENABLE(__FN_SUPER_CAP)

/*          ADC Control related constants       */
extern const float Const_ADC_V_VGAIN;							//The voltage reading gain resistor divider ratio is 11
extern const float Const_ADC_V_C_HolzerGAIN; 				    //Hall current sensor gain (chassis current, referee system current)
extern const float Const_ADC_V_C_BuckOutResGAIN; 				//Buck output current sensing gain (lt3790)
extern const float Const_ADC_V_C_BuckInputResGAIN;				//Buck input current sensing gain (lt3790)
extern const float Const_ADC_Cap_TotalEnergy;					//Total capacitance energy
extern const float Const_ADC_CapValue;							//Capacity of capacitor bank
extern const float Const_ADC_CurrentErrorVoltage;               //ACS712 error


/*          DAC Control related constants       */
extern const float Const_DAC_GAIN;			                    //DAC Current setting gain (lt3790)
extern const float Const_DAC_DetectRES;			                //DAC Current setting resistor (lt3790)

/*          CAN Handle definition              */
// extern CAN_HandleTypeDef* Const_BusComm_CAN_HANDLER;
extern UART_HandleTypeDef* Const_SuperCap_UART_HANDLER;

/*          Super Cap control const             */
extern const float Cap_MinVoltage;
extern const float Cap_ChargeReservedPower;
extern const float Cap_AvailableVoltage;

#endif


/*      infantry chasiss const                  */

#if __FN_IF_ENABLE(__FN_INFANTRY_CHASSIS)
/*          Uart Handle definition              */
extern UART_HandleTypeDef* Const_Referee_UART_HANDLER;
extern CAN_HandleTypeDef* Const_BusComm_CAN_HANDLER;
extern UART_HandleTypeDef* Const_SuperCap_UART_HANDLER;
/*          Motor control constant              */
extern const float Const_YAW_MOTOR_INIT_OFFSET;

/*          Chassis control filter const        */
extern const float Const_Chassis_MOVE_REF_TO_MOTOR_REF;
extern const float Const_Chassis_ROTATE_REF_TO_MOTOR_REF;

void Const_SetChasisMotorParam(void);
void Const_SetGimbalYawMotorParam(void);

#endif


/*      infantry gimbal const       */

#if __FN_IF_ENABLE(__FN_INFANTRY_GIMBAL)

/*          Remote bessel_filter                */
extern Filter_Bessel_TypeDef Remote_forward_backFilter;
extern Filter_Bessel_TypeDef Remote_right_leftFilter;
extern Filter_Bessel_TypeDef Remote_mouse_y_Filter;

/*          Remote control const                */
extern const float REMOTE_PITCH_ANGLE_TO_REF;
extern const float REMOTE_YAW_ANGLE_TO_REF;
extern const float MOUSE_PITCH_ANGLE_TO_FACT;
extern const float MOUSE_YAW_ANGLE_TO_FACT;
extern const float MOUSE_CHASSIS_ACCELERATE;
extern const float MOUSE_CHASSIS_SLOWDOWN;
extern const float MOUSE_CHASSIS_MAX_SPEED;
extern const float MOUSE_CHASSIS_MAX_GYRO_SPEED;

extern const float Const_YAW_MOTOR_INIT_OFFSET;

/*          MiniPC_ctrl const                   */
extern const uint32_t Const_MiniPC_Follow_Target_Time;
extern const uint32_t Const_MiniPC_Lost_Target_Time;
extern const uint32_t Const_MiniPC_New_Target_Time;

/*          Uart Handle definition              */
extern UART_HandleTypeDef* Const_IMU_UART_HANDLER;
extern UART_HandleTypeDef* Const_Remote_UART_HANDLER;
extern UART_HandleTypeDef* Const_MiniPC_UART_HANDLER;
extern CAN_HandleTypeDef* Const_BusComm_CAN_HANDLER;

/*          Gimbal pitch limit                  */
extern const float Const_PITCH_UMAXANGLE;          
extern const float Const_PITCH_DMAXANGLE;        
extern const float Const_YAW_MAXANGLE;  
extern const float Const_YAW_MOTOR_INIT_OFFSET;
extern const float Const_PITCH_MOTOR_INIT_OFFSET; 
extern const float Const_SERVO_INIT_OFFSET;

void Const_SetGimbalPitchMotorParam(void);

/*          Shooter const                       */
void Const_SetShooterPIDParam(void);

extern float Const_ShooterLockedCurrent;
extern float Const_ShooterLockedSpeed;
extern float Const_ShooterLockedTime;
extern float Const_ShooterRelockedTime;
extern float Const_ShooterLockedReverseSpeed;

extern float Const_ShooterSlowSpeed;
extern float Const_ShooterFastSpeed;

extern float Const_Shooter15mpers;
extern float Const_Shooter18mpers;
extern float Const_Shooter30mpers; 

extern float Const_FeederSlowSpeed;
extern float Const_FeederFastSpeed;

extern uint16_t Const_HeatCtrlFastLimit;
extern uint16_t Const_HeatCtrlSlowLimit;
extern uint16_t Const_HeatCtrlSingleCount;
extern uint16_t Const_HeatCtrlStopLimit;

#endif

#ifdef __cplusplus
}
#endif

#endif
