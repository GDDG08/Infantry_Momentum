/*
 *  Project      : Infantry_Momentum
 *
 *  file         : buscomm_ctrl.h
 *  Description  : This file contains Bus bus communication control function
 *  LastEditors  : ����ؼ���ᶯ��
 *  Date         : 2021-05-04 20:53:31
 *  LastEditTime : 2021-05-16 01:21:15
 */

#ifndef BUSCOMM_CTRL_H
#define BUSCOMM_CTRL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "configure.h"

#include "cmsis_os.h"
#include "FreeRTOS.h"

#include "can_util.h"
#include "buff_lib.h"
#include "string.h"
#include "filter_alg.h"

extern const uint8_t Const_BusComm_FRAME_HEADER_SOF;
//      power limit mode
extern const uint8_t POWER_LIMITED;
extern const uint8_t POWER_UNLIMIT;
//      gimbal yaw mode
extern const uint8_t GIMBAL_YAW_CTRL_BIG_ENERGY;
extern const uint8_t GIMBAL_YAW_CTRL_SMALL_ENERGY;
extern const uint8_t GIMBAL_YAW_CTRL_ARMOR;
extern const uint8_t GIMBAL_YAW_CTRL_IMU_DEBUG;
extern const uint8_t GIMBAL_YAW_CTRL_NO_AUTO;
//      chassis mode
extern const uint8_t CHASSIS_CTRL_STOP;
extern const uint8_t CHASSIS_CTRL_NORMAL;
extern const uint8_t CHASSIS_CTRL_GYRO;
extern const uint8_t CHASSIS_CTRL_DANCE;
//      cap mode
extern const uint8_t SUPERCAP_CTRL_OFF;
extern const uint8_t SUPERCAP_CTRL_ON;
//      cap boost mode
extern const uint8_t SUPERCAP_BOOST;
extern const uint8_t SUPERCAP_UNBOOST;
//      cap state
extern const uint8_t SUPERCAP_MODE_OFF;
extern const uint8_t SUPERCAP_MODE_ON;
extern const uint8_t SUPERCAP_MODE_ERROR;

extern CAN_TxHeaderTypeDef BusComm_ChassisCanTxHeader;
extern CAN_TxHeaderTypeDef BusComm_GimbalCanTxHeader;
extern CAN_TxHeaderTypeDef BusComm_SuperCapCanTxHeader;

extern CAN_TxHeaderTypeDef BusComm_CapMode;

extern int yyy_love;

typedef enum {
    BusComm_STATE_NULL = 0,
    BusComm_STATE_CONNECTED = 1,
    BusComm_STATE_LOST = 2,
    BusComm_STATE_ERROR = 3,
    BusComm_STATE_PENDING = 4
} BusComm_BusCommStateEnum;

typedef struct {
    BusComm_BusCommStateEnum state;
    uint32_t last_update_time;

    // Chassis up stream
    float yaw_relative_angle;  // Angle of chassis relative to pan tilt
    uint8_t robot_id;          // Robot ID
    uint8_t power_limit;       // Super capacitor state
    uint16_t heat_17mm;        // Heat transfer of 17mm launching mechanism
    uint16_t heat_cooling_rate;
    uint16_t heat_cooling_limit;
    uint16_t heat_speed_limit;
    uint8_t main_shooter_power;
    uint8_t cap_mode_fnl;
    uint8_t cap_boost_mode_fnl;
    uint8_t chassis_power_limit;
    uint8_t chassis_power_buffer;
    float chassis_power;

    // Gimbal up stream
    uint8_t gimbal_yaw_mode;      // Yaw mode of gimbal
    float gimbal_yaw_ref;         // gimbal yaw target value
    float gimbal_imu_pos;         // gimbal yaw IMU angle feedback value
    float gimbal_imu_spd;         // Speed feedback value of gimbal yaw IMU
    uint8_t chassis_mode;         // Chassis mode
    float chassis_fb_ref;         // Target value of forward and back speed of chassis
    float chassis_lr_ref;         // Target value of chassis left and right speed
    uint8_t cap_mode_user;        // Capacitance mode
    uint8_t cap_boost_mode_user;  // cap boost mode
    uint8_t power_limit_mode;     // Force to change power limit mode
    float pitch_angle;
    uint8_t ui_cmd;

    // Super Cap up stream
    uint32_t power_path_change_flag;
    uint8_t cap_state;
    uint8_t cap_rest_energy;
    float cap_rest_energy_display;

    float Cap_power;    // Chassis Power
    float Cap_voltage;  // Chassis Voltage
    float Cap_current;  // Chassis Current
} BusComm_BusCommDataTypeDef;

extern BusComm_BusCommDataTypeDef BusComm_BusCommData;
extern osMessageQId BusCommSend_QueueHandle;

void BusComm_InitBusComm(void);
BusComm_BusCommDataTypeDef* BusComm_GetBusDataPtr(void);
uint8_t BusComm_IsBusCommOffline(void);
void BusComm_SendBusCommData(void);
void BusComm_CANRxCallback(CAN_HandleTypeDef* phcan, uint32_t stdid, uint8_t rxdata[], uint32_t len);
uint8_t BusComm_VerifyBusCommData(uint8_t* buff, uint16_t rxdatalen);
void BusComm_DecodeBusCommData(uint8_t buff[], uint32_t stdid, uint16_t rxdatalen);
void BusComm_DecodeBusCommData_Cap(uint8_t buff[], uint32_t stdid, uint16_t rxdatalen);
void BusComm_ResetBusCommData(void);
void BusComm_Update(void);
void _cmd_mode_control(void);
void _cmd_ui_mode_control(void);

#endif

#ifdef __cplusplus
}
#endif
