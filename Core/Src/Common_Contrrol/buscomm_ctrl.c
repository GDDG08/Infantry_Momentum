/*
 *  Project      : Infantry_Momentum
 *
 *  file         : buscomm_ctrl.c
 *  Description  : This file contains Bus communication control function
 *  LastEditors  : ����ؼ���ᶯ��
 *  Date         : 2021-05-04 20:53:31
 *  LastEditTime : 2021-05-16 07:00:40
 */

#include "buscomm_ctrl.h"
#include "buscomm_cmd.h"
#include "const_lib.h"

#if __FN_IF_ENABLE(__FN_INFANTRY_CHASSIS)
#include "cha_chassis_ctrl.h"
#include "cha_gimbal_ctrl.h"
#include "cha_referee_ctrl.h"
#include "cha_power_ctrl.h"
#include "supercap_comm.h"
#endif

#if __FN_IF_ENABLE(__FN_INFANTRY_GIMBAL)
#include "gim_gimbal_ctrl.h"
#endif

#if __FN_IF_ENABLE(__FN_SUPER_CAP)
#include "supercap_ctrl.h"
#endif

/*      infantry communication functions      */
const uint16_t Const_BusComm_TX_BUFF_LEN = 200;
const uint16_t Const_BusComm_RX_BUFF_LEN = 200;
const uint16_t Const_BusComm_OFFLINE_TIME = 500;

const uint8_t Const_BusComm_SIZE = 8;
const uint8_t Const_BusComm_GIMBAL_BUFF_SIZE = 7;
const uint8_t Const_BusComm_CHASSIS_BUFF_SIZE = 6;
const uint8_t Const_BusComm_SUPERCAP_BUFF_SIZE = 1;

const uint8_t Const_BusComm_RECEIVE_SIZE = 11;
const uint8_t Const_BusComm_RECEIVE_CAP_SIZE = 2;

const uint8_t Const_BusComm_FRAME_HEADER_SOF = 0x5A;

//      power limit mode
const uint8_t POWER_LIMITED = 0x01;
const uint8_t POWER_UNLIMIT = 0x02;
//      gimbal yaw mode
const uint8_t GIMBAL_YAW_CTRL_NO_AUTO = 0x03;
const uint8_t GIMBAL_YAW_CTRL_ARMOR = 0x04;
const uint8_t GIMBAL_YAW_CTRL_IMU_DEBUG = 0x05;
const uint8_t GIMBAL_YAW_CTRL_BIG_ENERGY = 0x06;
const uint8_t GIMBAL_YAW_CTRL_SMALL_ENERGY = 0x07;
//      chassis mode
const uint8_t CHASSIS_CTRL_STOP = 0x08;
const uint8_t CHASSIS_CTRL_NORMAL = 0x09;
const uint8_t CHASSIS_CTRL_GYRO = 0x0A;
const uint8_t CHASSIS_CTRL_DANCE = 0x0B;
//      cap mode
const uint8_t SUPERCAP_CTRL_OFF = 0x00;
const uint8_t SUPERCAP_CTRL_ON = 0x01;
//      cap boost mode
const uint8_t SUPERCAP_BOOST = 0x01;
const uint8_t SUPERCAP_UNBOOST = 0x00;
//      cap state
const uint8_t SUPERCAP_MODE_OFF = 0x51;
const uint8_t SUPERCAP_MODE_ON = 0x52;
const uint8_t SUPERCAP_MODE_ERROR = 0x53;

// Dual bus communication protocol

uint8_t BusComm_TxData[Const_BusComm_TX_BUFF_LEN];
uint8_t BusComm_RxData[Const_BusComm_RX_BUFF_LEN];
BusComm_BusCommDataTypeDef BusComm_BusCommData;

const uint32_t Const_BusComm_CAN_TX_CHASSIS_STDID = 0x010;
const uint32_t Const_BusComm_CAN_TX_CHASSIS_EXTID = 0x01;
const uint32_t Const_BusComm_CAN_TX_GIMBAL_STDID = 0x011;
const uint32_t Const_BusComm_CAN_TX_GIMBAL_EXTID = 0x01;
const uint32_t Const_BusComm_CAN_TX_SUPERCAP_STDID = 0x013;
const uint32_t Const_BusComm_CAN_TX_SUPERCAP_EXTID = 0x01;
const uint32_t Const_BusComm_CAN_TX_EXTID = 0x01;

CAN_TxHeaderTypeDef BusComm_ChassisCanTxHeader;
CAN_TxHeaderTypeDef BusComm_GimbalCanTxHeader;
CAN_TxHeaderTypeDef BusComm_SuperCapCanTxHeader;

CAN_TxHeaderTypeDef BusComm_CapMode;

#if __FN_IF_ENABLE(__FN_INFANTRY)
/**
 * @brief      Inter bus communication initialization
 * @param      NULL
 * @retval     NULL
 */
void BusComm_InitBusComm() {
    BusComm_ResetBusCommData();
    Can_InitTxHeader(&BusComm_ChassisCanTxHeader, Const_BusComm_CAN_TX_CHASSIS_STDID, Const_BusComm_CAN_TX_CHASSIS_EXTID, Const_BusComm_SIZE);
    Can_InitTxHeader(&BusComm_GimbalCanTxHeader, Const_BusComm_CAN_TX_GIMBAL_STDID, Const_BusComm_CAN_TX_GIMBAL_EXTID, Const_BusComm_SIZE);
    Can_InitTxHeader(&BusComm_SuperCapCanTxHeader, Const_BusComm_CAN_TX_SUPERCAP_STDID, Const_BusComm_CAN_TX_SUPERCAP_EXTID, Const_BusComm_SIZE);
    Can_InitTxHeader(&BusComm_CapMode, CMD_SET_CAP_MODE, Const_BusComm_CAN_TX_EXTID, Const_BusComm_SIZE);
}

/**
 * @brief      Gets the pointer to the bus communication data object
 * @param      NULL
 * @retval     Pointer to bus communication data object
 */
BusComm_BusCommDataTypeDef* BusComm_GetBusDataPtr() {
    return &BusComm_BusCommData;
}

/**
 * @brief      Check whether the dual bus communication is offline
 * @param      NULL
 * @retval     NULL
 */
uint8_t BusComm_IsBusCommOffline() {
    BusComm_BusCommDataTypeDef* buscomm = BusComm_GetBusDataPtr();
    if (HAL_GetTick() - buscomm->last_update_time > Const_BusComm_OFFLINE_TIME) {
        buscomm->state = BusComm_STATE_LOST;
        return 1;
    }
    return 0;
}

/**
 * @brief      Data sending function of serial port in inter bus communication
 * @param      NULL
 * @retval     NULL
 */
void BusComm_SendBusCommData() {
    /* up data struct data    */
    BusComm_Update();

    BusComm_BusCommDataTypeDef* buscomm = BusComm_GetBusDataPtr();

    buscomm->state = BusComm_STATE_PENDING;

// Chassis stream
#if __FN_IF_ENABLE(__FN_INFANTRY_CHASSIS)
    static int flag = 0;
    flag++;
    if (flag >= 10) {
        flag = 0;
        uint32_t out_time = HAL_GetTick();
        for (int i = 0; i < Const_BusComm_CHASSIS_BUFF_SIZE; i++) {
            while (HAL_CAN_GetTxMailboxesFreeLevel(Const_BusComm_CAN_HANDLER) == 0) {
                if (HAL_GetTick() - out_time >= 2)
                    return;
            }
            if (Buscmd_ChaSend[i].bus_func != NULL)
                Buscmd_ChaSend[i].bus_func(BusComm_TxData);
        }
    }
#endif

// Gimbal steram
#if __FN_IF_ENABLE(__FN_INFANTRY_GIMBAL)
    uint32_t out_time = HAL_GetTick();

    for (int i = 0; i < Const_BusComm_GIMBAL_BUFF_SIZE; i++) {
        while (HAL_CAN_GetTxMailboxesFreeLevel(Const_BusComm_CAN_HANDLER) == 0) {
            if (HAL_GetTick() - out_time >= 2)
                return;
        }
        if (Buscmd_GimSend[i].bus_func != NULL)
            Buscmd_GimSend[i].bus_func(BusComm_TxData);
    }
#endif

    buscomm->state = BusComm_STATE_CONNECTED;
}

/**
 * @brief      Data check function of serial port in inter bus communication
 * @param      buff: data buffer
 * @param      rxdatalen: data length
 * @retval     Verification result (1 is correct, 0 is failed)
 */
uint8_t BusComm_VerifyBusCommData(uint8_t* buff, uint16_t rxdatalen) {
    const uint8_t FAILED = 0, SUCCEEDED = 1;

    if (buff[0] != Const_BusComm_FRAME_HEADER_SOF)
        return FAILED;

    uint16_t sum = 0, checksum = buff[rxdatalen - 1];
    for (int i = 0; i < rxdatalen - 1; ++i)
        sum += buff[i];
    if ((sum & 0xff) == checksum)
        return SUCCEEDED;
    else
        return FAILED;
}

/**
 * @brief      Data decoding function of serial port in inter bus communication
 * @param      buff: Data buffer
 * @param      rxdatalen: data length
 * @retval     NULL
 */
void BusComm_DecodeBusCommData(uint8_t buff[], uint32_t stdid, uint16_t rxdatalen) {
    BusComm_BusCommDataTypeDef* buscomm = BusComm_GetBusDataPtr();
    buscomm->last_update_time = HAL_GetTick();

    memcpy(BusComm_RxData, buff, rxdatalen);
    if (!BusComm_VerifyBusCommData(buff, rxdatalen)) {
        buscomm->state = BusComm_STATE_ERROR;
        return;
    }

    for (int i = 0; i < (Const_BusComm_RECEIVE_SIZE + 1); i++) {
        if ((BusComm_RxData[1] == Buscmd_Receive[i].cmd_id) && (Buscmd_Receive[i].bus_func != NULL)) {
            Buscmd_Receive[i].bus_func(BusComm_RxData);
            return;
        }
    }
}

/**
 * @brief      Data decoding function of serial port in inter bus communication for cap
 * @param      buff: Data buffer
 * @param      rxdatalen: data length
 * @retval     NULL
 */
void BusComm_DecodeBusCommData_Cap(uint8_t buff[], uint32_t stdid, uint16_t rxdatalen) {
    BusComm_BusCommDataTypeDef* buscomm = BusComm_GetBusDataPtr();
    buscomm->last_update_time = HAL_GetTick();

    memcpy(BusComm_RxData, buff, rxdatalen);

    for (int i = 0; i < (Const_BusComm_RECEIVE_CAP_SIZE + 1); i++) {
        if (stdid == Buscmd_Receive_Cap[i].cmd_id) {
            Buscmd_Receive_Cap[i].bus_func(BusComm_RxData);
            return;
        }
    }
}

/**
 * @brief      Reset inter bus communication data object
 * @param      NULL
 * @retval     NULL
 */
void BusComm_ResetBusCommData() {
    BusComm_BusCommDataTypeDef* buscomm = BusComm_GetBusDataPtr();

    buscomm->last_update_time = HAL_GetTick();

// Chassis stream
#if __FN_IF_ENABLE(__FN_INFANTRY_CHASSIS)
    buscomm->yaw_relative_angle = 0;
    buscomm->robot_id = 0;
    buscomm->heat_17mm = 0;
    buscomm->power_limit = 0;
    buscomm->heat_cooling_rate = 0;
    buscomm->heat_cooling_limit = 0;
    buscomm->heat_speed_limit = 0;
    buscomm->main_shooter_power = 0;
#endif

// Gimbal stream
#if __FN_IF_ENABLE(__FN_INFANTRY_GIMBAL)
    buscomm->gimbal_yaw_mode = 0;
    buscomm->gimbal_yaw_ref = 0.0f;
    buscomm->gimbal_imu_pos = 0.0f;
    buscomm->gimbal_imu_spd = 0.0f;
    buscomm->chassis_mode = 0;
    buscomm->chassis_fb_ref = 0.0f;
    buscomm->chassis_lr_ref = 0.0f;
    buscomm->cap_mode_user = SUPERCAP_CTRL_OFF;
    buscomm->power_limit_mode = POWER_LIMITED;
    buscomm->cap_boost_mode_user = SUPERCAP_UNBOOST;
    buscomm->pitch_angle = 0.0f;
    buscomm->ui_cmd = 0;
#endif

// SuperCap stream
#if __FN_IF_ENABLE(__FN_SUPER_CAP)
    buscomm->cap_state = SUPERCAP_MODE_OFF;
    buscomm->cap_rest_energy = 0;
#endif
}

/**
 * @brief      Assignment of inter bus communication structure
 * @param      NULL
 * @retval     NULL
 */
void BusComm_Update() {
    BusComm_BusCommDataTypeDef* data = BusComm_GetBusDataPtr();

// Chassis stream
#if __FN_IF_ENABLE(__FN_INFANTRY_CHASSIS)
    GimbalYaw_GimbalYawTypeDef* gimbal = GimbalYaw_GetGimbalYawPtr();
    Referee_RefereeDataTypeDef* referee = Referee_GetRefereeDataPtr();
    // CapComm_CapCommDataTypeDef* capcomm = CapComm_GetCapDataPty();

    data->heat_speed_limit = referee->shooter_heat0_speed_limit;
    int mode = 0;
    if (referee->shooter_heat0_speed_limit == 15)
        mode = 0;
    if (referee->shooter_heat0_speed_limit == 18)
        mode = 1;
    if (referee->shooter_heat0_speed_limit == 30)
        mode = 2;
    Referee_SetAimMode(mode);

    Referee_SetCapState(data->cap_rest_energy);
    Referee_SetPitchAngle(data->pitch_angle);

    data->yaw_relative_angle = Motor_gimbalMotorYaw.encoder.limited_angle - Const_YAW_MOTOR_INIT_OFFSET;
    data->robot_id = referee->robot_id;
    data->power_limit = referee->max_chassis_power;
    data->heat_17mm = referee->shooter_heat0;
    data->heat_cooling_rate = referee->shooter_heat0_cooling_rate;
    data->heat_cooling_limit = referee->shooter_heat0_cooling_limit;
    data->heat_speed_limit = referee->shooter_heat0_speed_limit;
    // data->cap_state = capcomm->cap_state;
    // data->cap_rest_energy = capcomm->cap_rest_energy;
    data->main_shooter_power = referee->mains_power_shooter_output;
#endif

    // Gimbal stream
#if __FN_IF_ENABLE(__FN_INFANTRY_GIMBAL)
    IMU_IMUDataTypeDef* imu = IMU_GetIMUDataPtr();
    Gimbal_GimbalTypeDef* gimbal = Gimbal_GetGimbalControlPtr();

    data->gimbal_yaw_mode = gimbal->yaw_mode + 0x02;
    data->gimbal_yaw_ref = gimbal->angle.yaw_angle_ref;
    data->gimbal_imu_pos = imu->angle.yaw;
    data->gimbal_imu_spd = imu->speed.yaw;
    data->pitch_angle = imu->angle.pitch;

#endif

    // Super Cap stream
#if __FN_IF_ENABLE(__FN_SUPER_CAP)
    Sen_PowerValueTypeDef* powerValue = Sen_GetPowerDataPtr();
    data->cap_rest_energy = powerValue->CapPercent;
#endif
}

void _cmd_mode_control() {
    BusComm_BusCommDataTypeDef* buscomm = BusComm_GetBusDataPtr();

#if __FN_IF_ENABLE(__FN_INFANTRY_CHASSIS)
    switch (buscomm->gimbal_yaw_mode) {
        case GIMBAL_YAW_CTRL_BIG_ENERGY: {
            GimbalYaw_SetMode(GimbalYaw_MODE_BIG_ENERGY);
            GimbalYaw_SetYawRef(buscomm->gimbal_yaw_ref);
            GimbalYaw_SetIMUYawPositionFdb(buscomm->gimbal_imu_pos);
            GimbalYaw_SetIMUYawSpeedFdb(buscomm->gimbal_imu_spd);
            GimbalYaw_SetGimbalYawControlState(1);
            GimbalYaw_SetGimbalYawOutputState(1);
            break;
        }
        case GIMBAL_YAW_CTRL_SMALL_ENERGY: {
            GimbalYaw_SetMode(GimbalYaw_MODE_SMALL_ENERGY);
            GimbalYaw_SetYawRef(buscomm->gimbal_yaw_ref);
            GimbalYaw_SetIMUYawPositionFdb(buscomm->gimbal_imu_pos);
            GimbalYaw_SetIMUYawSpeedFdb(buscomm->gimbal_imu_spd);
            GimbalYaw_SetGimbalYawControlState(1);
            GimbalYaw_SetGimbalYawOutputState(1);
            break;
        }
        case GIMBAL_YAW_CTRL_ARMOR: {
            GimbalYaw_SetMode(GimbalYaw_MODE_ARMOR);
            GimbalYaw_SetYawRef(buscomm->gimbal_yaw_ref);
            GimbalYaw_SetIMUYawPositionFdb(buscomm->gimbal_imu_pos);
            GimbalYaw_SetIMUYawSpeedFdb(buscomm->gimbal_imu_spd);
            GimbalYaw_SetGimbalYawControlState(1);
            GimbalYaw_SetGimbalYawOutputState(1);
            break;
        }
        case GIMBAL_YAW_CTRL_IMU_DEBUG: {
            GimbalYaw_SetMode(GimbalYaw_MODE_IMU_DEBUG);
            GimbalYaw_SetYawRef(buscomm->gimbal_yaw_ref);
            GimbalYaw_SetEncoderFdb();
            GimbalYaw_SetGimbalYawControlState(1);
            GimbalYaw_SetGimbalYawOutputState(1);
            break;
        }
        case GIMBAL_YAW_CTRL_NO_AUTO: {
            GimbalYaw_SetMode(GimbalYaw_MODE_NO_AUTO);
            GimbalYaw_SetYawRef(buscomm->gimbal_yaw_ref);
            GimbalYaw_SetIMUYawPositionFdb(buscomm->gimbal_imu_pos);
            GimbalYaw_SetIMUYawSpeedFdb(buscomm->gimbal_imu_spd);
            GimbalYaw_SetGimbalYawControlState(1);
            GimbalYaw_SetGimbalYawOutputState(1);
            break;
        }
        default:
            return;  // error, stop decoding
    }

    switch (buscomm->chassis_mode) {
        case CHASSIS_CTRL_STOP: {
            Chassis_SetMode(Chassis_MODE_STOP);
            Chassis_SetForwardBackRef(0);
            Chassis_SetLeftRightRef(0);
            Chassis_SetChassisControlState(1);
            Chassis_SetChassisOutputState(1);
            break;
        }
        case CHASSIS_CTRL_NORMAL: {
            Chassis_SetMode(Chassis_MODE_NORMAL);
            Chassis_SetForwardBackRef(buscomm->chassis_fb_ref);
            Chassis_SetLeftRightRef(buscomm->chassis_lr_ref);
            Chassis_SetChassisControlState(1);
            Chassis_SetChassisOutputState(1);

            Referee_SetWidthMode(0);

            break;
        }
        case CHASSIS_CTRL_GYRO: {
            Chassis_SetMode(Chassis_MODE_GYRO);
            Chassis_SetForwardBackRef(buscomm->chassis_fb_ref);
            Chassis_SetLeftRightRef(buscomm->chassis_lr_ref);
            Chassis_SetChassisControlState(1);
            Chassis_SetChassisOutputState(1);

            Referee_SetWidthMode(1);

            break;
        }
        case CHASSIS_CTRL_DANCE: {
            Chassis_SetMode(Chassis_MODE_DANCE);
            Chassis_SetForwardBackRef(buscomm->chassis_fb_ref);
            Chassis_SetLeftRightRef(buscomm->chassis_lr_ref);
            Chassis_SetChassisControlState(1);
            Chassis_SetChassisOutputState(1);

            Referee_SetWidthMode(1);

            break;
        }
        default:
            return;  // error, stop decoding
    }

    switch (buscomm->power_limit_mode) {
        case POWER_LIMITED:
            Power_ForceChangePowerMode(POWER_LIMIT);
            break;
        case POWER_UNLIMIT:
            Power_ForceChangePowerMode(POWER_UNLIMITED);
            break;
        default:
            break;
    }
#endif
}

int yyy_love = 0;
void _cmd_ui_mode_control() {
#if __FN_IF_ENABLE(__FN_INFANTRY_CHASSIS)
    BusComm_BusCommDataTypeDef* buscomm = BusComm_GetBusDataPtr();
    static int flag = 0;
    if (buscomm->ui_cmd == 1) {
        flag = 1;
    } else
        flag = 0;

    if (flag == 1) {
        Referee_Setup();
        yyy_love = 1;
    }
#endif
}

/**
 * @brief      Interrupt callback function of can in inter Bus communication
 * @param
 * @retval     NULL
 */
void BusComm_CANRxCallback(CAN_HandleTypeDef* phcan, uint32_t stdid, uint8_t rxdata[], uint32_t len) {
#if __FN_IF_ENABLE(__FN_INFANTRY)
    if (phcan == Const_BusComm_CAN_HANDLER) {
        BusComm_DecodeBusCommData(rxdata, stdid, len);
    } else if (phcan == &hcan1) {
#if __FN_IF_ENABLE(__FN_INFANTRY_CHASSIS)
        BusComm_DecodeBusCommData_Cap(rxdata, stdid, len);
#endif
    }
#endif
}

#endif
