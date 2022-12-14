/*
 *  Project      : Infantry_Momentum
 *
 *  file         : supercap_ctrl.c
 *  Description  : This file contains cap control function
 *  LastEditors  : ����ؼ���ᶯ��
 *  Date         : 2021-05-04 20:53:31
 *  LastEditTime : 2021-05-16 20:44:15
 */

#include "supercap_ctrl.h"

#if __FN_IF_ENABLE(__FN_INFANTRY_CHASSIS)

#define SUPER_CAP_PERIOD 1

CAP_CtrlDataTypeDef CapData;

/**
 * @brief   Get Cap Control Ptr
 * @param 	None
 * @retval	None
 * @note	None
 */
CAP_CtrlDataTypeDef* Cap_GetCapDataPtr(void) {
    return &CapData;
}

/**
 * @brief      Cap Initialize control function
 * @param      NULL
 * @retval     NULL
 */
void Cap_Init(void) {
    CAP_CtrlDataTypeDef* capctrl = Cap_GetCapDataPtr();
    Cap_ResetCapData();
    capctrl->last_update_time = HAL_GetTick();
    capctrl->starting_time = 0;
}

/**
 * @brief      Reset supercapacitor communication data object
 * @param      NULL
 * @retval     NULL
 */
void Cap_ResetCapData(void) {
    CAP_CtrlDataTypeDef* capctrl = Cap_GetCapDataPtr();

    capctrl->cap_boost_mode = 0;
    capctrl->cap_mode_Remote = 0;
    capctrl->cap_mode_Starting = 0;
    capctrl->cap_mode_Stall = 0;
    capctrl->cap_state = 0;
}

/**
 * @brief      Update buscomm data
 * @param      NULL
 * @retval     NULL
 */
void Cap_Update(void) {
    CAP_CtrlDataTypeDef* capctrl = Cap_GetCapDataPtr();
    BusComm_BusCommDataTypeDef* buscomm = BusComm_GetBusDataPtr();
    PowerCtrl_Data_t* PowCtr = PowerCtrl_GetPowerDataPtr();
    Referee_RefereeDataTypeDef* referee = Referee_GetRefereeDataPtr();

    capctrl->Sum_PowerReally = buscomm->Cap_power;
    capctrl->Chassis_voltage = buscomm->Cap_voltage;
    capctrl->Sum_CurrentReally = buscomm->Cap_current;
    capctrl->cap_mode_Remote = buscomm->cap_mode_user;
    capctrl->cap_boost_mode = buscomm->cap_boost_mode_user;

    buscomm->cap_mode_fnl = (capctrl->cap_mode_Remote | capctrl->cap_mode_Stall | capctrl->cap_mode_Starting);
    buscomm->cap_boost_mode_fnl = PowCtr->ChassisStarting_flag << 2 | PowCtr->ChassisDown_flag << 1 | capctrl->cap_boost_mode;
    buscomm->chassis_power_limit = referee->max_chassis_power;
    buscomm->chassis_power_buffer = (uint8_t)referee->chassis_power_buffer;
    buscomm->chassis_power = referee->chassis_power;
}

#elif __FN_IF_ENABLE(__FN_SUPER_CAP)

#include "supercap_comm.h"
#include "const_lib.h"

CAP_ControlValueTypeDef Cap_ControlState;

/**
 * @brief      Set charge current
 * @param      cur: cap charge current (ma)
 * @retval     NULL
 */
void Cap_SetChargeCurrent(float cur) {
    /* Set charging dead zone */
    float current = cur;
    if (current <= 0.1f)
        DAC_SetCurrent(0.0f);
    else if (current >= 5.0f)
        DAC_SetCurrent(5.0f);
    else
        DAC_SetCurrent(current);
}

/**
 * @brief      Cap Initialize control function
 * @param      NULL
 * @retval     NULL
 */
void Cap_Init() {
    CAP_ControlValueTypeDef* capvalue = Cap_GetCapControlPtr();

    Cap_SetChargeCurrent(0);
    /* set init current */
    GPIO_Close(CAP);

    GPIO_Close(BUCK);
    GPIO_Close(BOOST);
    /* set init state   */

    capvalue->cap_state = CAP_MODE_OFF;
    capvalue->power_limit = 40.0f;
    capvalue->power_path = Power_PATH_REFEREE;
    capvalue->charge_state = CAP_CHARGE_OFF;

    HAL_Delay(2000);
}

/**
 * @brief      Gets the pointer to the cap control data object
 * @param      NULL
 * @retval     Pointer to cap control data object
 */
CAP_ControlValueTypeDef* Cap_GetCapControlPtr() {
    return &Cap_ControlState;
}

/**
 * @brief      Judge capacitor state
 * @param      NULL
 * @retval     NULL
 */
void Cap_JudgeCapState() {
    Sen_CAPBasisValueTypeDef* basisvalue = Sen_GetBasisDataPtr();
    CAP_ControlValueTypeDef* capvalue = Cap_GetCapControlPtr();
    CapComm_CapCommDataTypeDef* capcomm = CapComm_GetCapDataPty();

    if (basisvalue->CapVoltage < Cap_MinVoltage && basisvalue->CapVoltage >= 0) {
        capvalue->cap_state = CAP_MODE_OFF;
        capcomm->cap_state = SUPERCAP_MODE_OFF;
    } else if (basisvalue->CapVoltage <= 0) {
        capvalue->cap_state = CAP_MODE_ERROR;
        capcomm->cap_state = SUPERCAP_MODE_ERROR;
    } else if (basisvalue->CapVoltage >= Cap_AvailableVoltage) {
        capvalue->cap_state = CAP_MODE_ON;
        capcomm->cap_state = SUPERCAP_MODE_ON;
    }
}

/**
 * @brief      Change Control mode
 * @param      NULL
 * @retval     NULL
 */
void Cap_CapCharge() {
    CAP_ControlValueTypeDef* capvalue = Cap_GetCapControlPtr();
    Sen_PowerValueTypeDef* sendata = Sen_GetPowerDataPtr();
    Sen_CAPBasisValueTypeDef* basisdata = Sen_GetBasisDataPtr();
    CapComm_CapCommDataTypeDef* capcomm = CapComm_GetCapDataPty();

    if (capcomm->cap_charge_mode == SUPERCAP_UNCHARGE) {
        GPIO_Close(BUCK);
        Cap_SetChargeCurrent(0);
    } else if (capcomm->cap_charge_mode == SUPERCAP_CHARGE) {
        capvalue->power_limit = 0.7f * (float)capcomm->power_limit;
        if (basisdata->CapVoltage <= 8.0f) {
            Cap_SetChargeCurrent(4.0f);
        } else if (basisdata->CapVoltage <= 10.0f) {
            Cap_SetChargeCurrent(3.0f);
        } else if (basisdata->CapVoltage >= 26.5f) {
            Cap_SetChargeCurrent(0);
        } else {
            Cap_SetChargeCurrent(capvalue->power_limit / basisdata->CapVoltage);
        }
        GPIO_Open(BUCK);
    }
}

/**
 * @brief      Change Control mode
 * @param      NULL
 * @retval     success 1 fail 0
 */
void Cap_ChangePowerPath(POWER_PathEnum path) {
    CAP_ControlValueTypeDef* capvalue = Cap_GetCapControlPtr();

    if (path == Power_PATH_CAP && (capvalue->cap_state == CAP_MODE_ERROR || capvalue->cap_state == CAP_MODE_OFF))
        GPIO_Close(CAP);
    else if (path == Power_PATH_CAP && capvalue->cap_state == CAP_MODE_ON)
        GPIO_Open(CAP);
    else if (path == Power_PATH_REFEREE)
        GPIO_Close(CAP);
}

/**
 * @brief      Super Cap control function
 * @param      NULL
 * @retval     NULL
 */
void Cap_Control() {
    CapComm_CapCommDataTypeDef* capcomm = CapComm_GetCapDataPty();

    Sensor_Decode();
    // sensor data decode
    Cap_JudgeCapState();
    // Judege Cap state
    Cap_CapCharge();

    if (capcomm->cap_mode == SUPERCAP_CTRL_ON)
        Cap_ChangePowerPath(Power_PATH_CAP);

    else if (capcomm->cap_mode == SUPERCAP_CTRL_OFF)
        Cap_ChangePowerPath(Power_PATH_REFEREE);
}

#endif
