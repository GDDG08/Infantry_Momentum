/*
 *  Project      : Infantry_Momentum
 * 
 *  file         : supercap_comm.h
 *  Description  : This file is for sb can init
 *  LastEditors  : 动情丶卜灬动心
 *  Date         : 2021-05-14 12:01:15
 *  LastEditTime : 2021-05-14 12:26:32
 */

#ifndef SUPERCAP_COMM_H
#define SUPERCAP_COMM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "configure.h"

#if __FN_IF_ENABLE(__FN_SUPER_CAP_COMM)

#include "uart_util.h"
#include "sensor_periph.h"
#include "buscomm_ctrl.h"
#include "supercap_ctrl.h"

typedef enum {
    CapComm_STATE_NULL = 0,
    CapComm_STATE_CONNECTED = 1,
    CapComm_STATE_LOST = 2,
    CapComm_STATE_ERROR = 3,
    CapComm_STATE_PENDING = 4
} CapComm_CapCommStateEnum;

typedef struct {
    CapComm_CapCommStateEnum state;
    uint32_t last_update_time;
    uint32_t power_path_change_flag;

    // Chassis up stream
    uint8_t power_limit;  // Super capacitor state
    uint8_t cap_charge_mode;
    uint8_t cap_mode;  // Capacitance mode
    uint8_t last_cap_mode;

    // Super Cap up stream
    uint8_t cap_state;
    uint8_t cap_rest_energy;

} CapComm_CapCommDataTypeDef;

extern CapComm_CapCommDataTypeDef CapComm_CapCommData;

void CapComm_InitCapComm(void);
CapComm_CapCommDataTypeDef* CapComm_GetCapDataPty(void);
void CapComm_ResetCapCommData(void);
uint8_t CapComm_IsCapCommOffline(void);
void CapComm_Update(void);
void CapComm_SendCapCommData(void);
uint8_t CapComm_VerifyCapCommData(uint8_t* buff, uint16_t rxdatalen);
void CapComm_DecodeCapCommData(uint8_t* buff, uint16_t rxdatalen);
void CapComm_RXCallback(UART_HandleTypeDef* huart);

#endif

#ifdef __cplusplus
}
#endif

#endif
