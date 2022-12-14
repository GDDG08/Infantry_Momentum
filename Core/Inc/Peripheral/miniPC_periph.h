/*
 *  Project      : Infantry_Momentum
 *
 *  file         : miniPC_periph.h
 *  Description  : This file contains mini_PC data transceiver related auxiliary functions
 *  LastEditors  : ����ؼ���ᶯ��
 *  Date         : 2021-05-04 20:53:31
 *  LastEditTime : 2021-05-08 10:58:40
 */

#ifndef MINIPC_PERIPH_H
#define MINIPC_PERIPH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "configure.h"

#if __FN_IF_ENABLE(__FN_PERIPH_MINIPC)

#include "uart_util.h"
#include "buff_lib.h"

typedef enum {
    MiniPC_NULL = 0,
    MiniPC_CONNECTED = 1,
    MiniPC_LOST = 2,
    MiniPC_ERROR = 3,
    MiniPC_PENDING = 4
} MiniPC_MiniPCStateEnum;

typedef struct {
    uint8_t heart_flag;

    // up stream
    uint8_t team_color;
    uint8_t mode;

    // down stream
    uint8_t is_get_target;  // 1 to get armor plate, 0 to not get armor plate
    float yaw_angle;
    float pitch_angle;
    float distance;

    uint8_t addressee;
    MiniPC_MiniPCStateEnum state;
    uint32_t last_update_time;
} MiniPC_MiniPCDataTypeDef;

extern const uint8_t Const_MiniPC_ARMOR;
extern const uint8_t Const_MiniPC_BIG_BUFF;
extern const uint8_t Const_MiniPC_LITTLE_BUFF;

extern MiniPC_MiniPCDataTypeDef MiniPC_MiniPCData;

void MiniPC_InitMiniPC(void);
MiniPC_MiniPCDataTypeDef* MiniPC_GetMiniPCDataPtr(void);
void MiniPC_SendHeartPacket(void);
uint8_t MiniPC_IsMiniPCOffline(void);
void MiniPC_RXCallback(UART_HandleTypeDef* huart);
void MiniPC_DecodeMiniPCPacket(uint8_t* buff, uint16_t rxdatalen);
void MiniPC_HeartPacketDecode(uint8_t* buff, uint16_t rxdatalen);
void MiniPC_ArmorPacketDecode(uint8_t* buff, uint16_t rxdatalen);
void MiniPC_ResetMiniPCData(void);
void MiniPC_Update(void);
uint8_t MiniPC_VerifyMiniPCData(uint8_t* buff, uint16_t rxdatalen);

#endif

#ifdef __cplusplus
}
#endif

#endif
