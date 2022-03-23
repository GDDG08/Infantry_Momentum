/*
 * @Project      : RM_Infantry_Neptune_frame
 * @FilePath     : \GITEE2\Core\Inc\Common_Contrrol\buscomm_cmd.h
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2021-07-24 11:39:13
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-03-20 15:09:05
 */
/*
 *  Project      : Infantry_Momentum
 *
 *  file         : buscomm_cmd.h
 *  Description  : This file is for idiot Can communication
 *  LastEditors  : ����ؼ���ᶯ��
 *  Date         : 2021-05-09 03:52:53
 *  LastEditTime : 2021-05-16 01:27:20
 */

#ifndef BUSCOMM_CMD_H
#define BUSCOMM_CMD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "configure.h"

#include "stm32f4xx_hal.h"

extern const uint8_t CMD_SET_YAW_RELATIVE_ANGLE;
extern const uint8_t CMD_SET_ROBOT_ID_POWER_LIMIT;
extern const uint8_t CMD_SET_17MM_DATA;
extern const uint8_t CMD_SET_COOLING_DATA;

extern const uint8_t CMD_SET_MODE;
extern const uint8_t CMD_SET_YAW_REF;
extern const uint8_t CMD_SET_IMU_POS;
extern const uint8_t CMD_SET_IMU_SPD;
extern const uint8_t CMD_SET_CHA_FB;
extern const uint8_t CMD_SET_CHA_LR;

extern const uint8_t CMD_SENT_CAP_STATE;

extern const uint32_t CMD_SET_CAP_MODE;
extern const uint32_t CMD_SET_CAP_STATE_1;
extern const uint32_t CMD_SET_CAP_STATE_2;

typedef struct {
    int cmd_id;
    void (*bus_func)(uint8_t buff[]);
} BusCmd_TableEntry;

extern BusCmd_TableEntry Buscmd_Receive[12];
extern BusCmd_TableEntry Buscmd_Receive_Cap[3];
extern BusCmd_TableEntry Buscmd_GimSend[7];
extern BusCmd_TableEntry Buscmd_ChaSend[6];
extern BusCmd_TableEntry Buscmd_CapSend[1];

#endif

#ifdef __cplusplus
}
#endif
