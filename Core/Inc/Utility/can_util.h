/*
 *  Project      : Infantry_Momentum
 * 
 *  file         : can.h
 *  Description  : This file contains the CAN functions
 *  LastEditors  : 动情丶卜灬动心
 *  Date         : 2021-05-04 20:53:31
 *  LastEditTime : 2021-05-08 10:53:26
 */

#ifndef CAN_UTIL_H
#define CAN_UTIL_H

#ifdef __cplusplus
extern "C" {
#endif 

#include "configure.h"

#if __FN_IF_ENABLE(__FN_UTIL_CAN)

#include "can.h"

void Can_ErrorHandler(uint32_t ret);
void Can_InitTxHeader(CAN_TxHeaderTypeDef *pheader, uint32_t stdid, uint32_t extid, uint32_t dlc);
void Can_InitFilterAndStart(CAN_HandleTypeDef* phcan);
void Can_SendMessage(CAN_HandleTypeDef* phcan, CAN_TxHeaderTypeDef* pheader, uint8_t txdata[]);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* phcan);
void Can_RxMessageCallback(CAN_HandleTypeDef* phcan, CAN_RxHeaderTypeDef* rxheader, uint8_t rxdata[]);

#endif

#ifdef __cplusplus
}
#endif

#endif
