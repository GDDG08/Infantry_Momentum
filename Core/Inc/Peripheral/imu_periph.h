/*
 *  Project      : Infantry_Momentum
 * 
 *  file         : imu_periph.h
 *  Description  : This file contains IMU function (For HI229)(Only speed and angle data)
 *  LastEditors  : 动情丶卜灬动心
 *  Date         : 2021-05-04 20:53:31
 *  LastEditTime : 2021-05-08 10:56:28
 */


#ifndef IMU_PERIPH_H
#define IMU_PERIPH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "configure.h"
#if __FN_IF_ENABLE(__FN_PERIPH_IMU)

#include "uart_util.h"

typedef enum {
    IMU_STATE_NULL      = 0,
    IMU_STATE_CONNECTED = 1,
    IMU_STATE_LOST      = 2,
    IMU_STATE_ERROR     = 3,
    IMU_STATE_PENDING   = 4
} IMU_IMUStateEnum;

typedef struct {
    float yaw;
    float pitch;
} IMU_IMUSpeedTypeDef;

typedef struct {
    float yaw;
    float pitch;
} IMU_IMUAngleTypeDef;

typedef struct {
    float yaw;
    float pitch;
} IMU_IMUCountTypeDef;

typedef struct {
    IMU_IMUCountTypeDef count;
    IMU_IMUSpeedTypeDef speed;
    IMU_IMUAngleTypeDef angle;
    IMU_IMUAngleTypeDef last_angle;
    IMU_IMUAngleTypeDef now_angle;

    float yaw_angle_offset;
    float pitch_angle_offset;
    
    IMU_IMUStateEnum state;
    uint32_t last_update_time;
} IMU_IMUDataTypeDef;

extern const uint16_t Const_IMU_RX_BUFF_LEN;
extern const uint16_t Const_IMU_IMU_OFFLINE_TIME;

IMU_IMUDataTypeDef* IMU_GetIMUDataPtr(void);
void IMU_InitIMU(void);
uint8_t IMU_IsIMUOffline(IMU_IMUDataTypeDef* imu);
void IMU_InitAngelOffset(IMU_IMUDataTypeDef *imu);
void IMU_RXCallback(UART_HandleTypeDef* huart);
void IMU_ResetIMUData(IMU_IMUDataTypeDef* imu);
void IMU_DecodeIMUData(IMU_IMUDataTypeDef* imu, uint8_t* buff, int rxdatalen);


#endif

#ifdef __cplusplus
}
#endif

#endif
