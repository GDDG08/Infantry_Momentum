/*
 *  Project      : Infantry_Momentum
 *
 *  file         : imu_periph.c
 *  Description  : This file contains IMU function (For HI229)(Only speed and angle data)
 *  LastEditors  : ����ؼ���ᶯ��
 *  Date         : 2021-05-04 20:53:31
 *  LastEditTime : 2021-05-07 03:41:00
 */

#include "imu_periph.h"

#if __FN_IF_ENABLE(__FN_PERIPH_IMU)

#include "crc_alg.h"
#include "const_lib.h"

const uint16_t Const_IMU_RX_BUFF_LEN = 20;
const uint16_t Const_IMU_IMU_OFFLINE_TIME = 1000;

uint8_t IMU_RxData[Const_IMU_RX_BUFF_LEN];
IMU_IMUDataTypeDef IMU_IMUData;
CRC_MatchEnum CRC_IMUEnum;

float speed[3];
float angle[3];

/**
 * @brief      Get pinter to the IMU data object
 * @param      NULL
 * @retval     Pointer to IMU data object
 */
IMU_IMUDataTypeDef* IMU_GetIMUDataPtr() {
    return &IMU_IMUData;
}

/**
 * @brief      Initialization IMU
 * @param      NULL
 * @retval     NULL
 */
void IMU_InitIMU() {
    IMU_ResetIMUData(&IMU_IMUData);
    IMU_IMUData.last_update_time = HAL_GetTick();
    Uart_InitUartDMA(Const_IMU_UART_HANDLER);
    Uart_ReceiveDMA(Const_IMU_UART_HANDLER, IMU_RxData, Const_IMU_RX_BUFF_LEN);
}

/**
 * @brief      Initialization offset and set mode
 * @param      imu: pinter to the IMU data object
 * @retval     NULL
 */
void IMU_InitAngelOffset(IMU_IMUDataTypeDef* imu) {
    //    float y = 0;
    //    for (int i = 0; i < 100; i++)
    //        y += imu->angle.pitch;
    imu->yaw_angle_offset = imu->angle.yaw;
    //    imu->pitch_angle_offset = - y / 100;
}

/**
 * @brief      Judge IMU offline
 * @param      imu: Pinter to the IMU data object
 * @retval     Offline or not��1 is offline��0 is not��
 */
uint8_t IMU_IsIMUOffline(IMU_IMUDataTypeDef* imu) {
    uint32_t now = HAL_GetTick();
    if ((now - imu->last_update_time) > Const_IMU_IMU_OFFLINE_TIME)
        imu->state = IMU_STATE_LOST;
    return imu->state == IMU_STATE_LOST;
}

/**
 * @brief      IMU UART callback function
 * @param      huart: Point to serial port handle
 * @retval     NULL
 */
void IMU_RXCallback(UART_HandleTypeDef* huart) {
    /* clear DMA transfer complete flag */
    __HAL_DMA_DISABLE(huart->hdmarx);

    /* handle uart data from DMA */
    int rxdatalen = Const_IMU_RX_BUFF_LEN - Uart_DMACurrentDataCounter(huart->hdmarx->Instance);
    IMU_DecodeIMUData(&IMU_IMUData, IMU_RxData, rxdatalen);

    /* restart dma transmission */
    __HAL_DMA_SET_COUNTER(huart->hdmarx, Const_IMU_RX_BUFF_LEN);
    __HAL_DMA_ENABLE(huart->hdmarx);
}

/**
 * @brief      Reset IMU data object
 * @param      imu: pinter to the IMU data object
 * @retval     NUL
 */
void IMU_ResetIMUData(IMU_IMUDataTypeDef* imu) {
    imu->count.pitch = 0;
    imu->count.yaw = 0;
    imu->angle.pitch = 0;
    imu->angle.yaw = 0;
    imu->now_angle.pitch = 0;
    imu->now_angle.yaw = 0;
    imu->yaw_angle_offset = 0;
    imu->pitch_angle_offset = 0;
}

/**
 * @brief      IMU decode data function    ��For HI229)
 * @param      referee: Pinter to the IMU data object
 * @param      buff: Data buffer
 * @param      rxdatalen: Data length
 * @retval     NULL
 */
void IMU_DecodeIMUData(IMU_IMUDataTypeDef* imu, uint8_t* buff, int rxdatalen) {
    imu->state = IMU_STATE_PENDING;
    imu->last_update_time = HAL_GetTick();

    int16_t temp[3];
    //  CRC verify
    CRC_IMUEnum = CRC_VerifyIMU_HI229(buff);
    //    if (CRC_IMUEnum == NOT_MATCH) return;

    // decode IMU speed
    int imu_bias = 6;
    temp[0] = (int16_t)((buff[imu_bias + 2] << 8) | buff[imu_bias + 1]);  // pitch
    temp[1] = (int16_t)((buff[imu_bias + 4] << 8) | buff[imu_bias + 3]);  // roll
    temp[2] = (int16_t)((buff[imu_bias + 6] << 8) | buff[imu_bias + 5]);  // yaw

    for (int i = 0; i < 3; i++) {
        speed[i] = (float)temp[i] * 0.1f;
    }
    imu->speed.pitch = speed[1];
    imu->speed.yaw = speed[2];

    // decode IMU angle
    imu->last_angle.pitch = imu->now_angle.pitch;
    imu->last_angle.yaw = imu->now_angle.yaw;
    int angle_bias = imu_bias + 7;
    temp[0] = (int16_t)((buff[angle_bias + 2] << 8) | buff[angle_bias + 1]);  // pitch
    temp[1] = (int16_t)((buff[angle_bias + 4] << 8) | buff[angle_bias + 3]);  // row
    temp[2] = (int16_t)((buff[angle_bias + 6] << 8) | buff[angle_bias + 5]);  // yaw

    angle[0] = (float)(temp[0] / 100.0f);
    angle[1] = (float)(temp[1] / 100.0f);
    angle[2] = (float)(temp[2] / 10.0f);

    imu->now_angle.pitch = angle[0];
    imu->now_angle.yaw = angle[2];

    if (imu->now_angle.pitch - imu->last_angle.pitch < -181)
        imu->count.pitch++;
    if (imu->now_angle.pitch - imu->last_angle.pitch > 181)
        imu->count.pitch--;
    imu->angle.pitch = (float)imu->count.pitch * 360.0f + imu->now_angle.pitch;

    if (imu->now_angle.yaw - imu->last_angle.yaw < -181)
        imu->count.yaw++;
    if (imu->now_angle.yaw - imu->last_angle.yaw > 181)
        imu->count.yaw--;
    imu->angle.yaw = (float)imu->count.yaw * 360.0f + imu->now_angle.yaw;

    imu->angle.pitch = imu->angle.pitch + imu->pitch_angle_offset;
    imu->angle.yaw = imu->angle.yaw - imu->yaw_angle_offset;

    imu->last_update_time = HAL_GetTick();
    imu->state = IMU_STATE_CONNECTED;
}

#endif
