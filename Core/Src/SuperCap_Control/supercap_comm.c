/*
 *  Project      : Infantry_Momentum
 *
 *  file         : supercap_comm.c
 *  Description  : This file is for sb can init
 *  LastEditors  : ����ؼ���ᶯ��
 *  Date         : 2021-05-14 12:01:01
 *  LastEditTime : 2021-05-14 12:20:22
 */

#include "supercap_comm.h"
#include "buscomm_ctrl.h"
#include "const_lib.h"

#if __FN_IF_ENABLE(__FN_SUPER_CAP_COMM)

const uint16_t Const_CapComm_TX_BUFF_LEN = 200;
const uint16_t Const_CapComm_RX_BUFF_LEN = 200;
const uint16_t Const_CapComm_OFFLINE_TIME = 100;
const uint16_t Const_CapComm_TO_CHA_STREAM_SIZE = 9;
const uint16_t Const_CapComm_TO_CAP_STREAM_SIZE = 9;
const uint8_t Const_CapComm_FRAME_HEADER_SOF = 0x5A;

uint8_t CapComm_TxData[Const_CapComm_TX_BUFF_LEN];
uint8_t CapComm_RxData[Const_CapComm_RX_BUFF_LEN];

CapComm_CapCommDataTypeDef CapComm_CapCommData;

/**
 * @brief      Inter bus communication initialization
 * @param      NULL
 * @retval     NULL
 */
void CapComm_InitCapComm() {
    CapComm_ResetCapCommData();
    CapComm_CapCommData.last_update_time = HAL_GetTick();
    Uart_InitUartDMA(Const_SuperCap_UART_HANDLER);
    Uart_ReceiveDMA(Const_SuperCap_UART_HANDLER, CapComm_RxData, Const_CapComm_RX_BUFF_LEN);
}

/**
 * @brief      Gets the pointer to the cap communication data object
 * @param      NULL
 * @retval     Pointer to cap communication data object
 */
CapComm_CapCommDataTypeDef* CapComm_GetCapDataPty() {
    return &CapComm_CapCommData;
}

/**
 * @brief      Reset supercapacitor communication data object
 * @param      NULL
 * @retval     NULL
 */
void CapComm_ResetCapCommData() {
    CapComm_CapCommDataTypeDef* capcomm = CapComm_GetCapDataPty();

    capcomm->cap_charge_mode = 0;
    capcomm->cap_mode = 0;
    capcomm->cap_rest_energy = 0;
    capcomm->cap_state = 0;
    capcomm->power_limit = 0;
    capcomm->power_path_change_flag = 0;
}

/**
 * @brief      Judege weather capcomm is offline or not
 * @param      NULL
 * @retval     online is 1  offline is 0
 */
uint8_t CapComm_IsCapCommOffline() {
    CapComm_CapCommDataTypeDef* capcomm = CapComm_GetCapDataPty();
    if (HAL_GetTick() - capcomm->last_update_time > Const_CapComm_OFFLINE_TIME) {
        capcomm->state = CapComm_STATE_LOST;
        return 1;
    }
    return 0;
}

/**
 * @brief      Update cap communication
 * @param      NULL
 * @retval     NULL
 */
void CapComm_Update() {
    CapComm_CapCommDataTypeDef* capcomm = CapComm_GetCapDataPty();

// chassis to cap stream
#if __FN_IF_ENABLE(__FN_INFANTRY_CHASSIS)
    BusComm_BusCommDataTypeDef* buscomm = BusComm_GetBusDataPtr();

    capcomm->cap_mode = buscomm->cap_mode;
    capcomm->power_limit = (uint8_t)buscomm->power_limit;
    capcomm->cap_charge_mode = buscomm->cap_charge_mode;
#endif

// cap to chassis stream
#if __FN_IF_ENABLE(__FN_SUPER_CAP)
    Sen_PowerValueTypeDef* sensor = Sen_GetPowerDataPtr();
    CAP_ControlValueTypeDef* capctrl = Cap_GetCapControlPtr();
    if (sensor->CapPercent >= 100)
        capcomm->cap_rest_energy = 100;
    else
        capcomm->cap_rest_energy = sensor->CapPercent;
        //  capcomm->cap_state = capctrl->cap_state;
#endif
}

/**
 * @brief      Data sending function of serial port in inter cap communication
 * @param      NULL
 * @retval     NULL
 */
void CapComm_SendCapCommData() {
    /* up data struct data    */
    CapComm_Update();

    CapComm_CapCommDataTypeDef* capcomm = CapComm_GetCapDataPty();

    capcomm->last_update_time = HAL_GetTick();
    capcomm->state = CapComm_STATE_PENDING;
// chassis to cap
#if __FN_IF_ENABLE(__FN_INFANTRY_CHASSIS)
    uint8_t* buff = CapComm_TxData;
    int size = Const_CapComm_TO_CAP_STREAM_SIZE;
    buff[0] = Const_CapComm_FRAME_HEADER_SOF;
    buff[1] = capcomm->power_limit;
    buff[2] = capcomm->cap_mode;
    buff[3] = capcomm->cap_charge_mode;
#endif

// cap to chassis
#if __FN_IF_ENABLE(__FN_SUPER_CAP)
    uint8_t* buff = CapComm_TxData;
    int size = Const_CapComm_TO_CHA_STREAM_SIZE;
    buff[0] = Const_CapComm_FRAME_HEADER_SOF;
    buff[1] = capcomm->cap_rest_energy;
    buff[2] = capcomm->cap_state;
#endif

    uint16_t checksum = 0;
    for (int i = 0; i < size - 1; ++i)
        checksum += buff[i];
    buff[size - 1] = checksum & 0xff;
    if (HAL_UART_GetState(Const_SuperCap_UART_HANDLER) & 0x01)
        return;  // tx busy
    Uart_SendMessage_IT(Const_SuperCap_UART_HANDLER, buff, size);

    capcomm->state = CapComm_STATE_CONNECTED;
}

/**
 * @brief      Data check function of serial port in inter cap communication
 * @param      buff: data buffer
 * @param      rxdatalen: data length
 * @retval     Verification result (1 is correct, 0 is failed)
 */
uint8_t CapComm_VerifyCapCommData(uint8_t* buff, uint16_t rxdatalen) {
    const uint8_t FAILED = 0, SUCCEEDED = 1;

    if (buff[0] != Const_CapComm_FRAME_HEADER_SOF)
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
 * @brief      Data decoding function of serial port in inter cap communication
 * @param      buff: Data buffer
 * @param      rxdatalen: data length
 * @retval     NULL
 */
void CapComm_DecodeCapCommData(uint8_t* buff, uint16_t rxdatalen) {
    CapComm_CapCommDataTypeDef* capcomm = CapComm_GetCapDataPty();

    capcomm->last_update_time = HAL_GetTick();

    capcomm->last_cap_mode = capcomm->cap_mode;

    if (!CapComm_VerifyCapCommData(buff, rxdatalen)) {
        capcomm->state = CapComm_STATE_ERROR;
        return;
    }

// cap to chassis
#if __FN_IF_ENABLE(__FN_INFANTRY_CHASSIS)
    capcomm->cap_rest_energy = buff[1];
    capcomm->cap_state = buff[2];
#endif

// chassis to cap
#if __FN_IF_ENABLE(__FN_SUPER_CAP)
    capcomm->power_limit = buff[1];
    capcomm->cap_mode = buff[2];
    capcomm->cap_charge_mode = buff[3];

    if ((capcomm->last_cap_mode == SUPERCAP_CTRL_ON) && (capcomm->cap_mode == SUPERCAP_CTRL_OFF)) {
        capcomm->power_path_change_flag = HAL_GetTick();
    }
#endif
}

/**
 * @brief      Interrupt callback function of serial port in inter cap communication
 * @param      huart: Pointer to serial port handle
 * @retval     NULL
 */
void CapComm_RXCallback(UART_HandleTypeDef* huart) {
    /* clear DMA transfer complete flag */
    __HAL_DMA_DISABLE(huart->hdmarx);

    /* handle dbus data dbus_buf from DMA */
    uint16_t rxdatalen = Const_CapComm_RX_BUFF_LEN - Uart_DMACurrentDataCounter(huart->hdmarx->Instance);
    CapComm_DecodeCapCommData(CapComm_RxData, rxdatalen);

    /* restart dma transmission */
    __HAL_DMA_SET_COUNTER(huart->hdmarx, Const_CapComm_RX_BUFF_LEN);
    __HAL_DMA_ENABLE(huart->hdmarx);
}

#endif
