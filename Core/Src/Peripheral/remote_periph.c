/*
 *  Project      : Infantry_Momentum
 * 
 *  file         : remote_periph.c
 *  Description  : This file contains remote relevant function
 *  LastEditors  : ����ؼ���ᶯ��
 *  Date         : 2021-05-04 20:53:31
 *  LastEditTime : 2021-05-07 03:36:21
 */

#include "remote_periph.h"

#if __FN_IF_ENABLE(__FN_PERIPH_REMOTE)

#include "const_lib.h"
#include "gim_remote_ctrl.h"

/*          Remote control related constants    */
const uint16_t Const_Remote_RX_BUFF_LEN             = 54;
const uint16_t Const_Remote_RX_FRAME_LEN            = 18;
const uint16_t Const_Remote_CHANNEL_VALUE_LIMIT     = 640;
const uint16_t Const_Remote_CHANNEL_VALUE_OFFSET    = 1024;
const uint16_t Const_Remote_CHANNEL_ERROR_LIMIT     = 700;
const uint16_t Const_Remote_REMOTE_OFFLINE_TIME     = 1000;

uint8_t Remote_RxData[Const_Remote_RX_BUFF_LEN];
Remote_RemoteDataTypeDef Remote_RemoteData;

/**
  * @brief      Gets the pointer of the remote control object
  * @param      NULL
  * @retval     Pointer to remote control object
  */
Remote_RemoteDataTypeDef* Remote_GetRemoteDataPtr() {
    return &Remote_RemoteData;
}


/**
  * @brief      Initialize remote control
  * @param      NULL
  * @retval     NULL
  */
void Remote_InitRemote() {
    Remote_ResetRemoteData(&Remote_RemoteData);
    Remote_RemoteData.last_update_time = HAL_GetTick();
    Uart_InitUartDMA(Const_Remote_UART_HANDLER);
    Uart_ReceiveDMA(Const_Remote_UART_HANDLER, Remote_RxData, Const_Remote_RX_BUFF_LEN);
}


/**
  * @brief      Switch position state
  * @param      sw: Original switch value
  * @retval     Switch position status
  */
Remote_SwitchStateEnum Remote_ToSwitchState(uint8_t sw) {
    return (Remote_SwitchStateEnum) sw;
}


/**
  * @brief      Judge whether the remote control is offline
  * @param      rc: pointer to remote control object
  * @retval     Offline or not (1 is yes, 0 is no)
  */
uint8_t Remote_IsRemoteOffline(Remote_RemoteDataTypeDef* rc) {
    uint32_t now = HAL_GetTick();
    if ((now - rc->last_update_time) > Const_Remote_REMOTE_OFFLINE_TIME)
        rc->state = Remote_STATE_LOST;
    return rc->state == Remote_STATE_LOST;
}


/**
  * @brief      Remote control receiving callback function
  * @param      huart: Pointer to UART handle
  * @retval     NULL
  */
void Remote_RXCallback(UART_HandleTypeDef* huart) {
    /* clear DMA transfer complete flag */
    __HAL_DMA_DISABLE(huart->hdmarx);

    /* handle uart data from DMA */
    int rxdatalen = Const_Remote_RX_BUFF_LEN - Uart_DMACurrentDataCounter(huart->hdmarx->Instance);
    Remote_DecodeRemoteData(&Remote_RemoteData, Remote_RxData, rxdatalen);

    /* restart dma transmission */
    __HAL_DMA_SET_COUNTER(huart->hdmarx, Const_Remote_RX_BUFF_LEN);
    __HAL_DMA_ENABLE(huart->hdmarx);
}


/**
  * @brief      Judge whether the remote control data is wrong
  * @param      rc: pointer to remote control object
  * @retval     Error or not (1 is yes, 0 is no)
  */
uint8_t Remote_IsRemoteError(Remote_RemoteDataTypeDef* rc) {
    const uint8_t REMOTE_OK      = 0;
    const uint8_t REMOTE_ERROR   = 1;

    for (int i = 0; i < 5; ++i)
        if (abs(rc->remote.ch[i]) > Const_Remote_CHANNEL_ERROR_LIMIT) {
            return REMOTE_ERROR;
        }
    for (int i = 0; i < 2; ++i)
        if (rc->remote.s[i] == Remote_SWITCH_NULL) {
            return REMOTE_ERROR;
        }
    return REMOTE_OK;
}


/**
  * @brief      Remote control keyboard data decoding
  * @param      key: Remote control keyboard data object
  * @param      v: Original remote control keyboard data value
  * @retval     NULL
  */
void Remote_DecodeKeyboardData(Remote_KeyboardTypeDef* key, uint16_t v) {
    const uint16_t KEY_MASK_W       = 1 << 0;
    const uint16_t KEY_MASK_S       = 1 << 1;
    const uint16_t KEY_MASK_A       = 1 << 2;
    const uint16_t KEY_MASK_D       = 1 << 3;
    const uint16_t KEY_MASK_SHIFT   = 1 << 4;
    const uint16_t KEY_MASK_CTRL    = 1 << 5;
    const uint16_t KEY_MASK_Q       = 1 << 6;
    const uint16_t KEY_MASK_E       = 1 << 7;
    const uint16_t KEY_MASK_R       = 1 << 8;
    const uint16_t KEY_MASK_F       = 1 << 9;
    const uint16_t KEY_MASK_G       = 1 << 10;
    const uint16_t KEY_MASK_Z       = 1 << 11;
    const uint16_t KEY_MASK_X       = 1 << 12;
    const uint16_t KEY_MASK_C       = 1 << 13;
    const uint16_t KEY_MASK_V       = 1 << 14;
    const uint16_t KEY_MASK_B       = 1 << 15;

    key->w      = (v & KEY_MASK_W    ) > 0;
    key->s      = (v & KEY_MASK_S    ) > 0;
    key->a      = (v & KEY_MASK_A    ) > 0;
    key->d      = (v & KEY_MASK_D    ) > 0;
    key->shift  = (v & KEY_MASK_SHIFT) > 0;
    key->ctrl   = (v & KEY_MASK_CTRL ) > 0;
    key->q      = (v & KEY_MASK_Q    ) > 0;
    key->e      = (v & KEY_MASK_E    ) > 0;
    key->r      = (v & KEY_MASK_R    ) > 0;
    key->f      = (v & KEY_MASK_F    ) > 0;
    key->g      = (v & KEY_MASK_G    ) > 0;
    key->z      = (v & KEY_MASK_Z    ) > 0;
    key->x      = (v & KEY_MASK_X    ) > 0;
    key->c      = (v & KEY_MASK_C    ) > 0;
    key->v      = (v & KEY_MASK_V    ) > 0;
    key->b      = (v & KEY_MASK_B    ) > 0;
}


/**
  * @brief      Remote control decoding function
  * @param      rc: The pointer points to the remote control data object
  * @param      buff: data buff
  * @param      rxdatalen: Data length
  * @retval     NULL
  */
void Remote_DecodeRemoteData(Remote_RemoteDataTypeDef* rc, uint8_t* buff, int rxdatalen) {

    if (rxdatalen != Const_Remote_RX_FRAME_LEN) {
        return;                                     //Data length error
    }
    
    rc->state           = Remote_STATE_PENDING;
    rc->last_update_time = HAL_GetTick();   
    
    /*buff[0] is ch0 low 8 bit��buff[1] low 3 bit ch0 high 3 bit*/
    rc->remote.ch[0]    = Remote_CancelChannelOffset(((uint16_t)buff[0] | (uint16_t)buff[1] << 8) & 0x07FF);
    /*buff[1] high 5 bit is ch1 low 5 bit��buff[2] low 6 bit is ch1 high 6 bit*/
    rc->remote.ch[1]    = Remote_CancelChannelOffset(((uint16_t)buff[1] >> 3 | (uint16_t)buff[2] << 5) & 0x07FF);
    /*buff[2] high 2 bit is ch2 low 2 bit, buff[3] low ch2 mid 8bit��buff[4] low 1 bit is ch2 high 1 bit*/
    rc->remote.ch[2]    = Remote_CancelChannelOffset(((uint16_t)buff[2] >> 6 | (uint16_t)buff[3] << 2 | (uint16_t)buff[4] << 10) & 0x07FF);
    /*buff[4] high 7 bit is ch3 low 7 bit��buff[5] low 4 bit is ch3 high 4 bit*/
    rc->remote.ch[3]    = Remote_CancelChannelOffset(((uint16_t)buff[4] >> 1 | (uint16_t)buff[5] << 7) & 0x07FF);
    /*buff[5] high 2 bit is s1*/
    rc->remote.s[0]     = Remote_ToSwitchState((buff[5] >> 6) & 0x03);
    /*buff[6] 6��7bit is s2*/
    rc->remote.s[1]     = Remote_ToSwitchState((buff[5] >> 4) & 0x03);
    /*buff[6],buff[7]is x*/
    rc->mouse.x         = ((int16_t)buff[6] | (int16_t)buff[7] << 8);
    /*buff[8],buff[9]is y*/
    rc->mouse.y         = ((int16_t)buff[8] | (int16_t)buff[9] << 8);
    /*buff[10],buff[11]is z*/
    rc->mouse.z         = ((int16_t)buff[10] | (int16_t)buff[11] << 8);
    /*buff[12] Is left key*/
    rc->mouse.l         = buff[12];
    /*buff[13] Is right key*/
    rc->mouse.r         = buff[13];
    /*buff[14],buff[15] Is the keyboard value*/
    Remote_DecodeKeyboardData(&(rc->key), ((int16_t)buff[14]) | ((int16_t)buff[15] << 8));
    /*buff[16],buff[17] Is Dial wheel*/
    rc->remote.ch[4]    = Remote_CancelChannelOffset(((uint16_t)buff[16] | (uint16_t)buff[17] << 8) & 0x07FF);

    if (rc->remote.ch[4] == -1024) rc->remote.ch[4] = 0;

    if (Remote_IsRemoteError(rc)) {
        rc->state       = Remote_STATE_ERROR;
        Remote_ResetRemoteData(rc);
        return;
    }
    rc->state           = Remote_STATE_CONNECTED;
}


/**
  * @brief      Initialize remote control data
  * @param      rc: Pointer to remote control object
  * @retval     NULL
  */
void Remote_ResetRemoteData(Remote_RemoteDataTypeDef* rc) {
    for (int i = 0; i < 5; ++i)
        rc->remote.ch[i] = 0;
    for (int i = 0; i < 2; ++i)
        rc->remote.s[i] = Remote_ToSwitchState(0);
    rc->mouse.x = 0;
    rc->mouse.y = 0;
    rc->mouse.z = 0;
    rc->mouse.l = 0;
    rc->mouse.r = 0;
    Remote_DecodeKeyboardData(&(rc->key), 0);
}


/**
  * @brief      Remove remote control offset
  * @param      ch: Original channel value
  * @retval     True value
  */
int16_t Remote_CancelChannelOffset(uint16_t ch) {
    return (int16_t) ch - Const_Remote_CHANNEL_VALUE_OFFSET;
}

#endif
