/*
 *  Project      : Infantry_Momentum
 * 
 *  file         : buscomm_cmd.c
 *  Description  : This file is for idiot Can communication
 *  LastEditors  : 动情丶卜灬动心
 *  Date         : 2021-05-09 03:52:32
 *  LastEditTime : 2021-05-09 07:29:11
 */

#include "buscomm_cmd.h"
#include "buscomm_ctrl.h"
#include "const_lib.h"

#if __FN_IF_ENABLE(__FN_INFANTRY_CHASSIS)
    #include "cha_chassis_ctrl.h"
    #include "cha_gimbal_ctrl.h"
    #include "referee_periph.h"
    #include "cha_power_ctrl.h"
#endif

#if __FN_IF_ENABLE(__FN_INFANTRY_GIMBAL)
    #include "gim_gimbal_ctrl.h"
#endif

#if __FN_IF_ENABLE(__FN_SUPER_CAP)
    #include "supercap_ctrl.h"
#endif

const uint8_t CMD_SET_YAW_RELATIVE_ANGLE        = 0xA1;
const uint8_t CMD_SET_ROBOT_ID_POWER_LIMIT      = 0xA2;
const uint8_t CMD_SET_17MM_DATA                 = 0xA3;
const uint8_t CMD_SET_COOLING_DATA              = 0xA4;

const uint8_t CMD_SET_MODE                      = 0xB2;
const uint8_t CMD_SET_YAW_REF                   = 0xB3;
const uint8_t CMD_SET_IMU_POS                   = 0xB4;
const uint8_t CMD_SET_IMU_SPD                   = 0xB5;
const uint8_t CMD_SET_CHA_FB                    = 0xB6;
const uint8_t CMD_SET_CHA_LR                    = 0xB7;

const uint8_t CMD_SENT_CAP_STATE                = 0xC2;

const uint8_t CMD_CHASSIS_SENT_PACK_1           = 0x01;
const uint8_t CMD_CHASSIS_SENT_PACK_2           = 0x02;
const uint8_t CMD_CHASSIS_SENT_PACK_3           = 0x03;
const uint8_t CMD_CHASSIS_SENT_PACK_4           = 0x04;

const uint8_t CMD_GIMBAL_SENT_PACK_1            = 0x01;
const uint8_t CMD_GIMBAL_SENT_PACK_2            = 0x02;
const uint8_t CMD_GIMBAL_SENT_PACK_3            = 0x03;
const uint8_t CMD_GIMBAL_SENT_PACK_4            = 0x04;
const uint8_t CMD_GIMBAL_SENT_PACK_5            = 0x05;
const uint8_t CMD_GIMBAL_SENT_PACK_6            = 0x06;

const uint8_t CMD_SUPERCAP_SENT_PACK_1          = 0x01;

static void _sent_yaw_relative_angle(uint8_t buff[]);
static void _sent_robot_id_power_limit(uint8_t buff[]);
static void _sent_17mm_data(uint8_t buff[]);
static void _sent_cooling_data(uint8_t buff[]);
static void _sent_mode(uint8_t buff[]);
static void _sent_yaw_ref(uint8_t buff[]);
static void _sent_imu_pos(uint8_t buff[]);
static void _sent_imu_spd(uint8_t buff[]);
static void _sent_chassis_fb(uint8_t buff[]);
static void _sent_chassis_lr(uint8_t buff[]);
static void _sent_cap_state(uint8_t buff[]);
static void _set_yaw_relative_angle(uint8_t buff[]);
static void _set_robot_id_power_limit(uint8_t buff[]);
static void _set_17mm_data(uint8_t buff[]);
static void _set_colling_data(uint8_t buff[]);
static void _set_mode(uint8_t buff[]);
static void _set_yaw_ref(uint8_t buff[]);
static void _set_imu_pos(uint8_t buff[]);
static void _set_imu_spd(uint8_t buff[]);
static void _set_cha_fb(uint8_t buff[]);
static void _set_cha_lr(uint8_t buff[]);
static void _set_cap_state(uint8_t buff[]);

BusCmd_TableEntry Buscmd_Revice[11] = {
    {CMD_SET_YAW_RELATIVE_ANGLE  , &_set_yaw_relative_angle     },
    {CMD_SET_ROBOT_ID_POWER_LIMIT, &_set_robot_id_power_limit   },
    {CMD_SET_17MM_DATA           , &_set_17mm_data              },
    {CMD_SET_COOLING_DATA        , &_set_colling_data           },
    {CMD_SET_MODE                , &_set_mode                   },
    {CMD_SET_YAW_REF             , &_set_yaw_ref                },
    {CMD_SET_IMU_POS             , &_set_imu_pos                },
    {CMD_SET_IMU_SPD             , &_set_imu_spd                },
    {CMD_SET_CHA_FB              , &_set_cha_fb                 },
    {CMD_SET_CHA_LR              , &_set_cha_lr                 },
    {CMD_SENT_CAP_STATE          , &_set_cap_state              }
};

BusCmd_TableEntry Buscmd_GimSent[6] = {
    {CMD_GIMBAL_SENT_PACK_1, &_sent_chassis_lr },
    {CMD_GIMBAL_SENT_PACK_2, &_sent_chassis_fb },
    {CMD_GIMBAL_SENT_PACK_3, &_sent_imu_spd    },
    {CMD_GIMBAL_SENT_PACK_4, &_sent_imu_pos    },
    {CMD_GIMBAL_SENT_PACK_5, &_sent_yaw_ref    },
    {CMD_GIMBAL_SENT_PACK_6, &_sent_mode       }
};

BusCmd_TableEntry Buscmd_ChaSent[4] = {
    {CMD_CHASSIS_SENT_PACK_1, &_sent_cooling_data           },
    {CMD_CHASSIS_SENT_PACK_1, &_sent_17mm_data              },
    {CMD_CHASSIS_SENT_PACK_1, &_sent_robot_id_power_limit   },
    {CMD_CHASSIS_SENT_PACK_1, &_sent_yaw_relative_angle     }
};

BusCmd_TableEntry Buscmd_CapSent[1] = {
    {CMD_SUPERCAP_SENT_PACK_1, &_sent_cap_state}
};



/*      sent functions driver       */
static void _sent_yaw_relative_angle(uint8_t buff[]) {
    BusComm_BusCommDataTypeDef *buscomm = BusComm_GetBusDataPtr();
    CAN_TxHeaderTypeDef *pheader = &BusComm_ChassisCanTxHeader;
   
    memset(buff, 0, 8);
    buff[0] = Const_BusComm_FRAME_HEADER_SOF;
    buff[1] = CMD_SET_YAW_RELATIVE_ANGLE;
    float2buff(buscomm->yaw_relative_angle, buff + 2);
    uint16_t checksum = 0;
    for (int i = 0; i < 7; ++i)
        checksum += buff[i];
    buff[7] = checksum & 0xff;
    Can_SendMessage(Const_BusComm_CAN_HANDLER, pheader, buff);
}

static void _sent_robot_id_power_limit(uint8_t buff[]) {
    BusComm_BusCommDataTypeDef *buscomm = BusComm_GetBusDataPtr();
    CAN_TxHeaderTypeDef *pheader = &BusComm_ChassisCanTxHeader;
   
    memset(buff, 0, 8);
    buff[0] = Const_BusComm_FRAME_HEADER_SOF;
    buff[1] = CMD_SET_ROBOT_ID_POWER_LIMIT;
    buff[2] = buscomm->robot_id;
    buff[3] =buscomm->power_limit;
    uint16_t checksum = 0;
    for (int i = 0; i < 7; ++i)
        checksum += buff[i];
    buff[7] = checksum & 0xff;
    Can_SendMessage(Const_BusComm_CAN_HANDLER, pheader, buff);
}


static void _sent_17mm_data(uint8_t buff[]) {
    BusComm_BusCommDataTypeDef *buscomm = BusComm_GetBusDataPtr();
    CAN_TxHeaderTypeDef *pheader = &BusComm_ChassisCanTxHeader;
   
    memset(buff, 0, 8);
    buff[0] = Const_BusComm_FRAME_HEADER_SOF;
    buff[1] = CMD_SET_17MM_DATA;
    ui162buff(buscomm->heat_17mm, buff + 2);
    ui162buff(buscomm->heat_speed_limit, buff + 4);
    uint16_t checksum = 0;
    for (int i = 0; i < 7; ++i)
        checksum += buff[i];
    buff[7] = checksum & 0xff;
    Can_SendMessage(Const_BusComm_CAN_HANDLER, pheader, buff);
}


static void _sent_cooling_data(uint8_t buff[]) {
    BusComm_BusCommDataTypeDef *buscomm = BusComm_GetBusDataPtr();
    CAN_TxHeaderTypeDef *pheader = &BusComm_ChassisCanTxHeader;
    
    memset(buff, 0, 8);
    buff[0] = Const_BusComm_FRAME_HEADER_SOF;
    buff[1] = CMD_SET_COOLING_DATA;
    ui162buff(buscomm->heat_cooling_rate, buff + 2);
    ui162buff(buscomm->heat_cooling_limit, buff + 4);
    uint16_t checksum = 0;
    for (int i = 0; i < 7; ++i)
        checksum += buff[i];
    buff[7] = checksum & 0xff;
    Can_SendMessage(Const_BusComm_CAN_HANDLER, pheader, buff);
}


static void _sent_mode(uint8_t buff[]) {
    BusComm_BusCommDataTypeDef *buscomm = BusComm_GetBusDataPtr();
    CAN_TxHeaderTypeDef *pheader = &BusComm_GimbalCanTxHeader;
 
    memset(buff, 0, 8);
    buff[0] = Const_BusComm_FRAME_HEADER_SOF;
    buff[1] = CMD_SET_MODE;
    buff[2] = buscomm->gimbal_yaw_mode;
    buff[3] = buscomm->power_limit_mode;
    buff[4] = buscomm->cap_charge_mode;
    buff[5] = buscomm->cap_mode;
    buff[6] = buscomm->chassis_mode;
    uint16_t checksum = 0;
    for (int i = 0; i < 7; ++i)
        checksum += buff[i];
    buff[7] = checksum & 0xff;
    Can_SendMessage(Const_BusComm_CAN_HANDLER, pheader, buff);
}


static void _sent_yaw_ref(uint8_t buff[]) {
    BusComm_BusCommDataTypeDef *buscomm = BusComm_GetBusDataPtr();
    CAN_TxHeaderTypeDef *pheader = &BusComm_GimbalCanTxHeader;
   
    memset(buff, 0, 8);
    buff[0] = Const_BusComm_FRAME_HEADER_SOF;
    buff[1] = CMD_SET_YAW_REF;
    float2buff(buscomm->gimbal_yaw_ref, buff + 2);
    uint16_t checksum = 0;
    for (int i = 0; i < 7; ++i)
        checksum += buff[i];
    buff[7] = checksum & 0xff;
    Can_SendMessage(Const_BusComm_CAN_HANDLER, pheader, buff);
}


static void _sent_imu_pos(uint8_t buff[]) {
    BusComm_BusCommDataTypeDef *buscomm = BusComm_GetBusDataPtr();
    CAN_TxHeaderTypeDef *pheader = &BusComm_GimbalCanTxHeader;
 
    memset(buff, 0, 8);
    buff[0] = Const_BusComm_FRAME_HEADER_SOF;
    buff[1] = CMD_SET_IMU_POS;
    float2buff(buscomm->gimbal_imu_pos, buff + 2);
    uint16_t checksum = 0;
    for (int i = 0; i < 7; ++i)
        checksum += buff[i];
    buff[7] = checksum & 0xff;
    Can_SendMessage(Const_BusComm_CAN_HANDLER, pheader, buff);
}


static void _sent_imu_spd(uint8_t buff[]) {
    BusComm_BusCommDataTypeDef *buscomm = BusComm_GetBusDataPtr();
    CAN_TxHeaderTypeDef *pheader = &BusComm_GimbalCanTxHeader;  
    
    memset(buff, 0, 8);
    buff[0] = Const_BusComm_FRAME_HEADER_SOF;
    buff[1] = CMD_SET_IMU_SPD;
    float2buff(buscomm->gimbal_imu_spd, buff + 2);
    uint16_t checksum = 0;
    for (int i = 0; i < 7; ++i)
        checksum += buff[i];
    buff[7] = checksum & 0xff;
    Can_SendMessage(Const_BusComm_CAN_HANDLER, pheader, buff);
}


static void _sent_chassis_fb(uint8_t buff[]) {
    BusComm_BusCommDataTypeDef *buscomm = BusComm_GetBusDataPtr();
    CAN_TxHeaderTypeDef *pheader = &BusComm_GimbalCanTxHeader;
  
    memset(buff, 0, 8);
    buff[0] = Const_BusComm_FRAME_HEADER_SOF;
    buff[1] = CMD_SET_CHA_FB;
    float2buff(buscomm->chassis_fb_ref, buff + 2);
    uint16_t checksum = 0;
    for (int i = 0; i < 7; ++i)
        checksum += buff[i];
    buff[7] = checksum & 0xff;
    Can_SendMessage(Const_BusComm_CAN_HANDLER, pheader, buff);
}

static void _sent_chassis_lr(uint8_t buff[]) {
    BusComm_BusCommDataTypeDef *buscomm = BusComm_GetBusDataPtr();
    CAN_TxHeaderTypeDef *pheader = &BusComm_GimbalCanTxHeader;

    memset(buff, 0, 8);
    buff[0] = Const_BusComm_FRAME_HEADER_SOF;
    buff[1] = CMD_SET_CHA_LR;
    float2buff(buscomm->chassis_lr_ref, buff + 2);
    uint16_t checksum = 0;
    for (int i = 0; i < 7; ++i)
        checksum += buff[i];
    buff[7] = checksum & 0xff;
    Can_SendMessage(Const_BusComm_CAN_HANDLER, pheader, buff);
}

static void _sent_cap_state(uint8_t buff[]) {
    BusComm_BusCommDataTypeDef *buscomm = BusComm_GetBusDataPtr();
    CAN_TxHeaderTypeDef *pheader = &BusComm_SuperCapCanTxHeader;  
    
    memset(buff, 0, 8);
    buff[0] = Const_BusComm_FRAME_HEADER_SOF;
    buff[1] = CMD_SENT_CAP_STATE;
    buff[2] = buscomm->cap_state;
    buff[3] = buscomm->cap_rest_energy;
    uint16_t checksum = 0;
    for (int i = 0; i < 7; ++i)
        checksum += buff[i];
    buff[7] = checksum & 0xff;
    Can_SendMessage(Const_BusComm_CAN_HANDLER, pheader, buff);
}



int count6;
float rate6;
/*          function driver      */
static void _set_yaw_relative_angle(uint8_t buff[]) {
                       count6++;
       rate6 = 1000 * count6 / HAL_GetTick();
    BusComm_BusCommDataTypeDef *buscomm = BusComm_GetBusDataPtr();
    buscomm->yaw_relative_angle = buff2float(buff + 2);
}
int count7;
float rate7;
static void _set_robot_id_power_limit(uint8_t buff[]) {
                       count7++;
       rate7 = 1000 * count7 / HAL_GetTick();
    BusComm_BusCommDataTypeDef *buscomm = BusComm_GetBusDataPtr();
    buscomm->robot_id = buff[2];
    buscomm->power_limit = buff[3];
}
int count8;
float rate8;
static void _set_17mm_data(uint8_t buff[]) {
                       count8++;
       rate8 = 1000 * count8 / HAL_GetTick();
    BusComm_BusCommDataTypeDef *buscomm = BusComm_GetBusDataPtr();
    buscomm->heat_17mm = buff2ui16(buff + 2);
    buscomm->heat_speed_limit = buff2ui16(buff + 4);
}
int count9;
float rate9;
static void _set_colling_data(uint8_t buff[]) {
                       count9++;
       rate9 = 1000 * count9 / HAL_GetTick();
    BusComm_BusCommDataTypeDef *buscomm = BusComm_GetBusDataPtr();
    buscomm->heat_cooling_rate = buff2ui16(buff + 2);
    buscomm->heat_cooling_limit = buff2ui16(buff + 4);
}

int count10;
float rate10;
static void _set_mode(uint8_t buff[]) {
                       count10++;
       rate10 = 1000 * count10 / HAL_GetTick();
    BusComm_BusCommDataTypeDef *buscomm = BusComm_GetBusDataPtr();
    buscomm->gimbal_yaw_mode = buff[2];
    buscomm->power_limit_mode = buff[3];
    buscomm->cap_charge_mode = buff[4];
    buscomm->cap_mode = buff[5];
    buscomm->chassis_mode = buff[6];
    _cmd_mode_control();
}

int count1;
float rate1;
static void _set_yaw_ref(uint8_t buff[]) {
           count1++;
       rate1 = 1000 * count1 / HAL_GetTick();
    BusComm_BusCommDataTypeDef *buscomm = BusComm_GetBusDataPtr();
    buscomm->gimbal_yaw_ref = buff2float(buff + 2);
}
int count2;
float rate2;
static void _set_imu_pos(uint8_t buff[]) {
               count2++;
       rate2 = 1000 * count2 / HAL_GetTick();
    BusComm_BusCommDataTypeDef *buscomm = BusComm_GetBusDataPtr();
    buscomm->gimbal_imu_pos = buff2float(buff + 2);
}
int count3;
float rate3;
static void _set_imu_spd(uint8_t buff[]) {
                   count3++;
       rate3 = 1000 * count3 / HAL_GetTick();
    BusComm_BusCommDataTypeDef *buscomm = BusComm_GetBusDataPtr();
    buscomm->gimbal_imu_spd = buff2float(buff + 2);
}
int count4;
float rate4;
static void _set_cha_fb(uint8_t buff[]) {
                       count4++;
       rate4 = 1000 * count4 / HAL_GetTick();
    BusComm_BusCommDataTypeDef *buscomm = BusComm_GetBusDataPtr();
    buscomm->chassis_fb_ref = buff2float(buff + 2);
}
int count5;
float rate5;
static void _set_cha_lr(uint8_t buff[]) {
                   count5++;
       rate5 = 1000 * count5 / HAL_GetTick();
    BusComm_BusCommDataTypeDef *buscomm = BusComm_GetBusDataPtr();
    buscomm->chassis_lr_ref = buff2float(buff +2 );
}

static void _set_cap_state(uint8_t buff[]) {
                   count6++;
       rate6 = 1000 * count6 / HAL_GetTick();
    BusComm_BusCommDataTypeDef *buscomm = BusComm_GetBusDataPtr();
    buscomm->cap_state = buff[2];
    buscomm->cap_rest_energy = buff[3];
}
