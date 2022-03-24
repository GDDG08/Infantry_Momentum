/*
 *  Project      : Infantry_Momentum
 *
 *  file         : init_ctrl.c
 *  Description  : This file contains Initialize control function
 *  LastEditors  : ����ؼ���ᶯ��
 *  Date         : 2021-05-04 20:53:31
 *  LastEditTime : 2021-05-16 01:04:27
 */

#include "init_ctrl.h"

#include "imu_periph.h"
#include "led_periph.h"
#include "minipc_periph.h"
#include "motor_periph.h"
#include "referee_periph.h"
#include "remote_periph.h"
#include "sensor_periph.h"
#include "servo_periph.h"

#include "cha_chassis_ctrl.h"
#include "cha_power_ctrl.h"
#include "cha_gimbal_ctrl.h"
#include "cha_referee_ctrl.h"

#include "gim_gimbal_ctrl.h"
#include "gim_remote_ctrl.h"
#include "gim_miniPC_ctrl.h"
#include "gim_shoot_ctrl.h"

#include "supercap_ctrl.h"

#include "buscomm_ctrl.h"
#include "watchdog_ctrl.h"

#include "const_lib.h"

#include "supercap_comm.h"

/**
 * @brief      Initialize all peripherals
 * @param      NULL
 * @retval     NULL
 */
void Init_InitAll() {
#if __FN_IF_ENABLE(__FN_SUPER_CAP)

    Sen_Init();
    DAC_Init();
    /* basis periph init    */
    CapComm_InitCapComm();

    Cap_Init();
    /* control function init    */

#endif

#if __FN_IF_ENABLE(__FN_INFANTRY_GIMBAL)
    LED_InitAllLEDs();
    Servo_InitAllServos();

    IMU_InitIMU();
    MiniPC_InitMiniPC();

    Motor_InitAllMotors();
    BusComm_InitBusComm();
    Can_InitFilterAndStart(&hcan1);
    Can_InitFilterAndStart(&hcan2);

    Gimbal_Init_Offset();
    Shooter_InitShooter();
    MiniPC_InitControl();

    Remote_InitRemote();
    Remotr_RemotrControlInit();

#endif

#if __FN_IF_ENABLE(__FN_INFANTRY_CHASSIS)
    LED_InitAllLEDs();

    Referee_InitReferee();

    Motor_InitAllMotors();
    BusComm_InitBusComm();

#if __FN_IF_ENABLE(__FN_SUPER_CAP_COMM)
    CapComm_InitCapComm();
#endif

    Can_InitFilterAndStart(&hcan1);
    Can_InitFilterAndStart(&hcan2);

    Cap_Init();
    Chassis_InitChassis();
    GimbalYaw_InitGimbalYaw();
#endif
}

/**
 * @brief      Initialization delay
 * @param      NULL
 * @retval     NULL
 */
void Init_MainLoop() {
}
