/*
 *  Project      : Infantry_Momentum
 * 
 *  file         : gim_shoot_ctrl.c
 *  Description  : This file contains Shooter control function
 *  LastEditors  : 动情丶卜灬动心
 *  Date         : 2021-05-04 20:53:31
 *  LastEditTime : 2021-05-09 03:55:15
 */

#include "gim_shoot_ctrl.h"

#if __FN_IF_ENABLE(__FN_CTRL_SHOOTER)

#include "buscomm_ctrl.h"
#include "const_lib.h"
#include "gim_gimbal_ctrl.h"
#include "gim_remote_ctrl.h"

Motor_MotorParamTypeDef Shooter_shooterLeftMotorParam;
Motor_MotorParamTypeDef Shooter_shooterRightMotorParam;
Motor_MotorParamTypeDef Shooter_feederMotorParam;

Shoot_StatusTypeDef Shooter_ShooterControl;

/**
  * @brief      shooter control initialization
  * @param      NULL
  * @retval     NULL
  */
void Shooter_InitShooter() {
    Shoot_StatusTypeDef* shooter = Shooter_GetShooterControlPtr();

    shooter->shooter_control = 1;

    shooter->feeder_mode = Feeder_NULL;
    shooter->heat_ctrl.shooter_17mm_cooling_heat = 0;
    shooter->heat_ctrl.shooter_17mm_cooling_rate = 0;

    shooter->shooter_mode = Shoot_NULL;
    shooter->shoot_speed.feeder_shoot_speed = 0;
    shooter->shoot_speed.left_shoot_speed = 0;
    shooter->shoot_speed.right_shoot_speed = 0;

    shooter->shooter_speed_15mpers = Const_Shooter15mpers;
    shooter->shooter_speed_18mpers = Const_Shooter18mpers;
    shooter->shooter_speed_30mpers = Const_Shooter30mpers;

    shooter->shooter_speed_offset = 0;  // for sb referee systerm

    Shooter_InitShooterMotor();

    Const_SetShooterPIDParam();
    // Initialization of motor parameters (including PID parameters)
    Shooter_HeatCtrlInit();
}

/**
  * @brief      Shooter control
  * @param      NULL
  * @retval     NULL
  */
void Shooter_Control() {
    Shooter_UpdataControlData();

    Shooter_ShootControl();

    Shooter_FeederControl();

    Shooter_ShooterMotorOutput();
}

/**
  * @brief      Gets the pointer to the shooter control data object
  * @param      NULL
  * @retval     Pointer to shooter control data object
  */
Shoot_StatusTypeDef* Shooter_GetShooterControlPtr() {
    return &Shooter_ShooterControl;
}

/**
  * @brief      Change frequent mode
  * @param      mode: Shooter mode
  * @retval     NULL
  */
void Shooter_ChangeShooterMode(Shoot_ShooterModeEnum mode) {
    Shoot_StatusTypeDef* shooter = Shooter_GetShooterControlPtr();
    BusComm_BusCommDataTypeDef* buscomm = BusComm_GetBusDataPtr();

    if (buscomm->main_shooter_power == 1)
        shooter->shooter_mode = mode;
    else
        shooter->shooter_mode = Shoot_NULL;
}

/**
  * @brief      Change shooter mode
  * @param      mode: Feeder mode
  * @retval     NULL
  */
void Shooter_ChangeFeederMode(Shoot_FeederModeEnum mode) {
    Shoot_StatusTypeDef* shooter = Shooter_GetShooterControlPtr();
    if (shooter->feeder_mode == Feeder_LOCKED_ROTOR)
        return;
    shooter->last_feeder_mode = shooter->feeder_mode;
    shooter->feeder_mode = mode;
    if ((shooter->feeder_mode != shooter->last_feeder_mode) &&
        ((shooter->last_feeder_mode == Feeder_LOW_CONTINUE) ||
         (shooter->last_feeder_mode == Feeder_FAST_CONTINUE) ||
         (shooter->last_feeder_mode == Feeder_REFEREE))) {
        shooter->feeder_mode = Feeder_FINISH;
        Shooter_AngleCorrect();
    }
}

/**
  * @brief      Initialize Shooter Motor
  * @param      NULL
  * @retval     NULL
  */
void Shooter_InitShooterMotor() {
    HAL_Delay(2000);
    for (int i = 0; i < 7; i++) {
        Motor_shooterMotorLeft.pwm.duty = 0.1 * i;
        Motor_shooterMotorRight.pwm.duty = 0.1 * i;
        Motor_SendMotorGroupOutput(&Motor_shooterMotors);
        HAL_Delay(200);
    }
}

/**
  * @brief      Initialize Shooter heat control
  * @param      NULL
  * @retval     NULL
  */
void Shooter_HeatCtrlInit() {
    Shoot_StatusTypeDef* shooter = Shooter_GetShooterControlPtr();
}

/**
  * @brief      Set referee shooter speed
  * @param      NULL
  * @retval     NULL
  */
float Shooter_GetRefereeSpeed() {
    BusComm_BusCommDataTypeDef* buscomm = BusComm_GetBusDataPtr();
    Shoot_StatusTypeDef* shooter = Shooter_GetShooterControlPtr();

    // This is a big big big shit for sb referee system
    // I will delete it after gaming
    static int sb_referee = 0;

    if (sb_referee != buscomm->heat_speed_limit)
        shooter->shooter_speed_offset = 0;

    float speed;
    switch (buscomm->heat_speed_limit) {
        case 15:
            speed = shooter->shooter_speed_15mpers;
            break;
        case 18:
            speed = shooter->shooter_speed_18mpers;
            break;
        case 30:
            speed = shooter->shooter_speed_30mpers;
            break;
        default:
            speed = shooter->shooter_speed_15mpers;
            break;
    }

    sb_referee = buscomm->heat_speed_limit;
    return speed;
}

/**
  * @brief      Updata control data
  * @param      NULL
  * @retval     NULL
  */
void Shooter_UpdataControlData() {
    Shoot_StatusTypeDef* shooter = Shooter_GetShooterControlPtr();
    BusComm_BusCommDataTypeDef* buscomm = BusComm_GetBusDataPtr();

    shooter->heat_ctrl.shooter_17mm_cooling_heat = (float)buscomm->heat_17mm;
    shooter->heat_ctrl.shooter_17mm_cooling_rate = (float)buscomm->heat_cooling_limit;

    Motor_ReadPWMEncoder(&Motor_shooterMotorLeft);
    Motor_ReadPWMEncoder(&Motor_shooterMotorRight);

    //    Shooter_FeederMotorLockedJudge();
}

/**
  * @brief      Set feeder motor speed
  * @param      speed: Feeder motor speed ref
  * @retval     NULL
  */
void Shooter_SetFeederSpeed(float speed) {
    Shoot_StatusTypeDef* shooter = Shooter_GetShooterControlPtr();

    shooter->shoot_speed.feeder_shoot_speed = speed;
}

/**
  * @brief      Set shooter motor speed
  * @param      speed: shooter motor speed ref
  * @retval     NULL
  */
void Shooter_SetShooterSpeed(float speed) {
    Shoot_StatusTypeDef* shooter = Shooter_GetShooterControlPtr();

    shooter->shoot_speed.left_shoot_speed = speed;
    shooter->shoot_speed.right_shoot_speed = speed;
}

/**
  * @brief      Force change shooter mode
  * @param      mode: Feeder mode
  * @retval     NULL
  */
void Shooter_ForceChangeFeederMode(Shoot_FeederModeEnum mode) {
    Shoot_StatusTypeDef* shooter = Shooter_GetShooterControlPtr();

    shooter->feeder_mode = mode;
}

/**
  * @brief      Motor locked rotor judge
  * @param      NULL
  * @retval     NULL
  */
void Shooter_FeederMotorLockedJudge() {
    Shoot_StatusTypeDef* shooter = Shooter_GetShooterControlPtr();

    static int count = 0;
    if (shooter->feeder_mode != Feeder_LOCKED_ROTOR) {
        if ((abs(Motor_feederMotor.encoder.current) >= Const_ShooterLockedCurrent) &&
            (abs(Motor_feederMotor.encoder.speed) <= Const_ShooterLockedSpeed)) {
            count++;
            if (count > Const_ShooterLockedTime) {
                Shooter_ForceChangeFeederMode(Feeder_LOCKED_ROTOR);
            }
        } else
            count = 0;
    }
}

/**
  * @brief      Motor locked handle
  * @param      NULL
  * @retval     NULL
  */
void Shooter_MotorLockedHandle() {
    static int count_reverse = 0;
    Shooter_SetFeederSpeed(Const_ShooterLockedReverseSpeed);
    count_reverse++;
    if (count_reverse >= Const_ShooterRelockedTime) {
        count_reverse = 0;
        Shooter_ForceChangeFeederMode(Feeder_NULL);
    }
}

/**
  * @brief      Correct stop angle
  * @param      NULL
  * @retval     NULL
  */
void Shooter_AngleCorrect() {
    Motor_feederMotor.pid_pos.ref = Motor_feederMotor.pid_pos.fdb;
    //    Motor_feederMotor.pid_pos.ref = ((int)(Motor_feederMotor.pid_pos.fdb + 40.0f) / 45) * 45;
}

void Shooter_RealAngleCorrect() {
    Motor_feederMotor.pid_pos.ref = ((int)(Motor_feederMotor.pid_pos.fdb + 40.0f) / 45) * 45;
}

/**
  * @brief      Shooter heat control
  * @param      NULL
  * @retval     pid_num
  */
uint8_t Shooter_HeatCtrl() {
    Shoot_StatusTypeDef* shooter = Shooter_GetShooterControlPtr();

    if ((shooter->heat_ctrl.shooter_17mm_cooling_rate - shooter->heat_ctrl.shooter_17mm_cooling_heat) >= Const_HeatCtrlFastLimit) {  // sufficient heat remain, fast shooting
        shooter->heat_ctrl.current_speed = Const_FeederSlowSpeed;
        shooter->heat_ctrl.current_pidnum = 1;
        Shooter_SetFeederSpeed(shooter->heat_ctrl.current_speed);
        shooter->heat_ctrl.heat_tracking = 0;
    } else {
        if ((shooter->heat_ctrl.shooter_17mm_cooling_rate - shooter->heat_ctrl.shooter_17mm_cooling_heat) >= Const_HeatCtrlSlowLimit) {
            shooter->heat_ctrl.current_speed = Const_FeederSlowSpeed;
            shooter->heat_ctrl.current_pidnum = 1;
            Shooter_SetFeederSpeed(shooter->heat_ctrl.current_speed);
            shooter->heat_ctrl.heat_tracking = 0;
        } else {
            if ((shooter->heat_ctrl.shooter_17mm_cooling_rate - shooter->heat_ctrl.shooter_17mm_cooling_heat) <= Const_HeatCtrlWaitLimit) {
                shooter->heat_ctrl.current_speed = Const_FeederWaitSpeed;
                shooter->heat_ctrl.current_pidnum = 1;
                Shooter_SetFeederSpeed(shooter->heat_ctrl.current_speed);
                shooter->heat_ctrl.heat_tracking = 0;
            } else {
                if ((shooter->heat_ctrl.shooter_17mm_cooling_rate - shooter->heat_ctrl.shooter_17mm_cooling_heat) <= Const_HeatCtrlStopLimit) {
                    // insufficient heat remain, single shooting
                    //   shooter->heat_ctrl.heat_tracking += shooter->heat_ctrl.heat_tracking / 1000.0;
                    //   if (shooter->heat_ctrl.heat_tracking >= Const_HeatCtrlSingleCount) {
                    //       shooter->heat_ctrl.heat_tracking = 0;
                    //       Shooter_SingleShootReset();
                    //   }
                    //   Shooter_SingleShootCtrl();
                    //   shooter->heat_ctrl.current_pidnum = 2
                    // no heat remain, stop shooting
                    shooter->heat_ctrl.heat_tracking = 0;
                    Shooter_AngleCorrect();
                    shooter->heat_ctrl.current_pidnum = 2;
                }
            }
        }
    }
    return shooter->heat_ctrl.current_pidnum;
}

/**
  * @brief      Shooter control
  * @param      NULL
  * @retval     NULL
  */
void Shooter_ShootControl() {
    Shoot_StatusTypeDef* shooter = Shooter_GetShooterControlPtr();

    switch (shooter->shooter_mode) {
        case Shoot_NULL:
            GPIO_Close(LASER);
            GPIO_Close(BULLET_CHARGING);
            Shooter_SetShooterSpeed(0);
            break;
        case Shoot_FAST:
            GPIO_Open(LASER);
            GPIO_Open(BULLET_CHARGING);
            Shooter_SetShooterSpeed(Const_ShooterFastSpeed);
            break;
        case Shoot_SLOW:
            GPIO_Open(LASER);
            GPIO_Open(BULLET_CHARGING);
            Shooter_SetShooterSpeed(Const_ShooterSlowSpeed);
            break;
        case Shoot_REFEREE:
            GPIO_Open(LASER);
            GPIO_Open(BULLET_CHARGING);
            Shooter_SetShooterSpeed(Shooter_GetRefereeSpeed() + shooter->shooter_speed_offset);
        default:
            break;
    }

    Motor_SetMotorRef(&Motor_shooterMotorRight, shooter->shoot_speed.left_shoot_speed);
    Motor_SetMotorRef(&Motor_shooterMotorLeft, shooter->shoot_speed.left_shoot_speed);

    Motor_CalcMotorOutput(&Motor_shooterMotorRight, &Shooter_shooterRightMotorParam);
    Motor_CalcMotorOutput(&Motor_shooterMotorLeft, &Shooter_shooterLeftMotorParam);
}

/**
  * @brief      Shooter feeder control: single shooting
  * @param      NULL
  * @retval     NULL
  */
void Shooter_SingleShootCtrl() {
    Shoot_StatusTypeDef* shooter = Shooter_GetShooterControlPtr();

    if (fabs(Motor_feederMotor.pid_pos.fdb - Motor_feederMotor.pid_pos.ref) > 1.0f) {  // feeder motor not ready
        //return;     // do nothing
    }
    if (!shooter->single_shoot_done) {  // not shoot yet
        Motor_feederMotor.pid_pos.ref += 45.0f;
        shooter->single_shoot_done = 1;
    }
}

/**
  * @brief      Shooter feeder control: reset single shooting
  * @param      NULL
  * @retval     NULL
  */
void Shooter_SingleShootReset() {
    Shoot_StatusTypeDef* shooter = Shooter_GetShooterControlPtr();

    shooter->single_shoot_done = 0;
}

/**
  * @brief      Shooter feeder control
  * @param      NULL
  * @retval     NULL
  */
void Shooter_FeederControl() {
    Shoot_StatusTypeDef* shooter = Shooter_GetShooterControlPtr();

    int current_pid_num = 0;
    switch (shooter->feeder_mode) {
        case Feeder_NULL:
            current_pid_num = 1;
            Shooter_SetFeederSpeed(0);
            break;
        case Feeder_SINGLE:
            current_pid_num = 2;
            Shooter_SingleShootCtrl();
            break;
        case Feeder_FAST_CONTINUE:
            current_pid_num = 1;
            Shooter_SetFeederSpeed(Const_FeederFastSpeed);
            break;
        case Feeder_LOW_CONTINUE:
            current_pid_num = 1;
            Shooter_SetFeederSpeed(Const_FeederSlowSpeed);
            break;
        case Feeder_LOCKED_ROTOR:
            current_pid_num = 1;
            Shooter_MotorLockedHandle();
            break;
        case Feeder_REFEREE:
            current_pid_num = Shooter_HeatCtrl();
            break;
        case Feeder_FINISH:
            current_pid_num = 2;
            break;
        default:
            break;
    }

    Motor_feederMotor.pid_spd.ref = shooter->shoot_speed.feeder_shoot_speed;
    Motor_CalcMotorOutputRingOverrided(&Motor_feederMotor, current_pid_num, &Shooter_feederMotorParam);
}

/**
  * @brief      Output shooter motor
  * @param      NULL
  * @retval     NULL
  */
void Shooter_ShooterMotorOutput() {
    Shoot_StatusTypeDef* shooter = Shooter_GetShooterControlPtr();

    if (shooter->shooter_control == 1) {
        Motor_SendMotorGroupOutput(&Motor_shooterMotors);
        Motor_SendMotorGroupOutput(&Motor_feederMotors);
    }
}

#endif
