/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "configure.h"
#include "gim_gimbal_ctrl.h"
#include "gim_miniPC_ctrl.h"
#include "gim_shoot_ctrl.h"
#include "gim_remote_ctrl.h"
#include "cha_referee_ctrl.h"
#include "cha_chassis_ctrl.h"
#include "cha_gimbal_ctrl.h"
#include "cha_power_ctrl.h"
#include "supercap_ctrl.h"
#include "buscomm_ctrl.h"
#include "watchdog_ctrl.h"
#include "supercap_comm.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId GimbalHandle;
osThreadId BusCommHandle;
osThreadId RemoteHandle;
osThreadId ChassisHandle;
osThreadId SuperCapHandle;
osThreadId ShootHandle;
osThreadId MiniPCHandle;
osThreadId RefereeHandle;
osThreadId WatchDogHandle;
osMessageQId BusCommSend_QueueHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void Gimbal_Task(void const * argument);
void BusComm_Task(void const * argument);
void Remote_Task(void const * argument);
void Chassis_Task(void const * argument);
void SuperCap_Task(void const * argument);
void Shoot_Task(void const * argument);
void MiniPC_Task(void const * argument);
void Referee_Task(void const * argument);
void WatchDog_Task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t** ppxIdleTaskTCBBuffer, StackType_t** ppxIdleTaskStackBuffer, uint32_t* pulIdleTaskStackSize) {
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
    *ppxIdleTaskStackBuffer = &xIdleStack[0];
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
    /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of BusCommSend_Queue */
  osMessageQDef(BusCommSend_Queue, 30, uint8_t);
  BusCommSend_QueueHandle = osMessageCreate(osMessageQ(BusCommSend_Queue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of Gimbal */
  osThreadDef(Gimbal, Gimbal_Task, osPriorityNormal, 0, 128);
  GimbalHandle = osThreadCreate(osThread(Gimbal), NULL);

  /* definition and creation of BusComm */
  osThreadDef(BusComm, BusComm_Task, osPriorityNormal, 0, 128);
  BusCommHandle = osThreadCreate(osThread(BusComm), NULL);

  /* definition and creation of Remote */
  osThreadDef(Remote, Remote_Task, osPriorityNormal, 0, 128);
  RemoteHandle = osThreadCreate(osThread(Remote), NULL);

  /* definition and creation of Chassis */
  osThreadDef(Chassis, Chassis_Task, osPriorityNormal, 0, 128);
  ChassisHandle = osThreadCreate(osThread(Chassis), NULL);

  /* definition and creation of SuperCap */
  osThreadDef(SuperCap, SuperCap_Task, osPriorityNormal, 0, 128);
  SuperCapHandle = osThreadCreate(osThread(SuperCap), NULL);

  /* definition and creation of Shoot */
  osThreadDef(Shoot, Shoot_Task, osPriorityNormal, 0, 128);
  ShootHandle = osThreadCreate(osThread(Shoot), NULL);

  /* definition and creation of MiniPC */
  osThreadDef(MiniPC, MiniPC_Task, osPriorityNormal, 0, 128);
  MiniPCHandle = osThreadCreate(osThread(MiniPC), NULL);

  /* definition and creation of Referee */
  osThreadDef(Referee, Referee_Task, osPriorityNormal, 0, 128);
  RefereeHandle = osThreadCreate(osThread(Referee), NULL);

  /* definition and creation of WatchDog */
  osThreadDef(WatchDog, WatchDog_Task, osPriorityNormal, 0, 128);
  WatchDogHandle = osThreadCreate(osThread(WatchDog), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
    /* Infinite loop */
    for (;;) {
        osDelay(1);
    }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Gimbal_Task */
/**
* @brief Function implementing the Gimbal thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Gimbal_Task */
void Gimbal_Task(void const * argument)
{
  /* USER CODE BEGIN Gimbal_Task */
    /* Infinite loop */
    for (;;) {
#if __FN_IF_ENABLE(__FN_INFANTRY_GIMBAL)
        MiniPC_CalcAutoAim();
        Gimbal_CtrlPitch();
        Gimbal_CtrlYaw();
        GimbalPitch_Output();
#endif
#if __FN_IF_ENABLE(__FN_INFANTRY_CHASSIS)
        GimbalYaw_Control();
        GimbalYaw_Output();
#endif
        osDelay(1);
    }
  /* USER CODE END Gimbal_Task */
}

/* USER CODE BEGIN Header_BusComm_Task */
/**
* @brief Function implementing the BusComm thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_BusComm_Task */
void BusComm_Task(void const * argument)
{
  /* USER CODE BEGIN BusComm_Task */
    /* Infinite loop */
    for (;;) {
#if __FN_IF_ENABLE(__FN_INFANTRY)
        BusComm_SendBusCommData();
#endif
        osDelay(4);
    }
  /* USER CODE END BusComm_Task */
}

/* USER CODE BEGIN Header_Remote_Task */
/**
* @brief Function implementing the Remote thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Remote_Task */
void Remote_Task(void const * argument)
{
  /* USER CODE BEGIN Remote_Task */
    /* Infinite loop */
    for (;;) {
#if __FN_IF_ENABLE(__FN_CTRL_REMOTE)
        Remote_ControlCom();
#endif
        osDelay(1);
    }
  /* USER CODE END Remote_Task */
}

/* USER CODE BEGIN Header_Chassis_Task */
/**
* @brief Function implementing the Chassis thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Chassis_Task */
void Chassis_Task(void const * argument)
{
  /* USER CODE BEGIN Chassis_Task */
    /* Infinite loop */
    for (;;) {
#if __FN_IF_ENABLE(__FN_CTRL_CHASSIS)
        Chassis_Control();
        Chassis_Output();
#endif
        osDelay(1);
    }
  /* USER CODE END Chassis_Task */
}

/* USER CODE BEGIN Header_SuperCap_Task */
/**
* @brief Function implementing the SuperCap thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SuperCap_Task */
void SuperCap_Task(void const * argument)
{
  /* USER CODE BEGIN SuperCap_Task */
    /* Infinite loop */
    for (;;) {
#if __FN_IF_ENABLE(__FN_SUPER_CAP_COMM)
#if __FN_IF_ENABLE(__FN_SUPER_CAP)
        Cap_Control();
#endif
        CapComm_SendCapCommData();
#endif
        osDelay(100);
    }
  /* USER CODE END SuperCap_Task */
}

/* USER CODE BEGIN Header_Shoot_Task */
/**
* @brief Function implementing the Shoot thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Shoot_Task */
void Shoot_Task(void const * argument)
{
  /* USER CODE BEGIN Shoot_Task */
    /* Infinite loop */
    for (;;) {
#if __FN_IF_ENABLE(__FN_CTRL_SHOOTER)
        Shooter_Control();
#endif
        osDelay(1);
    }
  /* USER CODE END Shoot_Task */
}

/* USER CODE BEGIN Header_MiniPC_Task */
/**
* @brief Function implementing the MiniPC thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MiniPC_Task */
void MiniPC_Task(void const * argument)
{
  /* USER CODE BEGIN MiniPC_Task */
    /* Infinite loop */
    for (;;) {
#if __FN_IF_ENABLE(__FN_PERIPH_MINIPC)
        MiniPC_SendHeartPacket();
#endif
        osDelay(100);
    }
  /* USER CODE END MiniPC_Task */
}

/* USER CODE BEGIN Header_Referee_Task */
/**
* @brief Function implementing the Referee thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Referee_Task */
void Referee_Task(void const * argument)
{
  /* USER CODE BEGIN Referee_Task */
    /* Infinite loop */
    for (;;) {
#if __FN_IF_ENABLE(__FN_CTRL_REFEREE)
        if (yyy_love == 1) {
            Referee_Update();
        }
#endif
        osDelay(200);
    }
  /* USER CODE END Referee_Task */
}

/* USER CODE BEGIN Header_WatchDog_Task */
/**
* @brief Function implementing the WatchDog thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_WatchDog_Task */
void WatchDog_Task(void const * argument)
{
  /* USER CODE BEGIN WatchDog_Task */
    /* Infinite loop */
    for (;;) {
        WatchDog_FeedDog();
        osDelay(1);
    }
  /* USER CODE END WatchDog_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
