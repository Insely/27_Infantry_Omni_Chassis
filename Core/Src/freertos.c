/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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

#include "remote_control.h"
#include "Chassis.h"
#include "Gimbal.h"
#include "Global_status.h"
#include "Auto_control.h"
#include "shoot.h"
#include "music.h"

#include "referee_system.h"
#include "supercup.h"
#include "LED.h"
#include "DT7.h"
#include "VT13.h"
#include "motor.h"
#include <cmsis_os2.h>
#include "ui.h"

#include "iwdg.h"
#include "buzzer.h"
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
uint32_t color = 0;
/* USER CODE END Variables */
/* Definitions for Remote_control */
osThreadId_t Remote_controlHandle;
const osThreadAttr_t Remote_control_attributes = {
  .name = "Remote_control",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Gimbal */
osThreadId_t GimbalHandle;
const osThreadAttr_t Gimbal_attributes = {
  .name = "Gimbal",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Chassis */
osThreadId_t ChassisHandle;
const osThreadAttr_t Chassis_attributes = {
  .name = "Chassis",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Motor_control */
osThreadId_t Motor_controlHandle;
const osThreadAttr_t Motor_control_attributes = {
  .name = "Motor_control",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Shoot */
osThreadId_t ShootHandle;
const osThreadAttr_t Shoot_attributes = {
  .name = "Shoot",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Referee */
osThreadId_t RefereeHandle;
const osThreadAttr_t Referee_attributes = {
  .name = "Referee",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Log_and_debug */
osThreadId_t Log_and_debugHandle;
const osThreadAttr_t Log_and_debug_attributes = {
  .name = "Log_and_debug",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Remote_control_Task(void *argument);
void Gimbal_Task(void *argument);
void Chassis_Task(void *argument);
void Motor_control_Task(void *argument);
void Shoot_Task(void *argument);
void Referee_Task(void *argument);
void Log_and_debug_Task(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationIdleHook(void);

/* USER CODE BEGIN 2 */
void vApplicationIdleHook( void )
{
   /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
   to 1 in FreeRTOSConfig.h. It will be called on each iteration of the idle
   task. It is essential that code added to this hook function never attempts
   to block in any way (for example, call xQueueReceive() with a block time
   specified, or call vTaskDelay()). If the application makes use of the
   vTaskDelete() API function (as this demo application does) then it is also
   important that vApplicationIdleHook() is permitted to return to its calling
   function, because it is the responsibility of the idle task to clean up
   memory allocated by the kernel to any task that has since been deleted. */
}
/* USER CODE END 2 */

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

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Remote_control */
  Remote_controlHandle = osThreadNew(Remote_control_Task, NULL, &Remote_control_attributes);

  /* creation of Gimbal */
  GimbalHandle = osThreadNew(Gimbal_Task, NULL, &Gimbal_attributes);

  /* creation of Chassis */
  ChassisHandle = osThreadNew(Chassis_Task, NULL, &Chassis_attributes);

  /* creation of Motor_control */
  Motor_controlHandle = osThreadNew(Motor_control_Task, NULL, &Motor_control_attributes);

  /* creation of Shoot */
  ShootHandle = osThreadNew(Shoot_Task, NULL, &Shoot_attributes);

  /* creation of Referee */
  RefereeHandle = osThreadNew(Referee_Task, NULL, &Referee_attributes);

  /* creation of Log_and_debug */
  Log_and_debugHandle = osThreadNew(Log_and_debug_Task, NULL, &Log_and_debug_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_Remote_control_Task */
/**
  * @brief  Function implementing the Remote_control thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Remote_control_Task */
void Remote_control_Task(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN Remote_control_Task */
  
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
    Remote_Tasks();
    osDelay(1);
  }
  /* USER CODE END Remote_control_Task */
}

/* USER CODE BEGIN Header_Gimbal_Task */
/**
* @brief Function implementing the Gimbal thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Gimbal_Task */
void Gimbal_Task(void *argument)
{
  /* USER CODE BEGIN Gimbal_Task */
  osDelay(500);
  Gimbal_Init();
  osDelay(500);
  /* Infinite loop */
  for(;;)
  {
    
    Gimbal_Tasks();
    osDelay(1);
  }
  /* USER CODE END Gimbal_Task */
}

/* USER CODE BEGIN Header_Chassis_Task */
/**
* @brief Function implementing the Chassis thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Chassis_Task */
void Chassis_Task(void *argument)
{
  /* USER CODE BEGIN Chassis_Task */
  osDelay(500);
#if (USE_CHASSIS_HELM != 0 || USE_CHASSIS_OMNI != 0)
  Chassis_Init();
#endif
  /* Infinite loop */
  for(;;)
  {
#if (USE_CHASSIS_HELM != 0 || USE_CHASSIS_OMNI != 0)
    Chassis_Tasks();
#endif
    Supercup_SendData();
    osDelay(1);
  }
  /* USER CODE END Chassis_Task */
}

/* USER CODE BEGIN Header_Motor_control_Task */
/**
* @brief Function implementing the Motor_control thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Motor_control_Task */
void Motor_control_Task(void *argument)
{
  /* USER CODE BEGIN Motor_control_Task */
  /* Infinite loop */
  for(;;)
  {
  #if (USE_CHASSIS_HELM !=0 )
      if (Global.Control.mode != LOCK){
      DJIMotor_SendCurrent(CAN_20063508_1_4_ID, DJI_CAN_1);
      DJIMotor_SendCurrent(CAN_20063508_1_4_ID, DJI_CAN_3); 
      DJIMotor_SendCurrent(CAN_6020_1_4_ID, DJI_CAN_1);
      DJIMotor_SendCurrent(CAN_6020_1_4_ID, DJI_CAN_3);
      }
  #endif
  #if(USE_CHASSIS_OMNI !=0)
      if (Global.Control.mode != LOCK){
      DJIMotor_SendCurrent(CAN_20063508_1_4_ID, DJI_CAN_1);
      }
  #endif
  #if (USE_SHOOT != 0)
      DJIMotor_SendCurrent(CAN_20063508_1_4_ID, DJI_CAN_3);
  #endif
  #if (USE_GIMBAL != 0)
      DMMotor_SendCtrl(DM_CAN_3_2);
  #endif
    if (Global.Chassis.input.reset != 1)
      HAL_IWDG_Refresh(&hiwdg1);
    // Fdcanx_SendData(&hfdcan1,0x200,can_data,8);
    osDelay(1);
  }
  /* USER CODE END Motor_control_Task */
}

/* USER CODE BEGIN Header_Shoot_Task */
/**
* @brief Function implementing the Shoot thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Shoot_Task */
void Shoot_Task(void *argument)
{
  /* USER CODE BEGIN Shoot_Task */
  osDelay(500);
  Shoot_Init();
  /* Infinite loop */
  for(;;)
  {
    
  if (Global.Auto.mode != NONE && Global.Auto.input.Auto_control_online > 0 && Global.Auto.input.control_mode != 0)
  Auto_Control();
  Shoot_Tasks();
  osDelay(1);
 
  }
  /* USER CODE END Shoot_Task */
}

/* USER CODE BEGIN Header_Referee_Task */
/**
* @brief Function implementing the Referee thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Referee_Task */
void Referee_Task(void *argument)
{
  /* USER CODE BEGIN Referee_Task */
  Refree_system_init();
  // �ȴ�����ϵͳ���ߣ���ȡ��ȷ��robot_id
  while(Referee_data.robot_id == 0) {
    Referee_unpack_fifo_data(&referee_fifo, &referee_unpack_obj);
    osDelay(100);
  }
  ui_self_id = Referee_data.robot_id;
  ui_init_helm();
  osDelay(200); // �ȴ�����ϵͳ����ADD֡
  /* Infinite loop */
  int reinit_cnt = 0;
  for(;;)
  {
    Referee_unpack_fifo_data(&referee_fifo, &referee_unpack_obj);
    ui_self_id = Referee_data.robot_id;
    Supercapui_change(cap.remain_vol);
    Shootui_change();
    #if (USE_CHASSIS_HELM != 0)
    Chassisui_change(chassis.relative_angle);
    #elif (USE_CHASSIS_OMNI != 0)
    Chassisui_change(Chassis.chassis_yaw_angle);
    #else
    Chassisui_change(0);
    #endif
    Autoui_change();
    // ÿ10�����·���һ��ADD����ֹͼ�ζ�ʧ
    reinit_cnt++;
    if (reinit_cnt >= 100) {
      reinit_cnt = 0;
      ui_init_helm();
      osDelay(100);
    }
    ui_updata();
    osDelay(100);
  }
  /* USER CODE END Referee_Task */
}

/* USER CODE BEGIN Header_Log_and_debug_Task */
/**
* @brief Function implementing the Log_and_debug thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Log_and_debug_Task */
void Log_and_debug_Task(void *argument)
{
  /* USER CODE BEGIN Log_and_debug_Task */
  LED_ShowColor(RED);
  osDelay(500);
  LED_ShowColor(BLUE);
  osDelay(500);
  LED_ShowColor(GREEN);
  /* Infinite loop */
  for(;;)
  {

  }
  /* USER CODE END Log_and_debug_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

