/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
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
#include "usart.h"
#include "stdio.h"
#include "mb.h"
#include "mbutils.h"
#include <string.h>
#include "CmdHelper.h"
#include "ADS1220.h"
#include "flash.h"
#include "SoftIIC.h"
#include "tim.h"
#include "iwdg.h"
#include "MCP4725.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define SLAVE_ADDRESS   16

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
osThreadId myTask02Handle;
osThreadId productTaskHandle;
static volatile uint8_t product_drivers_ready;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void ModbusTask(void const * argument);
void ProductTestTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of myTask02 */
  osThreadDef(myTask02, ModbusTask, osPriorityIdle, 0, 256);
  myTask02Handle = osThreadCreate(osThread(myTask02), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  osThreadDef(productTask, ProductTestTask, osPriorityNormal, 0, 256);
  productTaskHandle = osThreadCreate(osThread(productTask), NULL);
  if (!defaultTaskHandle || !myTask02Handle || !productTaskHandle) Error_Handler();
  /* add threads, ... */
    
  /* definition and creation of myTask02 */
  
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
  uint8_t i=0;
  /* Infinite loop */
  for(;;)
  {      
    //HAL_IWDG_Refresh(&hiwdg);
    if(i ==1 )HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_SET);
    else if(i ==6)
		{
      i = 0;
			HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET);
    }
    i++;
    osDelay(500);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_ModbusTask */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ModbusTask */
void ModbusTask(void const * argument)
{
  /* USER CODE BEGIN ModbusTask */
  /* Infinite loop */
	HAL_TIM_Base_Start(&htim1);
	HAL_TIM_Base_Start(&htim2);
	ST_Flash_Read(FLASH_USER_START_ADDR, sizeof(ADS1220), &ADS1220);
	ADS1220_Init(&ADS1220);
	MCP4725_Init();
	eMBInit(MB_RTU, SLAVE_ADDRESS, SLAVE_ADDRESS, 115200, MB_PAR_NONE);
	eMBEnable();
    product_drivers_ready = 1;
	/* Infinite loop */
	for (;;) {
		eMBPoll();
		osDelay(1);
	}
  /* USER CODE END ModbusTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void ProductTestTask(void const * argument)
{
    (void)argument;
    for (;;) {
        if (product_drivers_ready) RunPendingProductTest();
        osDelay(1);
    }
}


/* USER CODE END Application */

