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
#include "LoRa.h"
#include "bmp280.h"
#include "config.h"
#include "spi.h"
#include "i2c.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
  struct bmp_data{
    float pressure;
    float temperature;
    float humidity;
  };
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
static uint8_t flight_status = IDLE;
/* USER CODE END Variables */
/* Definitions for AccGyroTask */
osThreadId_t AccGyroTaskHandle;
const osThreadAttr_t AccGyroTask_attributes = {
  .name = "AccGyroTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for PressTask */
osThreadId_t PressTaskHandle;
const osThreadAttr_t PressTask_attributes = {
  .name = "PressTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for GPSTask */
osThreadId_t GPSTaskHandle;
const osThreadAttr_t GPSTask_attributes = {
  .name = "GPSTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for ServoTask */
osThreadId_t ServoTaskHandle;
const osThreadAttr_t ServoTask_attributes = {
  .name = "ServoTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for LoRaTask */
osThreadId_t LoRaTaskHandle;
const osThreadAttr_t LoRaTask_attributes = {
  .name = "LoRaTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for MissionControlT */
osThreadId_t MissionControlTHandle;
const osThreadAttr_t MissionControlT_attributes = {
  .name = "MissionControlT",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for AccGyroQueue */
osMessageQueueId_t AccGyroQueueHandle;
const osMessageQueueAttr_t AccGyroQueue_attributes = {
  .name = "AccGyroQueue"
};
/* Definitions for PressQueue */
osMessageQueueId_t PressQueueHandle;
const osMessageQueueAttr_t PressQueue_attributes = {
  .name = "PressQueue"
};
/* Definitions for GPSQueue */
osMessageQueueId_t GPSQueueHandle;
const osMessageQueueAttr_t GPSQueue_attributes = {
  .name = "GPSQueue"
};
/* Definitions for ServoQueue */
osMessageQueueId_t ServoQueueHandle;
const osMessageQueueAttr_t ServoQueue_attributes = {
  .name = "ServoQueue"
};
/* Definitions for LoRaRXQueue */
osMessageQueueId_t LoRaRXQueueHandle;
const osMessageQueueAttr_t LoRaRXQueue_attributes = {
  .name = "LoRaRXQueue"
};
/* Definitions for LoRaTXQueue */
osMessageQueueId_t LoRaTXQueueHandle;
const osMessageQueueAttr_t LoRaTXQueue_attributes = {
  .name = "LoRaTXQueue"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
/* USER CODE BEGIN Application */
void ReactToCommand(uint8_t command){
  switch (command)
  {
  case RequestTelemetry:
    
    break;
  case AvionicsReboot:
    
    break;
  case GPSRestart:
    HAL_GPIO_WritePin(GPS_RST_GPIO_Port, GPS_RST_Pin, GPIO_PIN_RESET);
    osDelay(1);
    HAL_GPIO_WritePin(GPS_RST_GPIO_Port, GPS_RST_Pin, GPIO_PIN_SET);
    break;
  case Ra02LoRaRestart:
    HAL_GPIO_WritePin(LORA_RST_GPIO_Port, LORA_RST_Pin, GPIO_PIN_SET);
    osDelay(1);
    HAL_GPIO_WritePin(LORA_RST_GPIO_Port, LORA_RST_Pin, GPIO_PIN_RESET);
    break;
  case Buzzer10sTest:
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
    osDelay(10000);
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
    break;
  case EnterPowerSaveMode:
    
    break;
  case ControlSurfaceLock:
    
    break;
  case ControlSurfaceUnlock:
    
    break;
  case DeployParachute:
    flight_status=APOGEE;
    break;
  case DumpSavedTelemetry:
    
    break;
  case OverridePhasetoPrelaunch:
    flight_status=IDLE;
    break;
  case OverridePhasetoCountdown:
    
    break;
  case OverridePhasetoFlight:
    flight_status=ASCENT;
    break;
  case OverridePhasetoRecovery:
    flight_status=DESCENT;
    break;
  default:
    break;
  }
}
/* USER CODE END FunctionPrototypes */

void AccGyroFunc(void *argument);
void PressFunc(void *argument);
void GPSFunc(void *argument);
void ServoFunc(void *argument);
void LoRaFunc(void *argument);
void MissionControlFunc(void *argument);

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

  /* Create the queue(s) */
  /* creation of AccGyroQueue */
  AccGyroQueueHandle = osMessageQueueNew (32, sizeof(float), &AccGyroQueue_attributes);

  /* creation of PressQueue */
  PressQueueHandle = osMessageQueueNew (32, sizeof(float), &PressQueue_attributes);

  /* creation of GPSQueue */
  GPSQueueHandle = osMessageQueueNew (32, sizeof(float), &GPSQueue_attributes);

  /* creation of ServoQueue */
  ServoQueueHandle = osMessageQueueNew (32, sizeof(float), &ServoQueue_attributes);

  /* creation of LoRaRXQueue */
  LoRaRXQueueHandle = osMessageQueueNew (8, sizeof(uint8_t), &LoRaRXQueue_attributes);

  /* creation of LoRaTXQueue */
  LoRaTXQueueHandle = osMessageQueueNew (8, sizeof(uint8_t), &LoRaTXQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of AccGyroTask */
  AccGyroTaskHandle = osThreadNew(AccGyroFunc, NULL, &AccGyroTask_attributes);

  /* creation of PressTask */
  PressTaskHandle = osThreadNew(PressFunc, NULL, &PressTask_attributes);

  /* creation of GPSTask */
  GPSTaskHandle = osThreadNew(GPSFunc, NULL, &GPSTask_attributes);

  /* creation of ServoTask */
  ServoTaskHandle = osThreadNew(ServoFunc, NULL, &ServoTask_attributes);

  /* creation of LoRaTask */
  LoRaTaskHandle = osThreadNew(LoRaFunc, NULL, &LoRaTask_attributes);

  /* creation of MissionControlT */
  MissionControlTHandle = osThreadNew(MissionControlFunc, NULL, &MissionControlT_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_AccGyroFunc */
/**
  * @brief  Function implementing the AccGyroTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_AccGyroFunc */
void AccGyroFunc(void *argument)
{
  /* USER CODE BEGIN AccGyroFunc */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END AccGyroFunc */
}

/* USER CODE BEGIN Header_PressFunc */
/**
* @brief Function implementing the PressTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_PressFunc */
void PressFunc(void *argument)
{
  /* USER CODE BEGIN PressFunc */
  BMP280_HandleTypedef bmp280;
  bmp280_init_default_params(&bmp280.params);
	bmp280.addr = BMP280_I2C_ADDRESS_0;
	bmp280.i2c = &hi2c1;

  struct bmp_data bmp_data; 

  if(!bmp280_init(&bmp280, &bmp280.params)){
    flight_status=MODULE_INIT_ERROR;
  }
  /* Infinite loop */
  for(;;)
  {
    bmp280_read_float(&bmp280, &bmp_data.temperature, &bmp_data.pressure, &bmp_data.humidity);
    osStatus_t status = osMessageQueuePut(PressQueueHandle, &bmp_data, 0, portMAX_DELAY);
    if(status != osOK){continue;}
    osDelay(1);
  }
  /* USER CODE END PressFunc */
}

/* USER CODE BEGIN Header_GPSFunc */
/**
* @brief Function implementing the GPSTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_GPSFunc */
void GPSFunc(void *argument)
{
  /* USER CODE BEGIN GPSFunc */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END GPSFunc */
}

/* USER CODE BEGIN Header_ServoFunc */
/**
* @brief Function implementing the ServoTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ServoFunc */
void ServoFunc(void *argument)
{
  /* USER CODE BEGIN ServoFunc */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END ServoFunc */
}

/* USER CODE BEGIN Header_LoRaFunc */
/**
* @brief Function implementing the LoRaTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LoRaFunc */
void LoRaFunc(void *argument)
{
  /* USER CODE BEGIN LoRaFunc */
  uint8_t received_data;
  LoRa LoRa_module = newLoRa();
  LoRa_module.hSPIx                 = &hspi1;
	LoRa_module.CS_port               = CS_LORA_GPIO_Port;
	LoRa_module.CS_pin                = CS_LORA_Pin;
	LoRa_module.reset_port            = LORA_RST_GPIO_Port;
	LoRa_module.reset_pin             = LORA_RST_Pin;

  if(LoRa_init(&LoRa_module) != LORA_OK){
    flight_status= MODULE_INIT_ERROR;
  }
  LoRa_startReceiving(&LoRa_module);
  /* Infinite loop */
  for(;;)
  {
    LoRa_receive(&LoRa_module, &received_data, sizeof(received_data));
    osStatus_t status = osMessageQueuePut(LoRaRXQueueHandle, &received_data, 0, portMAX_DELAY);
    if(status != osOK){continue;}
    osDelay(1);
  }
  /* USER CODE END LoRaFunc */
}

/* USER CODE BEGIN Header_MissionControlFunc */
/**
* @brief Function implementing the MissionControlT thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MissionControlFunc */
void MissionControlFunc(void *argument)
{
  /* USER CODE BEGIN MissionControlFunc */
  uint8_t received_GS_data;
  struct bmp_data received_bmp_data; 
  /* Infinite loop */
  for(;;)
  {
    osStatus_t LoRaQueueStatus = osMessageQueueGet(LoRaRXQueueHandle, &received_GS_data, NULL, portMAX_DELAY);
    if (LoRaQueueStatus == osOK) {
        ReactToCommand(received_GS_data);
    }
    osStatus_t status = osMessageQueueGet(PressQueueHandle, &received_bmp_data, NULL, portMAX_DELAY);
    if (status == osOK) {
        //
    }
    osDelay(1);
  }
  /* USER CODE END MissionControlFunc */
}

/* Private application code --------------------------------------------------*/

/* USER CODE END Application */

