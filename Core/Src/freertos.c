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
#include "main.h"
#include "task.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "LoRa.h"
#include "bmp280.h"
#include "config.h"
#include "i2c.h"
#include "mission_control.h"
#include "spi.h"
#include "gps.h"
#include "imu.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
struct bmp_data
{
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
static uint8_t flight_status = ASCENT;
extern UART_HandleTypeDef huart2;
static mission_struct mission;
osMutexId_t mission_mutex;
const osMutexAttr_t mission_mutex_attr = {
    .name = "missionMutex"};
osMutexId_t i2c_mutex;
const osMutexAttr_t i2c_mutex_attr = {
    .name = "i2cMutex"};

// ESP32 COMMUNICATION BUFFERS
static uint8_t buf;
static uint8_t rec_buf[256];

/* USER CODE END Variables */
/* Definitions for AccGyroTask */
osThreadId_t AccGyroTaskHandle;
const osThreadAttr_t AccGyroTask_attributes = {
    .name = "AccGyroTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for PressTask */
osThreadId_t PressTaskHandle;
const osThreadAttr_t PressTask_attributes = {
    .name = "PressTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for GPSTask */
osThreadId_t GPSTaskHandle;
const osThreadAttr_t GPSTask_attributes = {
    .name = "GPSTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for ServoTask */
osThreadId_t ServoTaskHandle;
const osThreadAttr_t ServoTask_attributes = {
    .name = "ServoTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for LoRaTask */
osThreadId_t LoRaTaskHandle;
const osThreadAttr_t LoRaTask_attributes = {
    .name = "LoRaTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for MissionControlT */
osThreadId_t MissionControlTHandle;
const osThreadAttr_t MissionControlT_attributes = {
    .name = "MissionControlT",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for AccGyroQueue */
osMessageQueueId_t AccGyroQueueHandle;
const osMessageQueueAttr_t AccGyroQueue_attributes = {
    .name = "AccGyroQueue"};
/* Definitions for PressQueue */
osMessageQueueId_t PressQueueHandle;
const osMessageQueueAttr_t PressQueue_attributes = {
    .name = "PressQueue"};
/* Definitions for GPSQueue */
osMessageQueueId_t GPSQueueHandle;
const osMessageQueueAttr_t GPSQueue_attributes = {
    .name = "GPSQueue"};
/* Definitions for ServoQueue */
osMessageQueueId_t ServoQueueHandle;
const osMessageQueueAttr_t ServoQueue_attributes = {
    .name = "ServoQueue"};
/* Definitions for LoRaRXQueue */
osMessageQueueId_t LoRaRXQueueHandle;
const osMessageQueueAttr_t LoRaRXQueue_attributes = {
    .name = "LoRaRXQueue"};
/* Definitions for LoRaTXQueue */
osMessageQueueId_t LoRaTXQueueHandle;
const osMessageQueueAttr_t LoRaTXQueue_attributes = {
    .name = "LoRaTXQueue"};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
/* USER CODE BEGIN Application */

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
void MX_FREERTOS_Init(void)
{
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  i2c_mutex = osMutexNew(&i2c_mutex_attr);
  mission_mutex = osMutexNew(&mission_mutex_attr);
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of AccGyroQueue */
  AccGyroQueueHandle =
      osMessageQueueNew(32, sizeof(float), &AccGyroQueue_attributes);

  /* creation of PressQueue */
  PressQueueHandle =
      osMessageQueueNew(32, sizeof(float), &PressQueue_attributes);

  /* creation of GPSQueue */
  GPSQueueHandle = osMessageQueueNew(32, sizeof(float), &GPSQueue_attributes);

  /* creation of ServoQueue */
  ServoQueueHandle =
      osMessageQueueNew(32, sizeof(float), &ServoQueue_attributes);

  /* creation of LoRaRXQueue */
  LoRaRXQueueHandle =
      osMessageQueueNew(8, sizeof(uint8_t), &LoRaRXQueue_attributes);

  /* creation of LoRaTXQueue */
  LoRaTXQueueHandle =
      osMessageQueueNew(8, sizeof(uint8_t), &LoRaTXQueue_attributes);

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
  MissionControlTHandle =
      osThreadNew(MissionControlFunc, NULL, &MissionControlT_attributes);

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

  GY85 imu;
  osMutexAcquire(i2c_mutex, osWaitForever);
  I2C_ResetBus(&hi2c1);
  if (hi2c1.State != HAL_I2C_STATE_READY)
  {
    I2C_ResetBus(&hi2c1);
  }
  osMutexRelease(i2c_mutex);
#ifdef USE_IMU
  osMutexAcquire(i2c_mutex, osWaitForever);
  if (!GY85_Init(&imu, &hi2c1))
  {
    flight_status = GY85_INIT_ERROR;
  }
  osMutexRelease(i2c_mutex);
#endif
  /* Infinite loop */
  for (;;)
  {
#ifdef USE_IMU
    osMutexAcquire(i2c_mutex, osWaitForever);
    GY85_ReadAccel(&imu);
    GY85_ReadGyro(&imu);
    GY85_ReadMag(&imu);
    osMutexRelease(i2c_mutex);
    osMutexAcquire(mission_mutex, osWaitForever);
    mission.ax = imu.accel[0];
    mission.ay = imu.accel[1];
    mission.az = imu.accel[2];

    mission.gx = imu.gyro[0];
    mission.gy = imu.gyro[1];
    mission.gz = imu.gyro[2];

    mission.mx = imu.mag[0];
    mission.my = imu.mag[1];
    mission.mz = imu.mag[2];
    osMutexRelease(mission_mutex);
#endif
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

#ifdef USE_BMP280
  osMutexAcquire(i2c_mutex, osWaitForever);
  if (!bmp280_init(&bmp280, &bmp280.params))
  {
    flight_status = BMP_INIT_ERROR;
  }
  osMutexRelease(i2c_mutex);
#endif
  /* Infinite loop */
  for (;;)
  {
#ifdef USE_BMP280
    osMutexAcquire(i2c_mutex, osWaitForever);
    bmp280_read_float(&bmp280, &bmp_data.temperature, &bmp_data.pressure,
                      &bmp_data.humidity);
    osMutexRelease(i2c_mutex);
    osMutexAcquire(mission_mutex, osWaitForever);
    mission.temperature = bmp_data.temperature;
    mission.pressure = bmp_data.pressure;
    mission.humidity = bmp_data.humidity;
    osMutexRelease(mission_mutex);
#endif
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
  char latitude[16];
  char longitude[16];
#ifdef USE_GPS
  GPS_Init(&huart2);
#endif
  /* Infinite loop */
  for (;;)
  {
#ifdef USE_GPS
    GPS_Process();
    strcpy(latitude, gps_data.latitude);
    strcpy(longitude, gps_data.longitude);
    osMutexAcquire(mission_mutex, osWaitForever);
    strcpy(latitude, mission.latitude);
    strcpy(longitude, mission.longitude);
    osMutexRelease(mission_mutex);
#endif
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
  for (;;)
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
  LoRa lora;
  lora.hspi = &hspi1;
  lora.nss_port = CS_LORA_GPIO_Port;
  lora.nss_pin = CS_LORA_Pin;
  lora.reset_port = LORA_RST_GPIO_Port;
  lora.reset_pin = LORA_RST_Pin;
  // uint8_t buf = 69; // Moved as a static variable
  // uint8_t rec_buf[256]; // Moved as a static variable
#ifdef USE_LORA
  LoRa_Init(&lora);
  LoRa_ReceiveContinuous(&lora);
#endif

#ifdef USE_ESP
  HAL_UART_Receive_IT(&huart1, &buf, 1);
#endif
  /* Infinite loop */
  for (;;)
  {
#ifdef USE_LORA
    osMutexAcquire(mission_mutex, osWaitForever);
    if (mission.send_data)
    {
      mission.send_data = false;
      memcpy(rec_buf, &mission, sizeof(mission));
      LoRa_SendPacket(&lora, rec_buf, sizeof(mission));
      osMutexRelease(mission_mutex);
      LoRa_ReceiveContinuous(&lora);
    }
    else
    {
      int len = LoRa_ReceivePacket(&lora, &buf, sizeof(buf));
      if (buf != 69)
      {
        mission.lora_cmd = buf;
        buf = 69;
        mission.new_cmd_arrived = true;
      }
      else
      {
        mission.lora_cmd = buf;
        mission.new_cmd_arrived = false;
      }
      osMutexRelease(mission_mutex);
    }
#endif

#ifdef USE_ESP
    osMutexAcquire(mission_mutex, osWaitForever);
    if (mission.send_data)
    {
      mission.send_data = false;
      MissionToString(mission, rec_buf);
      HAL_UART_Transmit_IT(&huart1, (uint8_t *)rec_buf, strlen(rec_buf));
    }
    else
    {
      if (buf != 69)
      {
        mission.lora_cmd = buf;
        // buf = 69;
        mission.new_cmd_arrived = true;
      }
      else
      {
        mission.lora_cmd = buf;
        mission.new_cmd_arrived = false;
      }
      osMutexRelease(mission_mutex);
    }
#endif
  }
  osDelay(1);
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

  /* Infinite loop */
  for (;;)
  {
    osMutexAcquire(mission_mutex, osWaitForever);
    if (mission.new_cmd_arrived)
    {
      ReactToCommand(mission.lora_cmd, &flight_status, mission);
      mission.new_cmd_arrived = false;
    }
    flight_status = StateMachine(flight_status, mission);
    osMutexRelease(mission_mutex);
    osDelay(1);
  }
  /* USER CODE END MissionControlFunc */
}

/* Private application code --------------------------------------------------*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    HAL_UART_Receive_IT(&huart1, &buf, 1);
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    osMutexRelease(mission_mutex);
  }
}
/* USER CODE END Application */