#include "mission_control.h"
#include "config.h"
#include <stdlib.h>

static uint8_t first_input = 0;
static int fDEFGX = DEFGX;
static int fDEFGY = DEFGY;
static int fDEFGZ = DEFGZ;

void ReactToCommand(uint8_t command, uint8_t *flight_status, mission_struct mission)
{
  switch (command)
  {
  case RequestTelemetry:

    break;
  case AvionicsReboot:

    break;
  case GPSRestart:
    HAL_GPIO_WritePin(GPS_RST_GPIO_Port, GPS_RST_Pin, GPIO_PIN_RESET);
    // osDelay(1);
    HAL_GPIO_WritePin(GPS_RST_GPIO_Port, GPS_RST_Pin, GPIO_PIN_SET);
    break;
  case Ra02LoRaRestart:
    HAL_GPIO_WritePin(LORA_RST_GPIO_Port, LORA_RST_Pin, GPIO_PIN_SET);
    // osDelay(1);
    HAL_GPIO_WritePin(LORA_RST_GPIO_Port, LORA_RST_Pin, GPIO_PIN_RESET);
    break;
  case Buzzer10sTest:
    // HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
    // osDelay(10000); // TODO: This will hang mission control, to be changed
    // HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
    break;
  case EnterPowerSaveMode:

    break;
  case ControlSurfaceLock:

    break;
  case ControlSurfaceUnlock:

    break;
  case DeployParachute:
    *flight_status = APOGEE;
    break;
  case DumpSavedTelemetry:

    break;
  case OverridePhasetoPrelaunch:
    *flight_status = LAUNCH;
    break;
  case OverridePhasetoCountdown:
    *flight_status = LAUNCH;
    break;
  case OverridePhasetoFlight:
    *flight_status = ASCENT;
    break;
  case OverridePhasetoRecovery:
    *flight_status = DESCENT;
    break;
  default:
    break;
  }
}

uint8_t StateMachine(uint8_t flight_status, mission_struct mission)
{

  float dt = 0.1f;
  float gx = mission.gx * dt / 14.375f;
  float gy = mission.gy * dt / 14.375f;
  float gz = mission.gz * dt / 14.375f;

  if (first_input == 0)
  {
    first_input++;
    fDEFGX = gx;
    fDEFGY = gy;
    fDEFGZ = gz;
  }

  switch (flight_status)
  {
  case IDLE:

    break;
  case LAUNCH:
    flight_status = ASCENT;
    break;
  case ASCENT:
    if ((abs(gx - fDEFGX) > 30) || (abs(gy - fDEFGY) > 30) || (abs(gz - fDEFGZ) > 30))
    {
      flight_status = APOGEE;
    }
    break;
  case APOGEE:
    HAL_GPIO_WritePin(PYRO_1_GPIO_Port, PYRO_1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(PYRO_2_GPIO_Port, PYRO_2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
    osDelay(500);
    HAL_GPIO_WritePin(PYRO_1_GPIO_Port, PYRO_1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PYRO_2_GPIO_Port, PYRO_2_Pin, GPIO_PIN_RESET);
    flight_status = DESCENT;
    break;
  case DESCENT:

    break;
  case LANDING:

    break;
  case BMP_INIT_ERROR:

    break;
  default:
    break;
  }
  return flight_status;
}

void MissionToString(mission_struct mission, uint8_t *buf)
{
  snprintf(buf, 256,
           "Pressure: %.2f hPa, Temperature: %.2f C, Humidity: %.2f %%\r\n"
           "Latitude: %s, Longitude: %s\r\n"
           "Acceleration: (%d, %d, %d)\r\n"
           "Gyroscope: (%d, %d, %d)\r\n"
           "Magnetometer: (%d, %d, %d)\r\n"
           "NewCmdArrived: %d, SendData: %d, LoraCmd: %u\r\n",
           mission.pressure,
           mission.temperature,
           mission.humidity,
           mission.latitude,
           mission.longitude,
           mission.ax,
           mission.ay,
           mission.az,
           mission.gx,
           mission.gy,
           mission.gz,
           mission.mx,
           mission.my,
           mission.mz,
           mission.new_cmd_arrived,
           mission.send_data,
           mission.lora_cmd);
}