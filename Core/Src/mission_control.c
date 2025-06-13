#include "mission_control.h"
#include "config.h"

uint8_t ReactToCommand(uint8_t command, uint8_t flight_status){
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
  return flight_status;
}

uint8_t StateMachine(uint8_t flight_status){

  switch (flight_status)
    {
    case IDLE:
      
      break;
    case LAUNCH:
      
      break;
    case ASCENT:
      
      break;
    case APOGEE:
      
      break;
    case DESCENT:
      
      break;
    case LANDING:
      
      break;
    case MODULE_INIT_ERROR:
    
      break;
    default:
      break;
    }
  return flight_status;
}