#ifndef __MISSION_CONTROL_H__
#define __MISSION_CONTROL_H__

#include <stdint.h>

#include "main.h"
#include "cmsis_os.h"

uint8_t ReactToCommand(uint8_t command, uint8_t flight_status);
uint8_t StateMachine(uint8_t flight_status);

#endif  // __MISSION_CONTROL_H__