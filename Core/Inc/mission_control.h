#ifndef __MISSION_CONTROL_H__
#define __MISSION_CONTROL_H__

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "main.h"
#include "cmsis_os.h"

typedef struct
{
    float pressure;
    float temperature;
    float humidity;
    bool new_cmd_arrived;
    bool send_data;
    uint8_t lora_cmd;
    char latitude[16];
    char longitude[16];
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t mx, my, mz;
} mission_struct;

void ReactToCommand(uint8_t command, uint8_t *flight_status, mission_struct mission);
uint8_t StateMachine(uint8_t flight_status, mission_struct mission);
void MissionToString(mission_struct mission, uint8_t *buf);

#endif // __MISSION_CONTROL_H__