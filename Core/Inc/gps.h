#ifndef __GPS_H__
#define __GPS_H__

#include "stm32f4xx_hal.h" 

#define GPS_BUFFER_SIZE 256

typedef struct {
    char raw[GPS_BUFFER_SIZE];
    char latitude[16];
    char longitude[16];
} GPS_Data;

extern GPS_Data gps_data;

void GPS_Init(UART_HandleTypeDef *huart);
void GPS_Process(void);
void GPS_ParseGGA(const char *nmea);

#endif
