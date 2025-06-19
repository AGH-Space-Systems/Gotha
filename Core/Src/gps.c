#include "gps.h"
#include <string.h>
#include <stdio.h>

static UART_HandleTypeDef *gps_huart;
static char gps_rx_char;
static char gps_buffer[GPS_BUFFER_SIZE];
static uint16_t gps_index = 0;
static uint8_t gps_ready = 0;

GPS_Data gps_data;

void GPS_Init(UART_HandleTypeDef *huart) {
    gps_huart = huart;
    HAL_UART_Receive_IT(gps_huart, (uint8_t*)&gps_rx_char, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == gps_huart->Instance) {
        if (gps_index < GPS_BUFFER_SIZE - 1) {
            gps_buffer[gps_index++] = gps_rx_char;
            if (gps_rx_char == '\n') {
                gps_buffer[gps_index] = '\0';
                gps_index = 0;
                gps_ready = 1;
            }
        } else {
            gps_index = 0;
        }

        HAL_UART_Receive_IT(gps_huart, (uint8_t*)&gps_rx_char, 1);
    }
}

void GPS_Process(void) {
    if (!gps_ready) return;

    gps_ready = 0;

    if (strstr(gps_buffer, "$GPGGA") != NULL) {
        strcpy(gps_data.raw, gps_buffer);
        GPS_ParseGGA(gps_data.raw);
    }
}

void GPS_ParseGGA(const char *nmea) {
    char copy[GPS_BUFFER_SIZE];
    strncpy(copy, nmea, GPS_BUFFER_SIZE);
    char *token;
    char *rest = copy;
    int field = 0;

    while ((token = strtok_r(rest, ",", &rest))) {
        field++;
        if (field == 3) {
            strncpy(gps_data.latitude, token, sizeof(gps_data.latitude));
        } else if (field == 5) {
            strncpy(gps_data.longitude, token, sizeof(gps_data.longitude));
            break;
        }
    }
}
