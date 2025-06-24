#ifndef GY85_H
#define GY85_H

#include "stm32f4xx_hal.h"
#include "i2c.h"

typedef struct {
    I2C_HandleTypeDef *hi2c;
    int16_t accel[3];
    int16_t gyro[3];
    int16_t mag[3];
} GY85;

uint8_t GY85_Init(GY85 *imu, I2C_HandleTypeDef *hi2c);
void GY85_ReadAccel(GY85 *imu);
void GY85_ReadGyro(GY85 *imu);
void GY85_ReadMag(GY85 *imu);
void I2C_ResetBus(I2C_HandleTypeDef *hi2c);

#endif
