#include "imu.h"

#define ADXL345_ADDR    0x53 << 1
#define ITG3200_ADDR    0x68 << 1
#define HMC5883L_ADDR   0x1E << 1

uint8_t ok = 1;
 
void reset_I2C(GY85 *imu){
GPIO_InitTypeDef GPIO_InitStruct = {0};
HAL_I2C_DeInit(imu->hi2c);
GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
GPIO_InitStruct.Pin = GPIO_PIN_6;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
 
HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
 
for (int i = 0; i < 10; i++) {
HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
HAL_Delay(20);
HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
HAL_Delay(20);
}
MX_I2C1_Init();
}

uint8_t GY85_Init(GY85 *imu, I2C_HandleTypeDef *hi2c) {
    imu->hi2c = hi2c;
    reset_I2C(imu);
    uint8_t data;

    data = 0x08;

    uint8_t id = 0;
    if (HAL_I2C_Mem_Read(hi2c, ADXL345_ADDR, 0x00, 1, &id, 1, 100) == HAL_OK && id == 0xE5) {
       uint8_t adxl345_found = 1;
    }
    if (HAL_I2C_Mem_Write(hi2c, ADXL345_ADDR, 0x2D, 1, &data, 1, 100) != HAL_OK) return 0;
    uint8_t check;
    HAL_I2C_Mem_Write(hi2c, ADXL345_ADDR, 0x00, 1, &check,1, 100);

    // gyro
    data = 0x1E;  // 1E = 100Hz LPF, 188Hz band
    if (HAL_I2C_Mem_Write(hi2c, ITG3200_ADDR, 0x15, 1, &data, 1, 100) != HAL_OK) return 0;

    data = 0x00;  
    if (HAL_I2C_Mem_Write(hi2c, ITG3200_ADDR, 0x3E, 1, &data, 1, 100) != HAL_OK) return 0;

    // Magnet: 15Hz
    data = 0x70; 
    HAL_I2C_Mem_Write(hi2c, HMC5883L_ADDR, 0x00, 1, &data, 1, 100);
    data = 0xA0;  
    HAL_I2C_Mem_Write(hi2c, HMC5883L_ADDR, 0x01, 1, &data, 1, 100);
    data = 0x00; 
    HAL_I2C_Mem_Write(hi2c, HMC5883L_ADDR, 0x02, 1, &data, 1, 100);

    return 1;
}

//ADXL345 acc
void GY85_ReadAccel(GY85 *imu) {
    uint8_t buf[6];
    HAL_I2C_Mem_Read(imu->hi2c, ADXL345_ADDR, 0x32, 1, buf, 6, 100);

    for (int i = 0; i < 3; i++) {
        imu->accel[i] = (int16_t)((buf[2*i+1] << 8) | buf[2*i]);
    }
}

//ITG3200 gyro
void GY85_ReadGyro(GY85 *imu) {
    uint8_t buf[6];
    HAL_I2C_Mem_Read(imu->hi2c, ITG3200_ADDR, 0x1D, 1, buf, 6, 100);

    for (int i = 0; i < 3; i++) {
        imu->gyro[i] = (int16_t)((buf[2*i] << 8) | buf[2*i+1]);
    }
}

//HMC5883L magneto
void GY85_ReadMag(GY85 *imu) {
    uint8_t buf[6];
    HAL_I2C_Mem_Read(imu->hi2c, HMC5883L_ADDR, 0x03, 1, buf, 6, 100);

    imu->mag[0] = (int16_t)((buf[0] << 8) | buf[1]);
    imu->mag[2] = (int16_t)((buf[2] << 8) | buf[3]); 
    imu->mag[1] = (int16_t)((buf[4] << 8) | buf[5]);
}

void I2C_ResetBus(I2C_HandleTypeDef *hi2c) {
    __HAL_RCC_I2C1_FORCE_RESET();
    HAL_Delay(1);
    __HAL_RCC_I2C1_RELEASE_RESET();
    HAL_I2C_DeInit(hi2c);
    HAL_I2C_Init(hi2c);
}

