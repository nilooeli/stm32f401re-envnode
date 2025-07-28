#ifndef BMP280_H
#define BMP280_H

#include "stm32f4xx_hal.h"
#include <stdio.h>

// BMP280 default address
#define BMP280_I2C_ADDR        (0x76 << 1)
#define BMP280_REG_ID          0xD0
#define BMP280_REG_RESET       0xE0
#define BMP280_REG_STATUS      0xF3
#define BMP280_REG_CTRL_MEAS   0xF4
#define BMP280_REG_CONFIG      0xF5
#define BMP280_REG_PRESS_MSB   0xF7
// Calibration register base address
#define BMP280_REG_CALIB_START 0x88

// Calibration parameters structure
typedef struct {
	uint16_t dig_T1;
	int16_t  dig_T2;
	int16_t  dig_T3;
	uint16_t  dig_P1;
	int16_t  dig_P2;
	int16_t  dig_P3;
	int16_t  dig_P4;
	int16_t  dig_P5;
	int16_t  dig_P6;
	int16_t  dig_P7;
	int16_t  dig_P8;
	int16_t  dig_P9;
} BMP280_CalibParams;

// Data structure for output
typedef struct {
	float temperature;  // degrees in C
	float pressure;     // degrees in hPa
} BMP280_Data;

//Function Prototypes
HAL_StatusTypeDef BMP280_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef BMP280_Read(I2C_HandleTypeDef * hi2c, BMP280_Data *data);

#endif //BMP280_H



