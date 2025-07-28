#include "bmp280.h"
#include <math.h>
#include <string.h>

extern UART_HandleTypeDef huart2;

// Internal calibration storage
static BMP280_CalibParams calib_data;
static int32_t t_fine;

//--------------------------
// Read calibration data
//--------------------------
static void BMP280_ReadCalibration(I2C_HandleTypeDef *hi2c) {
	uint8_t calib_raw[24]; // 3 temp + 9 pressure ->  12 * 2 = 24 bytes
	// Read the raw data
	HAL_I2C_Mem_Read(hi2c, BMP280_I2C_ADDR, BMP280_REG_CALIB_START, 1, calib_raw, 24, HAL_MAX_DELAY);

	calib_data.dig_T1 = (uint16_t)(calib_raw[1] << 8 | calib_raw[0]);
	calib_data.dig_T2 = (int16_t)(calib_raw[3] << 8  | calib_raw[2]);
	calib_data.dig_T3 = (int16_t)(calib_raw[5] << 8  | calib_raw[4]);

	calib_data.dig_P1 = (uint16_t)(calib_raw[7] << 8 | calib_raw[6]);
	calib_data.dig_P2 = (int16_t)(calib_raw[9] << 8  | calib_raw[8]);
	calib_data.dig_P3 = (int16_t)(calib_raw[11] << 8 | calib_raw[10]);
	calib_data.dig_P4 = (int16_t)(calib_raw[13] << 8 | calib_raw[12]);
	calib_data.dig_P5 = (int16_t)(calib_raw[15] << 8 | calib_raw[14]);
	calib_data.dig_P6 = (int16_t)(calib_raw[17] << 8 | calib_raw[16]);
	calib_data.dig_P7 = (int16_t)(calib_raw[19] << 8 | calib_raw[18]);
	calib_data.dig_P8 = (int16_t)(calib_raw[21] << 8 | calib_raw[20]);
	calib_data.dig_P9 = (int16_t)(calib_raw[23] << 8 | calib_raw[22]);

}

//----------------------------
// BMP280 Initialization
//----------------------------
HAL_StatusTypeDef BMP280_Init(I2C_HandleTypeDef *hi2c) {
	uint8_t reset_cmd = 0xB6;  //reset command
	HAL_StatusTypeDef ret;

	ret = HAL_I2C_Mem_Write(hi2c, BMP280_I2C_ADDR, BMP280_REG_RESET, 1, &reset_cmd, 1, HAL_MAX_DELAY);



	// check if the reset write was successful
	if (ret != HAL_OK)
		return ret;

	HAL_Delay(100);
	// read calibration data
	BMP280_ReadCalibration(hi2c);

	uint8_t ctrl_meas = (1 << 5) | (1 << 2) | 0x03;
	// Set measurement control settings
	ret = HAL_I2C_Mem_Write(hi2c, BMP280_I2C_ADDR, BMP280_REG_CTRL_MEAS, 1, &ctrl_meas, 1, HAL_MAX_DELAY);

	return ret;
}

//-------------------------------
// Read and compensate data
//------------------------------
HAL_StatusTypeDef BMP280_Read(I2C_HandleTypeDef * hi2c, BMP280_Data *data) {
	uint8_t raw[6];
	if (HAL_I2C_Mem_Read(hi2c, BMP280_I2C_ADDR, BMP280_REG_PRESS_MSB, 1, raw, 6, HAL_MAX_DELAY) != HAL_OK)
	{
		return HAL_ERROR;
	}

	int32_t adc_P = ((int32_t)(raw[0] << 12) | (raw[1] << 4) | (raw[2] >> 4));
	int32_t adc_T = ((int32_t)(raw[3] << 12) | (raw[4] << 4) | (raw[5] >> 4));

	//Temperature Compensation
	int32_t var1, var2;
    var1 = ((((adc_T >> 3) - ((int32_t)calib_data.dig_T1 << 1))) * ((int32_t)calib_data.dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)calib_data.dig_T1)) *
              ((adc_T >> 4) - ((int32_t)calib_data.dig_T1))) >> 12) *
              ((int32_t)calib_data.dig_T3)) >> 14;
    t_fine = var1 + var2;
    data->temperature = ((t_fine * 5 + 128) >> 8) / 100.0f;

    //Pressure Compensation
    int64_t var1p, var2p, p;
    var1p = ((int64_t)t_fine) -128000;
    var2p = var1p * var1p * (int64_t)calib_data.dig_P6;
    var2p = var2p + ((var1p * (int64_t)calib_data.dig_P5) << 17);
	var2p = var2p + (((int64_t)calib_data.dig_P4) << 35);
	var1p = ((var1p * var1p * (int64_t)calib_data.dig_P3) >> 8) + ((var1p * (int64_t)calib_data.dig_P2) << 12);
	var1p = (((((int64_t)1) << 47) + var1p)) * ((int64_t)calib_data.dig_P1) >> 33;

	if (var1p == 0) {
		data->pressure = 0.0f;
		return HAL_ERROR;
	}

    p = 1048576 - adc_P;
    p = (((p << 31) - var2p) * 3125) / var1p;
    var1p = (((int64_t)calib_data.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2p = (((int64_t)calib_data.dig_P8) * p) >> 19;

    p = ((p + var1p + var2p) >> 8) + (((int64_t)calib_data.dig_P7) << 4);
    data->pressure = (float)p / 25600.0f;  // in hPa

    return HAL_OK;

}
