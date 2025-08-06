/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : mlx90614.h
  * @brief          : MLX90614 Sensor Header without dependency on i2c.h
  ******************************************************************************
  */
#ifndef _MLX90614_H
#define _MLX90614_H
#include <stdbool.h>
#include "stm32f7xx_hal.h"  // Direct HAL include instead of i2c.h

typedef enum {
  MLX90614_UNIT_RAW,
  MLX90614_UNIT_K,
  MLX90614_UNIT_C,
  MLX90614_UNIT_F
} MLX90614_UNIT_t;

typedef struct {
  uint8_t IIR : 3;
  uint8_t RepeatSensorTestON: 1;
  uint8_t SelectObjAmb: 2;
  uint8_t DualIRSensor: 1;
  uint8_t NegativeSignOfKs: 1;
  uint8_t FIR: 3;
  uint8_t GAIN: 3;
  uint8_t NegativeSignOfKt2: 1;
  uint8_t DisableSensorTest: 1;
} MLX90614_CONFIG_REG_t;

typedef struct {
  MLX90614_UNIT_t unit;
  int16_t rawEmissivity, rawAmbient, rawObject1, rawObject2, rawMax, rawMin;
  float emissivity, ambient, object1, object2, max, min;
  uint16_t id[4];
  MLX90614_CONFIG_REG_t configReg;
  I2C_HandleTypeDef* hi2c;   // Pointer to HAL I2C handle assigned at runtime
  uint16_t device_address;   // I2C device address assigned at runtime
} MLX90614_t;

bool mlx90614_init(MLX90614_t* sensor, I2C_HandleTypeDef* hi2c, uint16_t devAddress);
void mlx90614_setUnit(MLX90614_t* sensor, MLX90614_UNIT_t MLX90614_UNIT_);
bool mlx90614_readID(MLX90614_t* sensor, int16_t *id);
bool mlx90614_getEmissivity(MLX90614_t* sensor, float *emissivity);
bool mlx90614_setEmissivity(MLX90614_t* sensor, float emissivity);
bool mlx90614_setMax(MLX90614_t* sensor, float maxTemp);
bool mlx90614_setMin(MLX90614_t* sensor, float minTemp);
bool mlx90614_getMax(MLX90614_t* sensor, float *maxTemp);
bool mlx90614_getMin(MLX90614_t* sensor, float *minTemp);
bool mlx90614_getAmbient(MLX90614_t* sensor, float *ambientTemp);
bool mlx90614_getObject1(MLX90614_t* sensor, float *objectTemp);
bool mlx90614_getObject2(MLX90614_t* sensor, float *objectTemp);

#endif
