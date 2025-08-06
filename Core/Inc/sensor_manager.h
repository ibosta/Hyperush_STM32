/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : sensor_manager.h
  * @brief          : Multi-Sensor Management System Header
  ******************************************************************************
  * @attention
  * This software component is licensed under terms in the LICENSE file.
  * If no LICENSE file is present, this software is provided AS-IS.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef SENSOR_MANAGER_H_
#define SENSOR_MANAGER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* Include all sensor headers */
#include "INA226.h"
#include "mlx90614.h"
#include "bno055.h"
#include "ir_sensor.h"
#include "encoder.h"

/* Sensor status definitions */
typedef enum {
    SENSOR_STATUS_NOT_INITIALIZED = 0,
    SENSOR_STATUS_READY = 1,
    SENSOR_STATUS_ERROR = 2,
    SENSOR_STATUS_BUSY = 3
} sensor_status_t;

/* Sensor data structure */
typedef struct {
    /* INA226 Power Monitor */
    struct {
        float bus_voltage;
        float shunt_voltage;
        float current;
        float power;
        sensor_status_t status;
    } ina226;
    
    /* MLX90614 IR Temperature */
    struct {
        float ambient_temp;
        float object_temp;
        float emissivity;
        sensor_status_t status;
    } mlx90614;
    
    /* BNO055 IMU */
    struct {
        bno055_vector_t accelerometer;
        bno055_vector_t gyroscope;
        bno055_vector_t magnetometer;
        bno055_vector_t euler;
        bno055_vector_t linear_accel;
        bno055_vector_t gravity;
        bno055_quaternion_t quaternion;
        int8_t temperature;
        uint8_t calibration_status[4]; // sys, gyro, accel, mag
        sensor_status_t status;
    } bno055;
    
    /* IR Sensor */
    struct {
        ir_sensor_state_t state;
        uint16_t raw_value;
        float voltage;
        uint32_t detection_count;
        uint32_t last_detection_time;
        sensor_status_t status;
    } ir_sensor;
    
    /* Encoder */
    struct {
        int32_t position;
        int32_t revolution_count;
        encoder_direction_t direction;
        uint32_t speed_rpm;
        float angle_degrees;
        float angle_radians;
        sensor_status_t status;
    } encoder;
    
    /* System status */
    uint32_t last_update_time;
    uint32_t update_count;
    bool system_ready;
} sensor_data_t;

/* Sensor manager structure */
typedef struct {
    /* Sensor instances */
    INA226_t ina226_sensor;
    MLX90614_t mlx90614_sensor;
    bno055_t bno055_sensor;
    ir_sensor_t ir_sensor_instance;
    encoder_t encoder_instance;
    
    /* Communication handles */
    I2C_HandleTypeDef* hi2c1;
    I2C_HandleTypeDef* hi2c2;
    TIM_HandleTypeDef* htim_encoder;
    ADC_HandleTypeDef* hadc_ir;
    
    /* Data storage */
    sensor_data_t data;
    
    /* Configuration */
    uint32_t update_interval_ms;
    bool auto_update_enabled;
    
    /* Status */
    bool initialized;
} sensor_manager_t;

/* Function prototypes */
bool sensor_manager_init(sensor_manager_t* manager, 
                        I2C_HandleTypeDef* hi2c1, 
                        I2C_HandleTypeDef* hi2c2,
                        TIM_HandleTypeDef* htim_encoder,
                        ADC_HandleTypeDef* hadc_ir);

bool sensor_manager_init_ina226(sensor_manager_t* manager);
bool sensor_manager_init_mlx90614(sensor_manager_t* manager);
bool sensor_manager_init_bno055(sensor_manager_t* manager);
bool sensor_manager_init_ir_sensor(sensor_manager_t* manager, GPIO_TypeDef* gpio_port, uint16_t gpio_pin, uint32_t adc_channel);
bool sensor_manager_init_encoder(sensor_manager_t* manager, uint32_t counts_per_rev);

void sensor_manager_update_all(sensor_manager_t* manager);
void sensor_manager_update_ina226(sensor_manager_t* manager);
void sensor_manager_update_mlx90614(sensor_manager_t* manager);
void sensor_manager_update_bno055(sensor_manager_t* manager);
void sensor_manager_update_ir_sensor(sensor_manager_t* manager);
void sensor_manager_update_encoder(sensor_manager_t* manager);

sensor_data_t* sensor_manager_get_data(sensor_manager_t* manager);
bool sensor_manager_is_ready(sensor_manager_t* manager);
void sensor_manager_set_update_interval(sensor_manager_t* manager, uint32_t interval_ms);
void sensor_manager_enable_auto_update(sensor_manager_t* manager, bool enable);

/* Utility functions */
void sensor_manager_print_status(sensor_manager_t* manager);
void sensor_manager_reset_all_data(sensor_manager_t* manager);
uint32_t sensor_manager_get_update_count(sensor_manager_t* manager);
uint32_t sensor_manager_get_last_update_time(sensor_manager_t* manager);

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_MANAGER_H_ */
