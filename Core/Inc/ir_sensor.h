/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : ir_sensor.h
  * @brief          : IR Sensor Header
  ******************************************************************************
  * @attention
  * This software component is licensed under terms in the LICENSE file.
  * If no LICENSE file is present, this software is provided AS-IS.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef IR_SENSOR_H_
#define IR_SENSOR_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* IR Sensor Configuration */
#define IR_SENSOR_DEBOUNCE_TIME_MS      50

/* IR Sensor States */
typedef enum {
    IR_SENSOR_STATE_NO_OBJECT = 0,
    IR_SENSOR_STATE_OBJECT_DETECTED = 1
} ir_sensor_state_t;

/* IR Sensor Structure (Digital GPIO based) */
typedef struct {
    GPIO_TypeDef* gpio_port;
    uint16_t gpio_pin;
    ir_sensor_state_t state;
    ir_sensor_state_t previous_state;
    uint32_t last_change_time;
    uint32_t detection_count;
    uint32_t last_detection_time;
} ir_sensor_t;

/* Function prototypes */
bool ir_sensor_init(ir_sensor_t* sensor, GPIO_TypeDef* gpio_port, uint16_t gpio_pin);
void ir_sensor_update(ir_sensor_t* sensor);
bool ir_sensor_read_digital(ir_sensor_t* sensor);
ir_sensor_state_t ir_sensor_get_state(ir_sensor_t* sensor);
uint32_t ir_sensor_get_detection_count(ir_sensor_t* sensor);
void ir_sensor_reset_detection_count(ir_sensor_t* sensor);
uint32_t ir_sensor_get_last_detection_time(ir_sensor_t* sensor);
bool ir_sensor_is_object_detected(ir_sensor_t* sensor);

#ifdef __cplusplus
}
#endif

#endif /* IR_SENSOR_H_ */
