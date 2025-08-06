/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : ir_sensor.c
  * @brief          : IR Sensor Implementation
  ******************************************************************************
  * @attention
  * This software component is licensed under terms in the LICENSE file.
  * If no LICENSE file is present, this software is provided AS-IS.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "ir_sensor.h"
#include "main.h"

bool ir_sensor_init(ir_sensor_t* sensor, GPIO_TypeDef* gpio_port, uint16_t gpio_pin) {
    if (sensor == NULL) {
        return false;
    }
    
    sensor->gpio_port = gpio_port;
    sensor->gpio_pin = gpio_pin;
    sensor->state = IR_SENSOR_STATE_NO_OBJECT;
    sensor->previous_state = IR_SENSOR_STATE_NO_OBJECT;
    sensor->last_change_time = 0;
    sensor->detection_count = 0;
    sensor->last_detection_time = 0;
    
    return true;
}

void ir_sensor_update(ir_sensor_t* sensor) {
    if (sensor == NULL) {
        return;
    }
    
    // Read GPIO state (0 = object detected, 1 = no object)
    uint8_t gpio_state = HAL_GPIO_ReadPin(sensor->gpio_port, sensor->gpio_pin);
    
    // Update sensor state
    sensor->previous_state = sensor->state;
    sensor->state = (gpio_state == 0) ? IR_SENSOR_STATE_OBJECT_DETECTED : IR_SENSOR_STATE_NO_OBJECT;
    
    // Count object detections on state change
    if (sensor->previous_state != sensor->state && sensor->state == IR_SENSOR_STATE_OBJECT_DETECTED) {
        sensor->detection_count++;
        sensor->last_detection_time = HAL_GetTick();
    }
    
    sensor->last_change_time = HAL_GetTick();
}

bool ir_sensor_read_digital(ir_sensor_t* sensor) {
    if (sensor == NULL) {
        return false;
    }
    
    return (HAL_GPIO_ReadPin(sensor->gpio_port, sensor->gpio_pin) == 0);
}

ir_sensor_state_t ir_sensor_get_state(ir_sensor_t* sensor) {
    if (sensor == NULL) {
        return IR_SENSOR_STATE_NO_OBJECT;
    }
    
    return sensor->state;
}

uint32_t ir_sensor_get_detection_count(ir_sensor_t* sensor) {
    if (sensor == NULL) {
        return 0;
    }
    
    return sensor->detection_count;
}