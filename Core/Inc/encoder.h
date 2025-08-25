/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : encoder.h
  * @brief          : Encoder Header for Distance Measurement
  ******************************************************************************
  * @attention
  * This software component is licensed under terms in the LICENSE file.
  * If no LICENSE file is present, this software is provided AS-IS.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef ENCODER_H_
#define ENCODER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* Encoder Configuration */
#define ENCODER_DISTANCE_PER_PULSE    (65.0f / 3600.0f)  // 100 mm/devir, 3600 pulse/rev

/* Global Variables */
extern volatile int32_t encoder_position;
extern volatile int32_t encoder_last;

/* Function Prototypes */
float get_total_distance(void);  // Returns total distance in centimeters
extern volatile int32_t encoder_velocity;
extern volatile float encoder_speed;
extern volatile float encoder_total_distance;
extern const float encoder_distance_per_pulse;
extern volatile uint32_t last_tick;

/* Function prototypes */
void Encoder_Update(void);
void DistanceLedControl(void);

/* HAL Timer handle - should be defined in main.c */
extern TIM_HandleTypeDef htim3;

#ifdef __cplusplus
}
#endif

#endif /* ENCODER_H_ */
