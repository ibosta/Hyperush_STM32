/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : encoder.c
  * @brief          : Encoder Implementation for Distance Measurement
  ******************************************************************************
  * @attention
  * This software component is licensed under terms in the LICENSE file.
  * If no LICENSE file is present, this software is provided AS-IS.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "encoder.h"
#include "main.h"

/* Global Variables */
volatile int32_t encoder_position = 0;
volatile int32_t encoder_last = 0;
volatile int32_t encoder_velocity = 0;
volatile float encoder_speed = 0.0f;
volatile float encoder_total_distance = 0.0f;
const float encoder_distance_per_pulse = 50.7f / 3600.0f;  // 100 mm/devir, 3600 pulse/rev
volatile uint32_t last_tick = 0;

/**
  * @brief  Update encoder position and calculate distance
  * @param  None
  * @retval None
  */
void Encoder_Update(void) {
    uint32_t current_tick = HAL_GetTick();
    int32_t count = __HAL_TIM_GET_COUNTER(&htim3);

    encoder_velocity = count - encoder_last;
    encoder_last = count;

    // Timer sayacı taşması kontrolü
    if (encoder_velocity > 32767)
        encoder_velocity -= 65536;
    else if (encoder_velocity < -32767)
        encoder_velocity += 65536;

    // Toplam pozisyona ekle
    encoder_position += encoder_velocity;

    // Toplam mesafe (mm olarak)
    encoder_total_distance = encoder_position * encoder_distance_per_pulse;

    // Anlık hız (mm/s olarak)
    uint32_t delta_time = current_tick - last_tick;
    if (delta_time > 0) {
        encoder_speed = (encoder_velocity * encoder_distance_per_pulse * 1000.0f) / delta_time;
        last_tick = current_tick;
    }
    
    // MESAFE İÇİN LED KONTROLÜ BURADA ÇAĞRILIYOR
    DistanceLedControl();
}

/**
  * @brief  Control LEDs based on distance measurement
  * @param  None
  * @retval None
  */
void DistanceLedControl(void)
{
    static uint8_t tunnel_end_alerted = 0;
    static uint8_t direction = 1; // 1: ileri, 0: geri
    static float prev_distance = 0.0f;

    // Yön tespiti (ileri mi gidiyor geri mi)
    if (encoder_total_distance > prev_distance)
        direction = 1; // ileri
    else if (encoder_total_distance < prev_distance)
        direction = 0; // geri
    prev_distance = encoder_total_distance;

    // Önce tüm ledleri kapatıyoruz
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); // Kırmızı
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);  // Mavi
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);  // Yeşil

    // Tünel sonuna geldiyse, bütün ledleri kısa süreyle yakıp söndür
    if (!tunnel_end_alerted && encoder_total_distance >= 1860.0f) // 186 metre, mm cinsinden
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
        HAL_Delay(1000); // 1 saniye yakıp söndür
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
        tunnel_end_alerted = 1;
    }
    // Normal aralık kontrolü:
    else if (encoder_total_distance < 1860.0f)
    {
        tunnel_end_alerted = 0; // geri dönerken tekrar yanabilmesi için resetle
        if (direction) // ileri gidiyor
        {
            if (encoder_total_distance >= 1200.0f)
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET); // Yeşil 5 m ve üstü
            else if (encoder_total_distance >= 600.0f)
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET); // Mavi 3 m ve üstü
            else if (encoder_total_distance >= 100.0f)
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); // Kırmızı 1 m ve üstü
            // değilse ledler kapalı
        }
        else // geri gidiyor
        {
            if (encoder_total_distance <= 100.0f && encoder_total_distance > 0.0f)
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); // Kırmızı
            else if (encoder_total_distance <= 600.0f && encoder_total_distance > 1000.0f)
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET); // Mavi
            else if (encoder_total_distance <= 1200.0f && encoder_total_distance > 3000.0f)
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET); // Yeşil
            // değilse ledler kapalı
        }
    }
}
