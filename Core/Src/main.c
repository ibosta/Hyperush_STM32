/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "lwip.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "lwip/udp.h"
#include "lwip/ip_addr.h"
#include "udp_echoserver.h"
#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_gpio.h"
#include "mlx90614.h"
#include "bno055.h"
#include "ir_sensor.h"
#include "encoder.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// BNO055 sensor instance
bno055_t bno055_sensor;

// IR sensor instances
ir_sensor_t ir_sensor;
ir_sensor_t ir_sensor2;  // İkinci IR sensör için yapı
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define LD1_Pin GPIO_PIN_0
#define LD2_Pin GPIO_PIN_7
#define LD3_Pin GPIO_PIN_14
#define LD1_GPIO_Port GPIOB
#define LD2_GPIO_Port GPIOB
#define LD3_GPIO_Port GPIOB

/* IR Sensor pin tanımlamaları */
#define IR_SENSOR_Pin GPIO_PIN_5
#define IR_SENSOR_GPIO_Port GPIOF

/* Akım sensörü tanımlamaları */
#define ADC_PIN GPIO_PIN_4        // PA4 pin ADC1_IN4
#define ADC_PORT GPIOA
#define VCC 3.31f                  // Referans gerilim STM32
#define SIFIR_NOKTA 2.5f         // Sıfır akımda çıkış voltajı
#define HASSASIYET 0.04f          // Sensör hassasiyeti (V/A)

// Filtreleme parametreleri
#define SAMPLE_COUNT 50           // Ortalama için örnek sayısı
#define ALPHA 0.1f               // Low-pass filter katsayısı (0.1 = %10 yeni, %90 eski)


// İKİNCİ IR SENSOR
#define IR_SENSOR2_Pin GPIO_PIN_1
#define IR_SENSOR2_GPIO_Port GPIOG

// ENCODER Z pini  Tanımlaması
#define ENCODER_PIN GPIO_PIN_2
#define ENCODER_PIN_GPIO_Port GPIOF

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
extern ADC_HandleTypeDef hadc1;
uint32_t counter = 0;
extern uint8_t current_speed;  // udp_echoserver.c'deki global değişkeni dışa aktarıyoruz
struct udp_pcb *udp_socket;
ip_addr_t dest_ip;

// MLX90614 Temperature Sensor Variables (volatile for Live Expressions)
volatile float ambient_temp_C = 0.0f;
volatile float object_temp_C = 0.0f;
volatile uint8_t mlx_connection_status = 0;
volatile uint32_t ambient_error_count = 0;
volatile uint32_t object_error_count = 0;

// BNO055 Accelerometer Variables (volatile for Live Expressions)
volatile float accel_x = 0.0f;
volatile float accel_y = 0.0f;
volatile float accel_z = 0.0f;

// BNO055 Gyroscope Variables (volatile for Live Expressions)
volatile float gyro_x = 0.0f;
volatile float gyro_y = 0.0f;
volatile float gyro_z = 0.0f;

// Akım Sensörü Değişkenleri 
volatile float voltage = 0;
volatile float current = 0;
volatile float filtered_voltage = 0;
volatile float filtered_current = 0;

// Encoder Değişkenleri (encoder.c'den extern)
extern volatile int32_t encoder_position;
extern volatile int32_t encoder_velocity;  
extern volatile float encoder_speed;
extern volatile float encoder_total_distance;

// IR Sensör Variables (volatile for Live Expressions)
volatile uint8_t ir_sensor_state = 0;
volatile uint8_t last_ir_sensor_state = 1;
volatile uint32_t ir_object_count = 0;

// İKİNCİ IR SENSÖR - PF4 PİNİ (YENİ!)
volatile uint8_t ir_sensor2_state = 0;
volatile uint8_t last_ir_sensor2_state = 1;
volatile uint32_t ir_object2_count = 0;

// Additional global variables for sensor operations
MLX90614_t mlx90614_sensor_instance;
char uart_buffer[512];

// Motor kontrol pinleri
GPIO_TypeDef *INPUT1_PORT = GPIOG;
GPIO_TypeDef *INPUT2_PORT = GPIOD;
uint16_t INPUT1_PIN = GPIO_PIN_3;  // PG3
uint16_t INPUT2_PIN = GPIO_PIN_10; // PD10
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM10_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// ESC kalibrasyon ve hız kontrolü için değişkenler
static const uint32_t ESC_MIN_PULSE = 1050;     // Minimum pulse (Futaba standard)
static const uint32_t ESC_MAX_PULSE = 1940;     // Maximum pulse (Futaba standard)
static const uint32_t ESC_START_PULSE = 1050;   // Başlangıç pulse değeri
static uint8_t esc_calibrated = 0;              // Kalibrasyon durumu

// ESC başlangıç kalibrasyonu
void CalibrateESC(void) {
    // Önce minimum sinyal gönder ve bekle
    __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, ESC_MIN_PULSE);
    HAL_Delay(2000);  // 2 saniye bekle
    
    // Başlangıç sinyalini gönder
    __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, ESC_START_PULSE);
    HAL_Delay(2000);  // 2 saniye bekle
    
    esc_calibrated = 1;  // Kalibrasyon tamamlandı
}
void IncreaseMotorSpeed(void) {
    if (current_speed < 100) {
        current_speed += 1;  // %0.5'lik artış
        SetMotorSpeed(current_speed);
    }
}

void DecreaseMotorSpeed(void) {
    if (current_speed > 0) {
        current_speed  = 0;  // direkt sıfırlama
        SetMotorSpeed(current_speed);
    }
}

// ESC hız kontrolü için fonksiyon (0-100 yüzde)
void SetMotorSpeed(uint8_t speed_percent) {
    if (!esc_calibrated) return;
    
    if (speed_percent > 100) speed_percent = 100;
    
    uint32_t pulse;
    if (speed_percent == 0) {
        pulse = ESC_MIN_PULSE;
    } else {
        pulse = ESC_MIN_PULSE + (((uint32_t)(ESC_MAX_PULSE - ESC_MIN_PULSE) * speed_percent) / 100);
    }
    
    // Timer değerlerine ölçekle
    uint32_t timer_value = (pulse * (htim10.Init.Period + 1)) / 20000;
    __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, timer_value);
}

uint32_t ADC_Read_Average(void)
{
    uint32_t total = 0;
    uint32_t valid_samples = 0;

    for(int i = 0; i < SAMPLE_COUNT; i++)
    {
        HAL_ADC_Start(&hadc1);
        if(HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK)
        {
            uint32_t val = HAL_ADC_GetValue(&hadc1);
            total += val;
            valid_samples++;
        }
        HAL_ADC_Stop(&hadc1);
        HAL_Delay(1);  // Örnekler arası kısa bekleme
    }

    if(valid_samples > 0)
        return total / valid_samples;
    else
        return 0;
}

// Low-pass filtre fonksiyonu
float lowPassFilter(float newValue, float oldValue)
{
    return (ALPHA * newValue) + ((1.0f - ALPHA) * oldValue);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_LWIP_Init();
  MX_TIM10_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
    // MLX90614 sensor instance initialization
  mlx90614_init(&mlx90614_sensor_instance, &hi2c1, 0xB4);

  // IR sensor'ü başlat
  ir_sensor_init(&ir_sensor, IR_SENSOR_GPIO_Port, IR_SENSOR_Pin);

    // İKİNCİ IR SENSOR'Ü BAŞLAT - PF4 PİNİ (YENİ!)
  ir_sensor_init(&ir_sensor2, IR_SENSOR2_GPIO_Port, IR_SENSOR2_Pin);

    // BNO055 initialization (I2C2) - using existing library
  if(bno055_init(&bno055_sensor, &hi2c2, BNO055_ADDRESS_A)) {
      bno055_setOperationMode(&bno055_sensor, OPERATION_MODE_ACCONLY);
      HAL_Delay(100);
      sprintf(uart_buffer, "BNO055 initialized successfully.\r\n");
  } else {
      sprintf(uart_buffer, "BNO055 initialization failed.\r\n");
  }
  HAL_UART_Transmit(&huart3,(uint8_t*)uart_buffer,strlen(uart_buffer),10);

    // Encoder initialization (TIM3) - NOT TOUCHING
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_GPIO_WritePin(ENCODER_PIN_GPIO_Port, ENCODER_PIN, GPIO_PIN_SET);
  
  // Başlangıçta her iki pini HIGH yaparak aracı durdur
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, GPIO_PIN_SET);    // PG3 HIGH
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);   // PD10 HIGH
  
  // PWM çıkışını başlat ve ESC için güvenli başlangıç değerini ayarla
  HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
  __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, ESC_MIN_PULSE); // Minimum pulse değeri
  
  // ESC kalibrasyonunu yap
  CalibrateESC();
  
  // Initialize UDP echo server
  __udp_echoserver_init_oo();
  
  // Setup the UDP socket
  udp_socket = udp_new();
  if (udp_socket == NULL)
  {
    printf("Error creating UDP socket\n");
    return 1;
  }

  // Set packet headers informations
  IP4_ADDR(&dest_ip, 192, 168, 1, 255);
  uint16_t dest_port = 8080;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // Process incoming packet and let LwIP stack handle it
    MX_LWIP_Process();
    //Levitasyon motorunun hızını istediğimiz zaman durdurmamızı sağlayan fonksiyon 
    ProcessLevitation();
    // Ortalama ADC okuması
    uint32_t adc_val = ADC_Read_Average();

    // Ham voltaj hesaplama
    voltage = ((float)adc_val * VCC) / 4095.0f;
    current = (SIFIR_NOKTA - voltage) / HASSASIYET;

    // Low-pass filtre uygula
    filtered_voltage = lowPassFilter(voltage, filtered_voltage);
    filtered_current = lowPassFilter(current, filtered_current);
// MLX90614 Temperature readings - Güvenli okuma
    float ambient=0.0f, object=0.0f;
    if(mlx90614_getAmbient(&mlx90614_sensor_instance, &ambient)) {
        ambient_temp_C = ambient;
        mlx_connection_status = 1;
    } else { 
        ambient_error_count++; 
        mlx_connection_status = 0;
        ambient_temp_C = 25.0f; // Varsayılan sıcaklık
    }
    if(mlx90614_getObject1(&mlx90614_sensor_instance, &object)) {
        object_temp_C = object;
    } else { 
        object_error_count++;
        object_temp_C = 30.0f; // Varsayılan nesne sıcaklığı
    }
    
        // BNO055 Accelerometer readings using existing library
    bno055_vector_t accel_vector;
    if(bno055_getVector(&bno055_sensor, VECTOR_ACCELEROMETER, &accel_vector)) {
        accel_x = accel_vector.x;
        accel_y = accel_vector.y;
        accel_z = accel_vector.z;
    }
    
    // BNO055 Gyroscope readings using existing library
    bno055_vector_t gyro_vector;
    if(bno055_getVector(&bno055_sensor, VECTOR_GYROSCOPE, &gyro_vector)) {
        gyro_x = gyro_vector.x;
        gyro_y = gyro_vector.y;
        gyro_z = gyro_vector.z;
    }

        // Encoder readings (external function)
    Encoder_Update();

        // IR Sensor readings using existing library
    ir_sensor_update(&ir_sensor);
    ir_sensor_state = ir_sensor_get_state(&ir_sensor);
    if(last_ir_sensor_state != ir_sensor_state){
        if(ir_sensor_state == 0) ir_object_count++;
    }
    last_ir_sensor_state = ir_sensor_state;
    
    // İKİNCİ IR SENSOR OKUMA - PF4 PİNİ (YENİ!)
    ir_sensor_update(&ir_sensor2);
    ir_sensor2_state = ir_sensor_get_state(&ir_sensor2);
    if(last_ir_sensor2_state != ir_sensor2_state){
        if(ir_sensor2_state == 0) ir_object2_count++;
    }
    last_ir_sensor2_state = ir_sensor2_state;

    // Sensör verilerini integer'a çevir
    int temp_int = (int)(ambient_temp_C * 100);
    int obj_temp_int = (int)(object_temp_C * 100);
    int ax_int = (int)(accel_x * 100);
    int ay_int = (int)(accel_y * 100);
    int az_int = (int)(accel_z * 100);
    int gx_int = (int)(gyro_x * 100);
    int gy_int = (int)(gyro_y * 100);
    int gz_int = (int)(gyro_z * 100);
    
    // Encoder verilerini metreye çevir (cm -> m)
    float total_distance_m = get_total_distance() / 100.0f;  // cm'den metreye çevir
    long enc_pos_int = (long)(total_distance_m * 1000);  // Metreyi milimetreye çevir ve long olarak sakla
    long enc_spd_int = (long)(encoder_speed * 1000);    // Hızı da aynı şekilde

    //Akım Sensörü verilerini integer'a çevir

    int current_int = (int)(filtered_current * 100);

    // Sıcaklık ve IMU verilerini 0-9999 aralığında sınırla
    temp_int = temp_int < 0 ? 0 : (temp_int > 9999 ? 9999 : temp_int);
    obj_temp_int = obj_temp_int < 0 ? 0 : (obj_temp_int > 9999 ? 9999 : obj_temp_int);
    ax_int = ax_int < 0 ? 0 : (ax_int > 9999 ? 9999 : ax_int);
    ay_int = ay_int < 0 ? 0 : (ay_int > 9999 ? 9999 : ay_int);
    az_int = az_int < 0 ? 0 : (az_int > 9999 ? 9999 : az_int);
    gx_int = gx_int < 0 ? 0 : (gx_int > 9999 ? 9999 : gx_int);
    gy_int = gy_int < 0 ? 0 : (gy_int > 9999 ? 9999 : gy_int);
    gz_int = gz_int < 0 ? 0 : (gz_int > 9999 ? 9999 : gz_int);


    // JSON formatında birleştir
    char message[1024];  // Buffer boyutunu artırdık
    snprintf(message, sizeof(message), 
        "{\"temperature\":%d,\"object_temp\":%d,\"accel_x\":%d,\"accel_y\":%d,\"accel_z\":%d,"
        "\"gyro_x\":%d,\"gyro_y\":%d,\"gyro_z\":%d,\"total_distance\":%ld,\"encoder_speed\":%ld,"
        "\"ir_state\":%d,\"ir_state2\":%d,\"current\":%d}",
        temp_int, obj_temp_int, ax_int, ay_int, az_int, gx_int, gy_int, gz_int,
        enc_pos_int, enc_spd_int, ir_object_count, ir_object2_count,current_int);

    // Set the structure to handle the packet
    struct pbuf *p;
    p = pbuf_alloc(PBUF_TRANSPORT, strlen(message) + 1, PBUF_RAM);
    if (p == NULL)
    {
        printf("Error allocating pbuf\n");
        break;
    }

    // Send the JSON message over UDP
    memcpy(p->payload, message, strlen(message) + 1);
    err_t err = udp_sendto(udp_socket, p, &dest_ip, dest_port);
    if (err != ERR_OK)
    {
        printf("Error while sending UDP packet\n");
    }

    pbuf_free(p);

    // Update the counter value (if needed for additional features)
    counter++;
    if (counter == 10)
    {
        counter = 0;
    }

    // Wait for 15 ms
    HAL_Delay(15);

  }

  // This point shall not be reached
  udp_remove(udp_socket);
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x20404768;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x20404768;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x20404768;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 216-1;     // APB2 timer clock = 216MHz
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 20000-1;      // 20ms period for servo (50Hz)
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1000;            // Başlangıç: Motor kapalı
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */
  HAL_TIM_MspPostInit(&htim10);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pins : PF2 PF5 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PG1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : PD10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PG3 PG5 PG6 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* Configure GPIO pins */
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  /* Configure PG3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* Configure PD10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* Set initial pin states to HIGH */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
