/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Multi-Sensor System (INA226, MLX90614, BNO055, IR)
  ******************************************************************************
  * @attention
  * This software component is licensed under terms in the LICENSE file.
  * If no LICENSE file is present, this software is provided AS-IS.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "INA226.h"
#include "mlx90614.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// BNO055 Defines
#define BNO055_ADDRESS        (0x28 << 1)
#define BNO055_CHIP_ID_ADDR   0x00
#define BNO055_PAGE_ID_ADDR   0x07
#define BNO055_OPR_MODE_ADDR  0x3D
#define BNO055_PWR_MODE_ADDR  0x3E
#define BNO055_SYS_TRIGGER_ADDR 0x3F
#define BNO055_ACCEL_DATA_X_LSB_ADDR  0x08
#define BNO055_OPERATION_MODE_CONFIG  0x00
#define BNO055_OPERATION_MODE_ACCONLY 0x01
#define BNO055_POWER_MODE_NORMAL       0x00

// LED Defines
#define LD1_Pin GPIO_PIN_0
#define LD2_Pin GPIO_PIN_7
#define LD3_Pin GPIO_PIN_14
#define LD1_GPIO_Port GPIOB
#define LD2_GPIO_Port GPIOB
#define LD3_GPIO_Port GPIOB

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

// AccelData_t struct definition (backup'tan)
typedef struct {
  volatile int16_t x;
  volatile int16_t y; 
  volatile int16_t z;
} AccelData_t;

// INA226 Power Sensor Variables (backup isimlerle)
volatile float ina226_bus_voltage = 0.0f;
volatile float ina226_shunt_voltage = 0.0f;
volatile float ina226_current = 0.0f;
volatile float ina226_power = 0.0f;
volatile float ina226_current_ma = 0.0f;
volatile float ina226_power_mw = 0.0f;
volatile uint8_t ina226_connection_status = 0;
volatile uint32_t ina226_error_count = 0;

// MLX90614 Temperature Sensor Variables (backup isimlerle)
volatile float ambient_temp_C = 0.0f;
volatile float object_temp_C = 0.0f;
volatile uint8_t mlx_connection_status = 0;
volatile uint32_t ambient_error_count = 0;
volatile uint32_t object_error_count = 0;

// BNO055 Accelerometer Variables (backup isimlerle)
volatile AccelData_t accel_data = {0, 0, 0};

// Encoder Değişkenleri (encoder.c'den extern) - DOKUNMUYORUZ
extern volatile int32_t encoder_position;
extern volatile int32_t encoder_velocity;  
extern volatile float encoder_speed;
extern volatile float encoder_total_distance;

// IR Sensör Variables (backup isimlerle)
volatile uint8_t ir_sensor_state = 0;
volatile uint8_t last_ir_sensor_state = 1;
volatile uint32_t ir_object_count = 0;

// Additional global variables from backup
MLX90614_t mlx90614_sensor_instance;
char uart_buffer[512];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

// Sensor Function Prototypes (backup'tan)
void INA226_Test_Simple(void);
void INA226_ReadAllValues_Fixed(void);
void MLX90614_ReadTemps(void);
uint8_t BNO055_Init(void);
uint8_t BNO055_ReadReg(uint8_t reg, uint8_t* value);
uint8_t BNO055_ReadRegs(uint8_t reg, uint8_t* buf, uint8_t len);
uint8_t BNO055_WriteReg(uint8_t reg, uint8_t value);
void BNO055_ReadAccel(void);
void IR_Sensor_Check(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// INA226 Simple Test Function (backup'tan)
void INA226_Test_Simple(void) {
    sprintf(uart_buffer, "I2C2 adres taraması (0x40-0x4F):\r\n");
    HAL_UART_Transmit(&huart2,(uint8_t*)uart_buffer,strlen(uart_buffer),10);
    for (uint8_t addr = 0x40; addr <= 0x4F; addr++) {
        if (HAL_I2C_IsDeviceReady(&hi2c2, addr << 1, 3, 10) == HAL_OK) {
            sprintf(uart_buffer, "  Cihaz bulundu: 0x%02X\r\n", addr);
            HAL_UART_Transmit(&huart2,(uint8_t*)uart_buffer,strlen(uart_buffer),10);
        }
    }
    uint16_t device_id = INA226_ID();
    sprintf(uart_buffer, "INA226 Device ID: 0x%04X (0x2260 bekleniyor)\r\n", device_id);
    HAL_UART_Transmit(&huart2,(uint8_t*)uart_buffer,strlen(uart_buffer),10);
    if(device_id == 0x2260) {
        ina226_connection_status = 1;
        sprintf(uart_buffer, "INA226 bağlantısı BAŞARILI!\r\n");
    } else {
        ina226_connection_status = 0;
        sprintf(uart_buffer, "INA226 bağlantısı BAŞARISIZ!\r\n");
    }
    HAL_UART_Transmit(&huart2,(uint8_t*)uart_buffer,strlen(uart_buffer),10);
}

// INA226 Read All Values Function (backup'tan)
void INA226_ReadAllValues_Fixed(void) {
    if(ina226_connection_status == 1) {
        ina226_bus_voltage = INA226_BusVoltage();
        ina226_shunt_voltage = INA226_ShuntVoltage() * 1000.0f;
        ina226_current = INA226_Current();
        ina226_current_ma = ina226_current * 1000.0f;
        ina226_power = INA226_Power();
        ina226_power_mw = ina226_power * 1000.0f;
        if(ina226_bus_voltage < 0.0f || ina226_bus_voltage > 36.0f){
            ina226_error_count++;
        }
    }
}

// MLX90614 Temperature Reading Function (backup'tan)
void MLX90614_ReadTemps(void) {
    float ambient=0.0f, object=0.0f;
    if(mlx90614_getAmbient(&mlx90614_sensor_instance, &ambient)) {
        ambient_temp_C = ambient;
    } else { 
        ambient_error_count++; 
        mlx_connection_status=0; 
    }
    if(mlx90614_getObject1(&mlx90614_sensor_instance, &object)) {
        object_temp_C = object;
    } else { 
        object_error_count++; 
        mlx_connection_status=0; 
    }
    if(ambient_error_count==0 && object_error_count==0) {
        mlx_connection_status=1;
    }
}

// BNO055 Functions (backup'tan)
uint8_t BNO055_ReadReg(uint8_t reg, uint8_t* value){
    return HAL_I2C_Mem_Read(&hi2c2, BNO055_ADDRESS, reg, 1, value, 1, 10);
}

uint8_t BNO055_WriteReg(uint8_t reg, uint8_t value){
    return HAL_I2C_Mem_Write(&hi2c2, BNO055_ADDRESS, reg, 1, &value, 1, 10);
}

uint8_t BNO055_ReadRegs(uint8_t reg, uint8_t* buf, uint8_t len){
    return HAL_I2C_Mem_Read(&hi2c2, BNO055_ADDRESS, reg, 1, buf, len, 10);
}

uint8_t BNO055_Init(void){
    uint8_t id=0;
    if(BNO055_ReadReg(BNO055_CHIP_ID_ADDR, &id)!=HAL_OK) return 0;
    if(id!=0xA0) return 0;
    if(BNO055_WriteReg(BNO055_PWR_MODE_ADDR, BNO055_POWER_MODE_NORMAL)!=HAL_OK) return 0;
    HAL_Delay(1);
    if(BNO055_WriteReg(BNO055_OPR_MODE_ADDR, BNO055_OPERATION_MODE_CONFIG)!=HAL_OK) return 0;
    HAL_Delay(1);
    if(BNO055_WriteReg(BNO055_PAGE_ID_ADDR,0)!=HAL_OK) return 0;
    if(BNO055_WriteReg(BNO055_OPR_MODE_ADDR, BNO055_OPERATION_MODE_ACCONLY)!=HAL_OK) return 0;
    HAL_Delay(1);
    return 1;
}

void BNO055_ReadAccel(void){
    uint8_t buf[6];
    if(BNO055_ReadRegs(BNO055_ACCEL_DATA_X_LSB_ADDR, buf, 6)==HAL_OK){
        accel_data.x = (int16_t)(buf[1]<<8 | buf[0]);
        accel_data.y = (int16_t)(buf[3]<<8 | buf[2]);
        accel_data.z = (int16_t)(buf[5]<<8 | buf[4]);
    }
}

// IR Sensor Check Function (backup'tan)
void IR_Sensor_Check(void){
    uint8_t state = HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_5);
    ir_sensor_state = state;
    if(last_ir_sensor_state != state){
        if(state == 0) ir_object_count++;
    }
    last_ir_sensor_state = state;
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
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  
  // MLX90614 sensor instance başlatma (backup'tan)
  mlx90614_init(&mlx90614_sensor_instance, &hi2c1, 0xB4);

  sprintf(uart_buffer, "=== Çoklu Sensör Sistemi Başlatılıyor ===\r\n");
  HAL_UART_Transmit(&huart2,(uint8_t*)uart_buffer,strlen(uart_buffer),10);

  // INA226 başlatma ve test (I2C2)
  INA226_INIT();
  INA226_SetCalibration(0.002, 10.0); // 2mOhm shunt, 10A max current
  HAL_Delay(1);
  INA226_Test_Simple();

  // BNO055 başlatma (I2C2)
  if(!BNO055_Init()){
      sprintf(uart_buffer, "BNO055 başlatılamadı!\r\n");
      HAL_UART_Transmit(&huart2,(uint8_t*)uart_buffer,strlen(uart_buffer),10);
  } else {
      sprintf(uart_buffer, "BNO055 başlatıldı.\r\n");
      HAL_UART_Transmit(&huart2,(uint8_t*)uart_buffer,strlen(uart_buffer),10);
  }

  // Encoder başlatma (TIM3 kullanıyoruz) - DOKUNMUYORUZ
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
    // Sensör okuma döngüsü (backup'tan)
    INA226_Test_Simple();
    INA226_ReadAllValues_Fixed();
    MLX90614_ReadTemps();
    BNO055_ReadAccel();
    Encoder_Update();
    IR_Sensor_Check();
    
    // Debug output (backup format)
    sprintf(uart_buffer,
        "INA226: Bus=%.3fV Shunt=%.1fmV I=%.2fmA P=%.2fmW\r\n"
        "MLX90614: Ambient=%.2fC Object=%.2fC\r\n"
        "BNO055 Accel: X=%d Y=%d Z=%d\r\n"
        "Encoder: Pos=%.2fmm Speed=%.2fmm/s\r\n"
        "IR Sensor: State=%d ObjectCount=%lu\r\n\n",
        ina226_bus_voltage, ina226_shunt_voltage, ina226_current_ma, ina226_power_mw,
        ambient_temp_C, object_temp_C,
        accel_data.x, accel_data.y, accel_data.z,
        encoder_total_distance, encoder_speed,
        ir_sensor_state, ir_object_count
    );

    HAL_UART_Transmit(&huart2,(uint8_t*)uart_buffer,strlen(uart_buffer),10);

    HAL_Delay(10); // 10ms döngü gecikmesi
  }
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();

  // LED'ler için GPIOB başlatma (backup'tan)
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD2_Pin|LD3_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = LD1_Pin|LD2_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // IR sensör için GPIOF Pin 5 (backup'tan)
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

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
