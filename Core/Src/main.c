/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Multi-Sensor System (INA226, MLX90614, BNO055, Encoder, IR)
  ******************************************************************************
  * @attention
  *
  * This software component is licensed under terms in the LICENSE file.
  * If no LICENSE file is present, this software is provided AS-IS.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include "INA226.h"

/* Private defines -----------------------------------------------------------*/
#define LD1_Pin GPIO_PIN_0
#define LD2_Pin GPIO_PIN_7
#define LD3_Pin GPIO_PIN_14
#define LD1_GPIO_Port GPIOB
#define LD2_GPIO_Port GPIOB
#define LD3_GPIO_Port GPIOB

#define MLX90614_ADDR (0x5A << 1)
#define MLX90614_REG_TA 0x06
#define MLX90614_REG_TOBJ1 0x07

#define BNO055_ADDRESS (0x28 << 1)
#define BNO055_CHIP_ID_ADDR 0x00
#define BNO055_PAGE_ID_ADDR 0x07
#define BNO055_OPR_MODE_ADDR 0x3D
#define BNO055_PWR_MODE_ADDR 0x3E
#define BNO055_SYS_TRIGGER_ADDR 0x3F
#define BNO055_ACCEL_DATA_X_LSB_ADDR 0x08
#define BNO055_OPERATION_MODE_CONFIG 0x00
#define BNO055_OPERATION_MODE_ACCONLY 0x01
#define BNO055_POWER_MODE_NORMAL 0x00

#define ENCODER_PPR 3600
#define WHEEL_DIAMETER_MM 6.0f
#define WHEEL_CIRCUMFERENCE_MM (3.14159265359f * WHEEL_DIAMETER_MM)
#define MM_PER_PULSE (WHEEL_CIRCUMFERENCE_MM / ENCODER_PPR)

/* Define a struct for accelerometer data */
typedef struct {
  volatile int16_t x;
  volatile int16_t y;
  volatile int16_t z;
} AccelData_t;

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart2;

volatile float ina226_bus_voltage = 0.0f;
volatile float ina226_shunt_voltage = 0.0f;
volatile float ina226_current = 0.0f;
volatile float ina226_power = 0.0f;
volatile float ina226_current_ma = 0.0f;
volatile float ina226_power_mw = 0.0f;
volatile uint8_t ina226_connection_status = 0;
volatile uint32_t ina226_error_count = 0;

volatile float ambient_temp_C = 0.0f;
volatile float object_temp_C = 0.0f;
volatile uint8_t mlx_connection_status = 0;
volatile uint32_t ambient_error_count = 0;
volatile uint32_t object_error_count = 0;

volatile AccelData_t accel_data = {0, 0, 0};

volatile uint32_t encoder_raw_count = 0;
volatile int32_t encoder_relative_count = 0;
volatile float encoder_distance_mm = 0.0f;
volatile float encoder_distance_cm = 0.0f;
volatile float encoder_rotations = 0.0f;
volatile int32_t encoder_direction = 0;

volatile uint8_t ir_sensor_state = 0;
volatile uint8_t last_ir_sensor_state = 1;
volatile uint32_t ir_object_count = 0;

char uart_buffer[400];

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);

void INA226_Test_Simple(void);
void INA226_Test_Raw(void);
void INA226_ReadAllValues_Fixed(void);

uint8_t MLX90614_ReadTempReg(uint8_t reg, float* temperature);
void MLX90614_ReadTemps(void);

uint8_t BNO055_Init(void);
uint8_t BNO055_ReadReg(uint8_t reg, uint8_t* value);
uint8_t BNO055_ReadRegs(uint8_t reg, uint8_t* buf, uint8_t len);
uint8_t BNO055_WriteReg(uint8_t reg, uint8_t value);
void BNO055_ReadAccel(void);

void Encoder_Update(void);
void IR_Sensor_Check(void);

/* USER CODE BEGIN 0 */

void INA226_Test_Simple(void) {
    sprintf(uart_buffer, "I2C2 adres taraması (0x40-0x4F):\r\n");
    HAL_UART_Transmit(&huart2,(uint8_t*)uart_buffer,strlen(uart_buffer),1000);

    for (uint8_t addr = 0x40; addr <= 0x4F; addr++) {
        if (HAL_I2C_IsDeviceReady(&hi2c2, addr << 1, 3, 1000) == HAL_OK) {
            sprintf(uart_buffer, "  Cihaz bulundu: 0x%02X\r\n", addr);
            HAL_UART_Transmit(&huart2,(uint8_t*)uart_buffer,strlen(uart_buffer),1000);
        }
    }

    uint16_t device_id = INA226_ID();
    sprintf(uart_buffer, "INA226 Device ID: 0x%04X (0x2260 bekleniyor)\r\n", device_id);
    HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);

    if(device_id == 0x2260) {
        ina226_connection_status = 1;
        sprintf(uart_buffer, "INA226 bağlantısı BAŞARILI!\r\n");
    } else {
        ina226_connection_status = 0;
        sprintf(uart_buffer, "INA226 bağlantısı BAŞARISIZ!\r\n");
    }
    HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);
}

void INA226_Test_Raw(void){
    if (!ina226_connection_status) return;

    uint8_t data[2];
    if(HAL_I2C_Mem_Read(&hi2c2, 0x40 << 1, 0x02, 1, data, 2, 10) == HAL_OK){
        uint16_t raw = (data[0] << 8) | data[1];
        float voltage = raw * 1.25e-3f;
        sprintf(uart_buffer, "Ham veri: 0x%04X, Voltaj: %.3fV\r\n", raw, voltage);
        HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);
    } else {
        sprintf(uart_buffer, "Ham veri okuma hatası!\r\n");
        HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);
    }
}

void INA226_ReadAllValues_Fixed(void){
    if (!ina226_connection_status) return;

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

uint8_t MLX90614_ReadTempReg(uint8_t reg, float* temperature) {
    uint8_t buf[3];
    if(HAL_I2C_Mem_Read(&hi2c1, MLX90614_ADDR, reg, 1, buf, 3, 10) != HAL_OK) {
        return 0;
    }
    uint16_t temp_raw = buf[0] | (buf[1] << 8);
    *temperature = (float)temp_raw * 0.02f - 273.15f;
    return 1;
}

void MLX90614_ReadTemps(void) {
    if(!MLX90614_ReadTempReg(MLX90614_REG_TA, &ambient_temp_C)) {
        ambient_error_count++;
        mlx_connection_status = 0;
    } else if(!MLX90614_ReadTempReg(MLX90614_REG_TOBJ1, &object_temp_C)) {
        object_error_count++;
        mlx_connection_status = 0;
    } else {
        mlx_connection_status = 1;
    }
}

uint8_t BNO055_ReadReg(uint8_t reg, uint8_t* value) {
    return HAL_I2C_Mem_Read(&hi2c2, BNO055_ADDRESS, reg, 1, value, 1, 10);
}

uint8_t BNO055_WriteReg(uint8_t reg, uint8_t value) {
    return HAL_I2C_Mem_Write(&hi2c2, BNO055_ADDRESS, reg, 1, &value, 1, 10);
}

uint8_t BNO055_ReadRegs(uint8_t reg, uint8_t* buf, uint8_t len) {
    return HAL_I2C_Mem_Read(&hi2c2, BNO055_ADDRESS, reg, 1, buf, len, 10);
}

uint8_t BNO055_Init(void) {
    uint8_t id = 0;
    if(BNO055_ReadReg(BNO055_CHIP_ID_ADDR, &id) != HAL_OK) return 0;
    if(id != 0xA0) return 0;
    if(BNO055_WriteReg(BNO055_PWR_MODE_ADDR, BNO055_POWER_MODE_NORMAL) != HAL_OK) return 0;
    HAL_Delay(5);
    if(BNO055_WriteReg(BNO055_OPR_MODE_ADDR, BNO055_OPERATION_MODE_CONFIG) != HAL_OK) return 0;
    HAL_Delay(10);
    if(BNO055_WriteReg(BNO055_PAGE_ID_ADDR, 0) != HAL_OK) return 0;
    if(BNO055_WriteReg(BNO055_OPR_MODE_ADDR, BNO055_OPERATION_MODE_ACCONLY) != HAL_OK) return 0;
    HAL_Delay(10);
    return 1;
}

void BNO055_ReadAccel(void) {
    uint8_t buf[6];
    if(BNO055_ReadRegs(BNO055_ACCEL_DATA_X_LSB_ADDR, buf, 6) == HAL_OK) {
        accel_data.x = (int16_t)(buf[1] << 8 | buf[0]);
        accel_data.y = (int16_t)(buf[3] << 8 | buf[2]);
        accel_data.z = (int16_t)(buf[5] << 8 | buf[4]);
    }
}

void Encoder_Update(void) {
    uint32_t count = __HAL_TIM_GET_COUNTER(&htim2);
    int32_t diff = (int32_t)(count - encoder_raw_count);
    encoder_raw_count = count;
    encoder_relative_count += diff;
    encoder_distance_mm = encoder_relative_count * MM_PER_PULSE;
    encoder_distance_cm = encoder_distance_mm / 10.0f;
    encoder_rotations = (float)encoder_relative_count / ENCODER_PPR;
    encoder_direction = (diff > 0) ? 1 : (diff < 0) ? -1 : 0;
}

void IR_Sensor_Check(void) {
    uint8_t state = HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_5);
    ir_sensor_state = state;
    if(last_ir_sensor_state != state) {
        if(state == 0) { // Aktif düşük sensör varsayımı
            ir_object_count++;
        }
    }
    last_ir_sensor_state = state;
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_I2C2_Init();
    MX_TIM2_Init();
    MX_USART2_UART_Init();

    sprintf(uart_buffer, "=== Çoklu Sensör Sistemi Başlatılıyor ===\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);

    INA226_INIT();
    HAL_Delay(10);
    INA226_Test_Simple();

    if(!BNO055_Init()){
        sprintf(uart_buffer, "BNO055 başlatılamadı!\r\n");
        HAL_UART_Transmit(&huart2,(uint8_t*)uart_buffer,strlen(uart_buffer),1000);
    } else {
        sprintf(uart_buffer, "BNO055 başlatıldı.\r\n");
        HAL_UART_Transmit(&huart2,(uint8_t*)uart_buffer,strlen(uart_buffer),1000);
    }

    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);

    while(1){
        INA226_Test_Raw();
        INA226_ReadAllValues_Fixed();
        MLX90614_ReadTemps();
        BNO055_ReadAccel();
        Encoder_Update();
        IR_Sensor_Check();

        sprintf(uart_buffer,
            "INA226: Bus=%.3fV Shunt=%.1fmV I=%.2fmA P=%.2fmW\r\n"
            "MLX90614: Ambient=%.2fC Object=%.2fC\r\n"
            "BNO055 Accel: X=%d Y=%d Z=%d\r\n"
            "Encoder: %.2fmm (%.2fcm) Rot: %.2f Dir: %ld\r\n"
            "IR Sensor: State=%d ObjectCount=%lu\r\n\n",
            ina226_bus_voltage, ina226_shunt_voltage, ina226_current_ma, ina226_power_mw,
            ambient_temp_C, object_temp_C,
            accel_data.x, accel_data.y, accel_data.z,
            encoder_distance_mm, encoder_distance_cm, encoder_rotations, encoder_direction,
            ir_sensor_state, ir_object_count
        );

        HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);

        if (ina226_bus_voltage > 3.0f) {
            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
        } else {
            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
        }

        HAL_Delay(10);
    }
}

/* System Clock Configuration */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 4;
    RCC_OscInitStruct.PLL.PLLN = 200;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;

    if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }
    if(HAL_PWREx_EnableOverDrive() != HAL_OK) { Error_Handler(); }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK){ Error_Handler(); }
}

/* Initialization functions */
static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD2_Pin|LD3_Pin, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = LD1_Pin|LD2_Pin|LD3_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
}

static void MX_I2C1_Init(void) {
    hi2c1.Instance = I2C1;
    hi2c1.Init.Timing = 0x00A0A3F7;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if(HAL_I2C_Init(&hi2c1) != HAL_OK) { Error_Handler(); }
    if(HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK) { Error_Handler(); }
    if(HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) { Error_Handler(); }
}

static void MX_I2C2_Init(void){
    hi2c2.Instance = I2C2;
    hi2c2.Init.Timing = 0x00A0A3F7;
    hi2c2.Init.OwnAddress1 = 0;
    hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.OwnAddress2 = 0;
    hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if(HAL_I2C_Init(&hi2c2) != HAL_OK) { Error_Handler(); }
    if(HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK) { Error_Handler(); }
    if(HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK) { Error_Handler(); }
}

static void MX_TIM2_Init(void) {
    TIM_Encoder_InitTypeDef sConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 4294967295;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
    sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC1Filter = 0;
    sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC2Filter = 0;

    if(HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK) { Error_Handler(); }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

    if(HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) { Error_Handler(); }
}

static void MX_USART2_UART_Init(void) {
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;

    if(HAL_UART_Init(&huart2) != HAL_OK) { Error_Handler(); }
}

/* Error Handler */
void Error_Handler(void) {
    __disable_irq();
    while (1) {
        HAL_GPIO_TogglePin(GPIOB, LD1_Pin);
        HAL_Delay(100);
        HAL_GPIO_TogglePin(GPIOB, LD2_Pin);
        HAL_Delay(100);
        HAL_GPIO_TogglePin(GPIOB, LD3_Pin);
        HAL_Delay(100);
    }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {
    /* User can add implementation here */
}
#endif
