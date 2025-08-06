/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bno055.c
  * @brief          : BNO055 9-DOF IMU Sensor Implementation
  ******************************************************************************
  * @attention
  * This software component is licensed under terms in the LICENSE file.
  * If no LICENSE file is present, this software is provided AS-IS.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "bno055.h"

/* Private function prototypes */
static bool bno055_writeRegister(bno055_t* sensor, uint8_t reg, uint8_t value);
static bool bno055_readRegister(bno055_t* sensor, uint8_t reg, uint8_t* data);
static bool bno055_readData(bno055_t* sensor, uint8_t reg, uint8_t* buffer, uint8_t len);

/**
  * @brief  Initialize BNO055 sensor
  * @param  sensor: Pointer to BNO055 structure
  * @param  hi2c: Pointer to I2C handle
  * @param  devAddress: I2C device address
  * @retval true if success, false otherwise
  */
bool bno055_init(bno055_t* sensor, I2C_HandleTypeDef* hi2c, uint16_t devAddress)
{
    if (sensor == NULL || hi2c == NULL) return false;
    
    sensor->hi2c = hi2c;
    sensor->device_address = devAddress << 1; // HAL uses 8-bit addresses
    
    // Check chip ID
    uint8_t id;
    if (!bno055_getChipID(sensor, &id)) {
        return false;
    }
    
    if (id != BNO055_ID) {
        return false;
    }
    
    // Reset the device
    if (!bno055_reset(sensor)) {
        return false;
    }
    
    // Wait for reset to complete
    HAL_Delay(650);
    
    // Set to normal power mode
    if (!bno055_setPowerMode(sensor, BNO055_POWER_MODE_NORMAL)) {
        return false;
    }
    
    HAL_Delay(10);
    
    // Get revision info
    if (!bno055_getRevInfo(sensor)) {
        return false;
    }
    
    // Set operation mode to NDOF
    if (!bno055_setOperationMode(sensor, OPERATION_MODE_NDOF)) {
        return false;
    }
    
    HAL_Delay(20);
    
    return true;
}

/**
  * @brief  Set operation mode
  * @param  sensor: Pointer to BNO055 structure
  * @param  mode: Operation mode
  * @retval true if success, false otherwise
  */
bool bno055_setOperationMode(bno055_t* sensor, bno055_opmode_t mode)
{
    if (sensor == NULL) return false;
    
    sensor->mode = mode;
    if (!bno055_writeRegister(sensor, BNO055_OPR_MODE_ADDR, mode)) {
        return false;
    }
    
    HAL_Delay(30);
    return true;
}

/**
  * @brief  Set power mode
  * @param  sensor: Pointer to BNO055 structure
  * @param  powermode: Power mode
  * @retval true if success, false otherwise
  */
bool bno055_setPowerMode(bno055_t* sensor, bno055_powermode_t powermode)
{
    if (sensor == NULL) return false;
    
    return bno055_writeRegister(sensor, BNO055_PWR_MODE_ADDR, powermode);
}

/**
  * @brief  Get chip ID
  * @param  sensor: Pointer to BNO055 structure
  * @param  id: Pointer to store chip ID
  * @retval true if success, false otherwise
  */
bool bno055_getChipID(bno055_t* sensor, uint8_t* id)
{
    if (sensor == NULL || id == NULL) return false;
    
    if (!bno055_readRegister(sensor, BNO055_CHIP_ID_ADDR, id)) {
        return false;
    }
    
    sensor->chip_id = *id;
    return true;
}

/**
  * @brief  Get revision information
  * @param  sensor: Pointer to BNO055 structure
  * @retval true if success, false otherwise
  */
bool bno055_getRevInfo(bno055_t* sensor)
{
    if (sensor == NULL) return false;
    
    uint8_t a, b;
    
    if (!bno055_readRegister(sensor, BNO055_ACCEL_REV_ID_ADDR, &sensor->accel_rev_id)) {
        return false;
    }
    
    if (!bno055_readRegister(sensor, BNO055_MAG_REV_ID_ADDR, &sensor->mag_rev_id)) {
        return false;
    }
    
    if (!bno055_readRegister(sensor, BNO055_GYRO_REV_ID_ADDR, &sensor->gyro_rev_id)) {
        return false;
    }
    
    if (!bno055_readRegister(sensor, BNO055_BL_REV_ID_ADDR, &sensor->bl_rev_id)) {
        return false;
    }
    
    if (!bno055_readRegister(sensor, BNO055_SW_REV_ID_LSB_ADDR, &a)) {
        return false;
    }
    
    if (!bno055_readRegister(sensor, BNO055_SW_REV_ID_MSB_ADDR, &b)) {
        return false;
    }
    
    sensor->sw_rev_id = (((uint16_t)b) << 8) | ((uint16_t)a);
    
    return true;
}

/**
  * @brief  Get system status
  * @param  sensor: Pointer to BNO055 structure
  * @param  system_status: Pointer to store system status
  * @param  self_test_result: Pointer to store self test result
  * @param  system_error: Pointer to store system error
  * @retval true if success, false otherwise
  */
bool bno055_getSystemStatus(bno055_t* sensor, uint8_t* system_status, uint8_t* self_test_result, uint8_t* system_error)
{
    if (sensor == NULL) return false;
    
    if (system_status != NULL) {
        if (!bno055_readRegister(sensor, BNO055_SYS_STAT_ADDR, system_status)) {
            return false;
        }
    }
    
    if (self_test_result != NULL) {
        if (!bno055_readRegister(sensor, BNO055_SELFTEST_RESULT_ADDR, self_test_result)) {
            return false;
        }
    }
    
    if (system_error != NULL) {
        if (!bno055_readRegister(sensor, BNO055_SYS_ERR_ADDR, system_error)) {
            return false;
        }
    }
    
    return true;
}

/**
  * @brief  Get calibration status
  * @param  sensor: Pointer to BNO055 structure
  * @param  sys: Pointer to store system calibration status
  * @param  gyro: Pointer to store gyro calibration status
  * @param  accel: Pointer to store accel calibration status
  * @param  mag: Pointer to store magnetometer calibration status
  * @retval true if success, false otherwise
  */
bool bno055_getCalibrationStatus(bno055_t* sensor, uint8_t* sys, uint8_t* gyro, uint8_t* accel, uint8_t* mag)
{
    if (sensor == NULL) return false;
    
    uint8_t calData = 0;
    if (!bno055_readRegister(sensor, BNO055_CALIB_STAT_ADDR, &calData)) {
        return false;
    }
    
    if (sys != NULL) *sys = (calData >> 6) & 0x03;
    if (gyro != NULL) *gyro = (calData >> 4) & 0x03;
    if (accel != NULL) *accel = (calData >> 2) & 0x03;
    if (mag != NULL) *mag = calData & 0x03;
    
    return true;
}

/**
  * @brief  Get vector data
  * @param  sensor: Pointer to BNO055 structure
  * @param  vector_type: Type of vector to read
  * @param  vector: Pointer to store vector data
  * @retval true if success, false otherwise
  */
bool bno055_getVector(bno055_t* sensor, uint8_t vector_type, bno055_vector_t* vector)
{
    if (sensor == NULL || vector == NULL) return false;
    
    uint8_t buffer[6];
    if (!bno055_readData(sensor, vector_type, buffer, 6)) {
        return false;
    }
    
    int16_t x = ((int16_t)buffer[1] << 8) | buffer[0];
    int16_t y = ((int16_t)buffer[3] << 8) | buffer[2];
    int16_t z = ((int16_t)buffer[5] << 8) | buffer[4];
    
    /* Convert the value to appropriate range (section 3.6.4) */
    /* and assign the value to the Vector type */
    switch(vector_type) {
        case VECTOR_MAGNETOMETER:
            /* 1uT = 16 LSB */
            vector->x = ((double)x)/16.0;
            vector->y = ((double)y)/16.0;
            vector->z = ((double)z)/16.0;
            break;
        case VECTOR_GYROSCOPE:
            /* 1dps = 16 LSB */
            vector->x = ((double)x)/16.0;
            vector->y = ((double)y)/16.0;
            vector->z = ((double)z)/16.0;
            break;
        case VECTOR_EULER:
            /* 1 degree = 16 LSB */
            vector->x = ((double)x)/16.0;
            vector->y = ((double)y)/16.0;
            vector->z = ((double)z)/16.0;
            break;
        case VECTOR_ACCELEROMETER:
        case VECTOR_LINEARACCEL:
        case VECTOR_GRAVITY:
            /* 1m/s^2 = 100 LSB */
            vector->x = ((double)x)/100.0;
            vector->y = ((double)y)/100.0;
            vector->z = ((double)z)/100.0;
            break;
    }
    
    return true;
}

/**
  * @brief  Get quaternion data
  * @param  sensor: Pointer to BNO055 structure
  * @param  quat: Pointer to store quaternion data
  * @retval true if success, false otherwise
  */
bool bno055_getQuaternion(bno055_t* sensor, bno055_quaternion_t* quat)
{
    if (sensor == NULL || quat == NULL) return false;
    
    uint8_t buffer[8];
    if (!bno055_readData(sensor, BNO055_QUATERNION_DATA_W_LSB_ADDR, buffer, 8)) {
        return false;
    }
    
    int16_t w = ((int16_t)buffer[1] << 8) | buffer[0];
    int16_t x = ((int16_t)buffer[3] << 8) | buffer[2];
    int16_t y = ((int16_t)buffer[5] << 8) | buffer[4];
    int16_t z = ((int16_t)buffer[7] << 8) | buffer[6];
    
    /* Assign to quaternion */
    /* See Table 3-31 QUATERNION format 1 quaternion = 2^14 LSB */
    const double scale = (1.0 / (1<<14));
    quat->w = scale * w;
    quat->x = scale * x;
    quat->y = scale * y;
    quat->z = scale * z;
    
    return true;
}

/**
  * @brief  Get temperature
  * @param  sensor: Pointer to BNO055 structure
  * @param  temp: Pointer to store temperature
  * @retval true if success, false otherwise
  */
bool bno055_getTemperature(bno055_t* sensor, int8_t* temp)
{
    if (sensor == NULL || temp == NULL) return false;
    
    return bno055_readRegister(sensor, BNO055_TEMP_ADDR, (uint8_t*)temp);
}

/**
  * @brief  Reset BNO055
  * @param  sensor: Pointer to BNO055 structure
  * @retval true if success, false otherwise
  */
bool bno055_reset(bno055_t* sensor)
{
    if (sensor == NULL) return false;
    
    return bno055_writeRegister(sensor, BNO055_SYS_RST_ADDR, 0x20);
}

/* Private functions */

/**
  * @brief  Write register
  * @param  sensor: Pointer to BNO055 structure
  * @param  reg: Register address
  * @param  value: Value to write
  * @retval true if success, false otherwise
  */
static bool bno055_writeRegister(bno055_t* sensor, uint8_t reg, uint8_t value)
{
    if (sensor == NULL) return false;
    
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(sensor->hi2c, sensor->device_address, 
                                                 reg, I2C_MEMADD_SIZE_8BIT, 
                                                 &value, 1, HAL_MAX_DELAY);
    return (status == HAL_OK);
}

/**
  * @brief  Read register
  * @param  sensor: Pointer to BNO055 structure
  * @param  reg: Register address
  * @param  data: Pointer to store read data
  * @retval true if success, false otherwise
  */
static bool bno055_readRegister(bno055_t* sensor, uint8_t reg, uint8_t* data)
{
    if (sensor == NULL || data == NULL) return false;
    
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(sensor->hi2c, sensor->device_address, 
                                                reg, I2C_MEMADD_SIZE_8BIT, 
                                                data, 1, HAL_MAX_DELAY);
    return (status == HAL_OK);
}

/**
  * @brief  Read multiple bytes
  * @param  sensor: Pointer to BNO055 structure
  * @param  reg: Register address
  * @param  buffer: Pointer to store read data
  * @param  len: Number of bytes to read
  * @retval true if success, false otherwise
  */
static bool bno055_readData(bno055_t* sensor, uint8_t reg, uint8_t* buffer, uint8_t len)
{
    if (sensor == NULL || buffer == NULL) return false;
    
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(sensor->hi2c, sensor->device_address, 
                                                reg, I2C_MEMADD_SIZE_8BIT, 
                                                buffer, len, HAL_MAX_DELAY);
    return (status == HAL_OK);
}
