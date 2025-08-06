/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bno055.h
  * @brief          : BNO055 9-DOF IMU Sensor Header
  ******************************************************************************
  * @attention
  * This software component is licensed under terms in the LICENSE file.
  * If no LICENSE file is present, this software is provided AS-IS.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef BNO055_H_
#define BNO055_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* BNO055 I2C Address */
#define BNO055_ADDRESS_A                0x28
#define BNO055_ADDRESS_B                0x29
#define BNO055_ID                       0xA0

/* BNO055 Register Map */
#define BNO055_CHIP_ID_ADDR             0x00
#define BNO055_ACCEL_REV_ID_ADDR        0x01
#define BNO055_MAG_REV_ID_ADDR          0x02
#define BNO055_GYRO_REV_ID_ADDR         0x03
#define BNO055_SW_REV_ID_LSB_ADDR       0x04
#define BNO055_SW_REV_ID_MSB_ADDR       0x05
#define BNO055_BL_REV_ID_ADDR           0x06

/* Accelerometer data register */
#define BNO055_ACCEL_DATA_X_LSB_ADDR    0x08
#define BNO055_ACCEL_DATA_X_MSB_ADDR    0x09
#define BNO055_ACCEL_DATA_Y_LSB_ADDR    0x0A
#define BNO055_ACCEL_DATA_Y_MSB_ADDR    0x0B
#define BNO055_ACCEL_DATA_Z_LSB_ADDR    0x0C
#define BNO055_ACCEL_DATA_Z_MSB_ADDR    0x0D

/* Magnetometer data register */
#define BNO055_MAG_DATA_X_LSB_ADDR      0x0E
#define BNO055_MAG_DATA_X_MSB_ADDR      0x0F
#define BNO055_MAG_DATA_Y_LSB_ADDR      0x10
#define BNO055_MAG_DATA_Y_MSB_ADDR      0x11
#define BNO055_MAG_DATA_Z_LSB_ADDR      0x12
#define BNO055_MAG_DATA_Z_MSB_ADDR      0x13

/* Gyroscope data registers */
#define BNO055_GYRO_DATA_X_LSB_ADDR     0x14
#define BNO055_GYRO_DATA_X_MSB_ADDR     0x15
#define BNO055_GYRO_DATA_Y_LSB_ADDR     0x16
#define BNO055_GYRO_DATA_Y_MSB_ADDR     0x17
#define BNO055_GYRO_DATA_Z_LSB_ADDR     0x18
#define BNO055_GYRO_DATA_Z_MSB_ADDR     0x19

/* Euler data registers */
#define BNO055_EULER_H_LSB_ADDR         0x1A
#define BNO055_EULER_H_MSB_ADDR         0x1B
#define BNO055_EULER_R_LSB_ADDR         0x1C
#define BNO055_EULER_R_MSB_ADDR         0x1D
#define BNO055_EULER_P_LSB_ADDR         0x1E
#define BNO055_EULER_P_MSB_ADDR         0x1F

/* Quaternion data registers */
#define BNO055_QUATERNION_DATA_W_LSB_ADDR 0x20
#define BNO055_QUATERNION_DATA_W_MSB_ADDR 0x21
#define BNO055_QUATERNION_DATA_X_LSB_ADDR 0x22
#define BNO055_QUATERNION_DATA_X_MSB_ADDR 0x23
#define BNO055_QUATERNION_DATA_Y_LSB_ADDR 0x24
#define BNO055_QUATERNION_DATA_Y_MSB_ADDR 0x25
#define BNO055_QUATERNION_DATA_Z_LSB_ADDR 0x26
#define BNO055_QUATERNION_DATA_Z_MSB_ADDR 0x27

/* Linear acceleration data registers */
#define BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR 0x28
#define BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR 0x29
#define BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR 0x2A
#define BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR 0x2B
#define BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR 0x2C
#define BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR 0x2D

/* Gravity data registers */
#define BNO055_GRAVITY_DATA_X_LSB_ADDR  0x2E
#define BNO055_GRAVITY_DATA_X_MSB_ADDR  0x2F
#define BNO055_GRAVITY_DATA_Y_LSB_ADDR  0x30
#define BNO055_GRAVITY_DATA_Y_MSB_ADDR  0x31
#define BNO055_GRAVITY_DATA_Z_LSB_ADDR  0x32
#define BNO055_GRAVITY_DATA_Z_MSB_ADDR  0x33

/* Temperature data register */
#define BNO055_TEMP_ADDR                0x34

/* Status registers */
#define BNO055_CALIB_STAT_ADDR          0x35
#define BNO055_SELFTEST_RESULT_ADDR     0x36
#define BNO055_INTR_STAT_ADDR           0x37
#define BNO055_SYS_CLK_STAT_ADDR        0x38
#define BNO055_SYS_STAT_ADDR            0x39
#define BNO055_SYS_ERR_ADDR             0x3A

/* Unit selection register */
#define BNO055_UNIT_SEL_ADDR            0x3B
#define BNO055_DATA_SELECT_ADDR         0x3C

/* Mode registers */
#define BNO055_OPR_MODE_ADDR            0x3D
#define BNO055_PWR_MODE_ADDR            0x3E

/* Reset register */
#define BNO055_SYS_RST_ADDR             0x3F

/* Power modes */
typedef enum {
    BNO055_POWER_MODE_NORMAL   = 0x00,
    BNO055_POWER_MODE_LOWPOWER = 0x01,
    BNO055_POWER_MODE_SUSPEND  = 0x02
} bno055_powermode_t;

/* Operation mode settings */
typedef enum {
    OPERATION_MODE_CONFIG        = 0x00,
    OPERATION_MODE_ACCONLY       = 0x01,
    OPERATION_MODE_MAGONLY       = 0x02,
    OPERATION_MODE_GYRONLY       = 0x03,
    OPERATION_MODE_ACCMAG        = 0x04,
    OPERATION_MODE_ACCGYRO       = 0x05,
    OPERATION_MODE_MAGGYRO       = 0x06,
    OPERATION_MODE_AMG           = 0x07,
    OPERATION_MODE_IMUPLUS       = 0x08,
    OPERATION_MODE_COMPASS       = 0x09,
    OPERATION_MODE_M4G           = 0x0A,
    OPERATION_MODE_NDOF_FMC_OFF  = 0x0B,
    OPERATION_MODE_NDOF          = 0x0C
} bno055_opmode_t;

/* BNO055 vector type */
typedef struct {
    float x;
    float y;
    float z;
} bno055_vector_t;

/* BNO055 quaternion type */
typedef struct {
    float w;
    float x;
    float y;
    float z;
} bno055_quaternion_t;

/* BNO055 device structure */
typedef struct {
    I2C_HandleTypeDef* hi2c;
    uint16_t device_address;
    bno055_opmode_t mode;
    uint8_t chip_id;
    uint8_t accel_rev_id;
    uint8_t mag_rev_id;
    uint8_t gyro_rev_id;
    uint16_t sw_rev_id;
    uint8_t bl_rev_id;
} bno055_t;

/* Function prototypes */
bool bno055_init(bno055_t* sensor, I2C_HandleTypeDef* hi2c, uint16_t devAddress);
bool bno055_setOperationMode(bno055_t* sensor, bno055_opmode_t mode);
bool bno055_setPowerMode(bno055_t* sensor, bno055_powermode_t powermode);
bool bno055_getChipID(bno055_t* sensor, uint8_t* id);
bool bno055_getRevInfo(bno055_t* sensor);
bool bno055_getSystemStatus(bno055_t* sensor, uint8_t* system_status, uint8_t* self_test_result, uint8_t* system_error);
bool bno055_getCalibrationStatus(bno055_t* sensor, uint8_t* sys, uint8_t* gyro, uint8_t* accel, uint8_t* mag);
bool bno055_getVector(bno055_t* sensor, uint8_t vector_type, bno055_vector_t* vector);
bool bno055_getQuaternion(bno055_t* sensor, bno055_quaternion_t* quat);
bool bno055_getTemperature(bno055_t* sensor, int8_t* temp);
bool bno055_reset(bno055_t* sensor);

/* Vector types for getVector function */
#define VECTOR_ACCELEROMETER     BNO055_ACCEL_DATA_X_LSB_ADDR
#define VECTOR_MAGNETOMETER      BNO055_MAG_DATA_X_LSB_ADDR
#define VECTOR_GYROSCOPE         BNO055_GYRO_DATA_X_LSB_ADDR
#define VECTOR_EULER             BNO055_EULER_H_LSB_ADDR
#define VECTOR_LINEARACCEL       BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR
#define VECTOR_GRAVITY           BNO055_GRAVITY_DATA_X_LSB_ADDR

#ifdef __cplusplus
}
#endif

#endif /* BNO055_H_ */
