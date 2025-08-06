/*
  mlx90614.c simplified version independent of i2c.h, using passed in I2C handle and device address
*/

#include "mlx90614.h"
#include <string.h>

#define MLX90614_REGISTER_TA 0x06
#define MLX90614_REGISTER_TOBJ1 0x07
#define MLX90614_REGISTER_TOBJ2 0x08
#define MLX90614_REGISTER_TOMAX 0x20
#define MLX90614_REGISTER_TOMIN 0x21
#define MLX90614_REGISTER_PWMCTRL 0x22
#define MLX90614_REGISTER_TARANGE 0x23
#define MLX90614_REGISTER_KE 0x24
#define MLX90614_REGISTER_CONFIG 0x25
#define MLX90614_REGISTER_ADDRESS 0x2E
#define MLX90614_REGISTER_ID0 0x3C
#define MLX90614_REGISTER_ID1 0x3D
#define MLX90614_REGISTER_ID2 0x3E
#define MLX90614_REGISTER_ID3 0x3F
#define MLX90614_REGISTER_SLEEP 0xFF

static uint8_t mlx90614_crc8(uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    while (len--)
    {
        uint8_t inbyte = *data++;
        for (uint8_t i = 8; i; i--)
        {
            uint8_t carry = (crc ^ inbyte) & 0x80;
            crc <<= 1;
            if (carry)
                crc ^= 0x07;
            inbyte <<= 1;
        }
    }
    return crc;
}


static bool mlx90614_read16_internal(MLX90614_t* sensor, uint8_t address, int16_t *data)
{
    uint8_t d[3];
    if(HAL_I2C_Mem_Read(sensor->hi2c, sensor->device_address, address, I2C_MEMADD_SIZE_8BIT, d, 3, 100) != HAL_OK)
        return false;
    *data = d[0] | (d[1] << 8);
    return true;
}

static bool mlx90614_write16_internal(MLX90614_t* sensor, uint8_t address, int16_t data)
{
    uint8_t d[5];
    d[0] = sensor->device_address;
    d[1] = address;
    d[2] = 0;
    d[3] = 0;
    d[4] = mlx90614_crc8(d, 4);

    if(HAL_I2C_Mem_Write(sensor->hi2c, sensor->device_address, address, I2C_MEMADD_SIZE_8BIT, &d[2], 3, 100) != HAL_OK)
        return false;
    HAL_Delay(10);

    d[2] = data & 0x00FF;
    d[3] = data >> 8;
    d[4] = mlx90614_crc8(d, 4);
    HAL_Delay(10);

    if(HAL_I2C_Mem_Write(sensor->hi2c, sensor->device_address, address, I2C_MEMADD_SIZE_8BIT, &d[2], 3, 100) != HAL_OK)
        return false;
    HAL_Delay(10);

    int16_t comp = 0;
    if(!mlx90614_read16_internal(sensor, address, &comp))
        return false;
    if(comp != data)
        return false;
    return true;
}

int16_t mlx90614_calcRawTemp(MLX90614_t* sensor, float calcTemp)
{
    int16_t rawTemp;
    if(sensor->unit == MLX90614_UNIT_RAW)
    {
        rawTemp = (int16_t)calcTemp;
    }
    else
    {
        float tempFloat;
        if(sensor->unit == MLX90614_UNIT_F)
        {
            tempFloat = (calcTemp - 32.0f) * 5.0f / 9.0f + 273.15f;
        }
        else if(sensor->unit == MLX90614_UNIT_C)
        {
            tempFloat = calcTemp + 273.15f;
        }
        else if(sensor->unit == MLX90614_UNIT_K)
        {
            tempFloat = calcTemp;
        }
        else
            tempFloat = calcTemp;

        tempFloat *= 50.0f;
        rawTemp = (int16_t)tempFloat;
    }
    return rawTemp;
}

float mlx90614_calcTemperature(MLX90614_t* sensor, int16_t rawTemp)
{
    float retTemp;
    if(sensor->unit == MLX90614_UNIT_RAW)
    {
        retTemp = (float)rawTemp;
    }
    else
    {
        retTemp = (float)(rawTemp) * 0.02f;
        if(sensor->unit != MLX90614_UNIT_K)
            retTemp -= 273.15f;
        if(sensor->unit == MLX90614_UNIT_F)
            retTemp = retTemp * 9.0f / 5.0f + 32.0f;
    }
    return retTemp;
}

bool mlx90614_init(MLX90614_t* sensor, I2C_HandleTypeDef* hi2c, uint16_t devAddress)
{
    memset(sensor, 0, sizeof(MLX90614_t));
    sensor->unit = MLX90614_UNIT_C;
    sensor->hi2c = hi2c;
    sensor->device_address = devAddress;
    if(HAL_I2C_IsDeviceReady(hi2c, devAddress, 1, 100) != HAL_OK)
        return false;
    if(!mlx90614_read16_internal(sensor, MLX90614_REGISTER_CONFIG, (int16_t*)&sensor->configReg)) return false;
    if(!mlx90614_readID(sensor, NULL)) return false;
    if(!mlx90614_getEmissivity(sensor, NULL)) return false;
    if(!mlx90614_getMax(sensor, NULL)) return false;
    if(!mlx90614_getMin(sensor, NULL)) return false;
    return true;
}

void mlx90614_setUnit(MLX90614_t* sensor, MLX90614_UNIT_t MLX90614_UNIT_)
{
    sensor->unit = MLX90614_UNIT_;
}

bool mlx90614_readID(MLX90614_t* sensor, int16_t *id)
{
    for(int i=0; i<4; i++)
    {
        int16_t temp = 0;
        if(!mlx90614_read16_internal(sensor, MLX90614_REGISTER_ID0 + i, &temp))
            return false;
        if(id != NULL)
            id[i] = (uint16_t)temp;
        sensor->id[i] = (uint16_t)temp;
    }
    return true;
}

bool mlx90614_getEmissivity(MLX90614_t* sensor, float *emissivity)
{
    if(mlx90614_read16_internal(sensor, MLX90614_REGISTER_KE, &sensor->rawEmissivity))
    {
        sensor->emissivity = ((float)((uint16_t)sensor->rawEmissivity)) / 65535.0f;
        if(emissivity != NULL)
            *emissivity = sensor->emissivity;
        return true;
    }
    return false;
}

bool mlx90614_setEmissivity(MLX90614_t* sensor, float emissivity)
{
    if(emissivity > 1.0f || emissivity < 0.1f)
        return false;
    sensor->rawEmissivity = (uint16_t)(65535.0f * emissivity);
    if(sensor->rawEmissivity < 0x2000)
        sensor->rawEmissivity = 0x2000;
    return mlx90614_write16_internal(sensor, MLX90614_REGISTER_KE, (int16_t)sensor->rawEmissivity);
}

bool mlx90614_setMax(MLX90614_t* sensor, float maxTemp)
{
    sensor->rawMax = mlx90614_calcRawTemp(sensor, maxTemp);
    return mlx90614_write16_internal(sensor, MLX90614_REGISTER_TOMAX, sensor->rawMax);
}

bool mlx90614_setMin(MLX90614_t* sensor, float minTemp)
{
    sensor->rawMin = mlx90614_calcRawTemp(sensor, minTemp);
    return mlx90614_write16_internal(sensor, MLX90614_REGISTER_TOMIN, sensor->rawMin);
}

bool mlx90614_getMax(MLX90614_t* sensor, float *maxTemp)
{
    if(mlx90614_read16_internal(sensor, MLX90614_REGISTER_TOMAX, &sensor->rawMax))
    {
        if(maxTemp != NULL)
            *maxTemp = mlx90614_calcTemperature(sensor, sensor->rawMax);
        return true;
    }
    return false;
}

bool mlx90614_getMin(MLX90614_t* sensor, float *minTemp)
{
    if(mlx90614_read16_internal(sensor, MLX90614_REGISTER_TOMIN, &sensor->rawMin))
    {
        if(minTemp != NULL)
            *minTemp = mlx90614_calcTemperature(sensor, sensor->rawMin);
        return true;
    }
    return false;
}

bool mlx90614_getAmbient(MLX90614_t* sensor, float *ambientTemp)
{
    if(mlx90614_read16_internal(sensor, MLX90614_REGISTER_TA, &sensor->rawAmbient))
    {
        if(ambientTemp != NULL)
            *ambientTemp = mlx90614_calcTemperature(sensor, sensor->rawAmbient);
        return true;
    }
    return false;
}

bool mlx90614_getObject1(MLX90614_t* sensor, float *objectTemp)
{
    if(mlx90614_read16_internal(sensor, MLX90614_REGISTER_TOBJ1, &sensor->rawObject1))
    {
        if(sensor->rawObject1 & 0x8000) return false;
        if(objectTemp != NULL)
            *objectTemp = mlx90614_calcTemperature(sensor, sensor->rawObject1);
        return true;
    }
    return false;
}

bool mlx90614_getObject2(MLX90614_t* sensor, float *objectTemp)
{
    if(mlx90614_read16_internal(sensor, MLX90614_REGISTER_TOBJ2, &sensor->rawObject2))
    {
        if(sensor->rawObject2 & 0x8000) return false;
        if(objectTemp != NULL)
            *objectTemp = mlx90614_calcTemperature(sensor, sensor->rawObject2);
        return true;
    }
    return false;
}
