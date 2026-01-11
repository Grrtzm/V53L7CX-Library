/**
  * Platform layer for VL53L7CX on ESP-IDF v5.5.x (new I2C master driver).
  *
  * This file is intentionally small and allocation-free.
  *
  */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "driver/gpio.h"
#include "driver/i2c_master.h"

// Conservative chunk size; 240 is a safe default for many I2C implementations.
// You can try 512 if your bus is clean and stable.
#ifndef VL53L7CX_I2C_CHUNK
#define VL53L7CX_I2C_CHUNK 240
#endif

/*
 * @brief The macro below is used to define the number of target per zone sent
 * through I2C. This value can be changed by user, in order to tune I2C
 * transaction, and also the total memory size (a lower number of target per
 * zone means a lower RAM). The value must be between 1 and 4.
 */

#ifndef VL53L7CX_NB_TARGET_PER_ZONE
#define VL53L7CX_NB_TARGET_PER_ZONE 1U
#endif

/* Default timeout (ms). You can override via Kconfig if desired. */
#ifndef VL53L7CX_I2C_TIMEOUT
#define VL53L7CX_I2C_TIMEOUT 50
#endif

/* Reset polarity: define one of these in sdkconfig if you use reset */
#ifdef CONFIG_VL53L7CX_RESET_PIN_HIGH
#define VL53L7CX_RESET_LEVEL 1
#elif defined(CONFIG_VL53L7CX_RESET_PIN_LOW)
#define VL53L7CX_RESET_LEVEL 0
#endif
#ifndef VL53L7CX_RESET_LEVEL
#define VL53L7CX_RESET_LEVEL 0
#endif

typedef struct
{
    i2c_master_dev_handle_t  handle;      /* device handle returned by i2c_master_bus_add_device() */
    i2c_master_bus_config_t  bus_config;  /* RJRP44-compatible: store bus config (optional use) */
    uint16_t                 address;     /* 7-bit address (0x29) */
    gpio_num_t               reset_gpio;  /* optional */
} VL53L7CX_Platform;

#define VL53L7CX_MAX_CLK_SPEED 1000000

esp_err_t VL53L7CX_Platform_Init(
    VL53L7CX_Platform *platform,
    const i2c_master_bus_config_t *bus_cfg,
    const i2c_device_config_t *dev_cfg);

/**
 * @brief Mandatory function used to write multiples bytes.
 * @param (VL53L7CX_Platform*) p_platform : Pointer of VL53L7CX platform
 * structure.
 * @param (uint16_t) Address : I2C location of values to write.
 * @param (uint8_t) *p_values : Buffer of bytes to write.
 * @param (uint32_t) size : Size of *p_values buffer.
 * @return (uint8_t) status : 0 if OK
 */
uint8_t VL53L7CX_WrMulti(VL53L7CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_values, uint32_t size);

/**
 * @brief Mandatory function used to read multiples bytes.
 * @param (VL53L7CX_Platform*) p_platform : Pointer of VL53L7CX platform
 * structure.
 * @param (uint16_t) Address : I2C location of values to read.
 * @param (uint8_t) *p_values : Buffer of bytes to read.
 * @param (uint32_t) size : Size of *p_values buffer.
 * @return (uint8_t) status : 0 if OK
 */
uint8_t VL53L7CX_RdMulti(VL53L7CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_values, uint32_t size);

/**
 * @brief Mandatory function used to write one single byte.
 * @param (VL53L7CX_Platform*) p_platform : Pointer of VL53L7CX platform
 * structure.
 * @param (uint16_t) Address : I2C location of value to read.
 * @param (uint8_t) value : Pointer of value to write.
 * @return (uint8_t) status : 0 if OK
 */
uint8_t VL53L7CX_WrByte (VL53L7CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t value);

/**
 * @brief Function used to read one single byte.
 * @param (VL53L7CX_Platform*) p_platform : Pointer of VL53L7CX platform
 * structure.
 * @param (uint16_t) Address : I2C location of value to read.
 * @param (uint8_t) *p_values : Pointer of value to read.
 * @return (uint8_t) status : 0 if OK
 */
uint8_t VL53L7CX_RdByte (VL53L7CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_value);

/**
 * @brief Optional function, only used to perform an hardware reset of the
 * sensor. This function is not used in the API, but it can be used by the host.
 * This function is not mandatory to fill if user don't want to reset the
 * sensor.
 * @param (VL53L7CX_Platform*) p_platform : Pointer of VL53L7CX platform
 * structure.
 * @return (uint8_t) status : 0 if OK
 */
uint8_t VL53L7CX_Reset_Sensor(VL53L7CX_Platform *p_platform);

/**
 * @brief Mandatory function, used to wait during an amount of time. It must be
 * filled as it's used into the API.
 * @param (VL53L7CX_Platform*) p_platform : Pointer of VL53L7CX platform
 * structure.
 * @param (uint32_t) TimeMs : Time to wait in ms.
 * @return (uint8_t) status : 0 if wait is finished.
 */
uint8_t VL53L7CX_WaitMs(VL53L7CX_Platform *p_platform, uint32_t TimeMs);

void VL53L7CX_SwapBuffer(uint8_t *buffer, uint32_t size);

static inline uint8_t WrMulti(void *p_platform, uint16_t reg, uint8_t *pdata, uint32_t count)
{
    return VL53L7CX_WrMulti((VL53L7CX_Platform *)p_platform, reg, pdata, count);
}

static inline uint8_t RdMulti(void *p_platform, uint16_t reg, uint8_t *pdata, uint32_t count)
{
    return VL53L7CX_RdMulti((VL53L7CX_Platform *)p_platform, reg, pdata, count);
}

static inline uint8_t WrByte(void *p_platform, uint16_t reg, uint8_t value)
{
    return VL53L7CX_WrByte((VL53L7CX_Platform *)p_platform, reg, value);
}

static inline uint8_t RdByte(void *p_platform, uint16_t reg, uint8_t *value)
{
    return VL53L7CX_RdByte((VL53L7CX_Platform *)p_platform, reg, value);
}

static inline uint8_t WaitMs(void *p_platform, uint32_t ms)
{
    return VL53L7CX_WaitMs((VL53L7CX_Platform *)p_platform, ms);
}

static inline void SwapBuffer(uint8_t *buffer, uint32_t size)
{
    VL53L7CX_SwapBuffer(buffer, (uint16_t)size);
}

#ifdef __cplusplus
}
#endif
