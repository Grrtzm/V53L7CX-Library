/**
 *
 * Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include "platform.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_check.h"

static i2c_master_bus_handle_t s_bus = NULL;
static const char *TAG = "PLATFORM";

esp_err_t VL53L7CX_Platform_Init(
    VL53L7CX_Platform *platform,
    const i2c_master_bus_config_t *bus_cfg,
    const i2c_device_config_t *dev_cfg)
{
    if (!platform || !bus_cfg || !dev_cfg)
    {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err;

    /* Create I2C bus once */
    if (s_bus == NULL)
    {
        err = i2c_new_master_bus(bus_cfg, &s_bus);
        if (err != ESP_OK)
        {
            return err;
        }
    }

    /* Add device */
    err = i2c_master_bus_add_device(s_bus, dev_cfg, &platform->handle);
    if (err != ESP_OK)
    {
        return err;
    }

    /* Store for compatibility / debugging */
    platform->bus_config = *bus_cfg;
    platform->address = dev_cfg->device_address;

    return ESP_OK;
}

static inline uint8_t vl53_err_to_status(esp_err_t err)
{
    return (err == ESP_OK) ? 0 : 1;
}

uint8_t VL53L7CX_WrMulti(VL53L7CX_Platform *p_platform,
                         uint16_t RegisterAdress,
                         uint8_t *p_values,
                         uint32_t size)
{
    if (!p_platform || !p_platform->handle || (!p_values && size))
    {
        return 1;
    }

    const uint32_t CHUNK = VL53L7CX_I2C_CHUNK;

    uint16_t reg = RegisterAdress;
    uint32_t remaining = size;
    uint8_t *p = p_values;

    while (remaining > 0)
    {
        const uint32_t n = (remaining > CHUNK) ? CHUNK : remaining;

        i2c_master_transmit_multi_buffer_info_t i2c_buffers[2];

        // 16-bit register address, big-endian
        uint8_t i2c_address[2] = {(uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF)};

        i2c_buffers[0].write_buffer = i2c_address;
        i2c_buffers[0].buffer_size = 2;

        i2c_buffers[1].write_buffer = p;
        i2c_buffers[1].buffer_size = n;

        esp_err_t err = i2c_master_multi_buffer_transmit(
            p_platform->handle, i2c_buffers, 2, VL53L7CX_I2C_TIMEOUT);

        if (err != ESP_OK)
        {
            return vl53_err_to_status(err);
        }

        // Next chunk: increment register address (auto-increment addressing)
        reg = (uint16_t)(reg + n);
        p += n;
        remaining -= n;
    }

    return 0;
}

uint8_t VL53L7CX_WrByte(VL53L7CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t value)
{

    // Write a single byte
    return VL53L7CX_WrMulti(p_platform, RegisterAdress, &value, 1);
}

uint8_t VL53L7CX_RdMulti(VL53L7CX_Platform *p_platform,
                         uint16_t RegisterAdress,
                         uint8_t *p_values,
                         uint32_t size)
{
    if (!p_platform || !p_platform->handle || (!p_values && size))
    {
        return 1;
    }

    const uint32_t CHUNK = 240;

    uint16_t reg = RegisterAdress;
    uint32_t remaining = size;
    uint8_t *p = p_values;

    while (remaining > 0)
    {
        const uint32_t n = (remaining > CHUNK) ? CHUNK : remaining;

        uint8_t i2c_address[2] = {(uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF)};

        esp_err_t err = i2c_master_transmit_receive(
            p_platform->handle, i2c_address, 2, p, n, VL53L7CX_I2C_TIMEOUT);

        if (err != ESP_OK)
        {
            return vl53_err_to_status(err);
        }

        reg = (uint16_t)(reg + n);
        p += n;
        remaining -= n;
    }

    return 0;
}

uint8_t VL53L7CX_RdByte(VL53L7CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_value)
{

    // Read a single byte
    return VL53L7CX_RdMulti(p_platform, RegisterAdress, p_value, 1);
}

uint8_t VL53L7CX_Reset_Sensor(VL53L7CX_Platform *p_platform)
{
    gpio_set_direction(p_platform->reset_gpio, GPIO_MODE_OUTPUT);

    gpio_set_level(p_platform->reset_gpio, VL53L7CX_RESET_LEVEL);
    VL53L7CX_WaitMs(p_platform, 100);

    gpio_set_level(p_platform->reset_gpio, !VL53L7CX_RESET_LEVEL);
    VL53L7CX_WaitMs(p_platform, 100);

    return ESP_OK;
}

uint8_t VL53L7CX_WaitMs(VL53L7CX_Platform *p_platform, uint32_t TimeMs)
{
    vTaskDelay(TimeMs / portTICK_PERIOD_MS);

    return ESP_OK;
}

void VL53L7CX_SwapBuffer(uint8_t *buffer, uint32_t size)
{
    for (uint32_t i = 0; i + 3 < size; i += 4)
    {
        uint8_t b0 = buffer[i + 0];
        uint8_t b1 = buffer[i + 1];
        buffer[i + 0] = buffer[i + 3];
        buffer[i + 1] = buffer[i + 2];
        buffer[i + 2] = b1;
        buffer[i + 3] = b0;
    }
}
