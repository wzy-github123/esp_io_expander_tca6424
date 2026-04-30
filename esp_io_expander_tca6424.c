/*
 * Licensed under the Apache License, Version 2.0 (the "License");
 * Author: wngfra
 */

#include <inttypes.h>
#include <string.h>
#include <stdlib.h>
#include "esp_bit_defs.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_io_expander.h"
#include "esp_io_expander_tca6424.h"

/* I2C communication related */
#define I2C_TIMEOUT_MS (1000)
#define I2C_CLK_SPEED (400000)

#define IO_COUNT (24)

/* Register address */
#define INPUT0_REG_ADDR  (0x00)
#define INPUT1_REG_ADDR  (0x01)
#define INPUT2_REG_ADDR  (0x02)

#define OUTPUT0_REG_ADDR (0x04)
#define OUTPUT1_REG_ADDR (0x05)
#define OUTPUT2_REG_ADDR (0x06)

#define POLARITY0_REG_ADDR (0x08)
#define POLARITY1_REG_ADDR (0x09)
#define POLARITY2_REG_ADDR (0x0a)

// Pin input output configuration, 1 for input and 0 for output
#define CONFIGURATION0_REG_ADDR   (0x0c)
#define CONFIGURATION1_REG_ADDR   (0x0d)
#define CONFIGURATION2_REG_ADDR   (0x0e)

/* Default register value on power-up */
#define DIR_REG_DEFAULT_VAL (0xfaff)
#define OUT_REG_DEFAULT_VAL (0xfbff)

/**
 * @brief Device Structure Type
 *
 */
typedef struct
{
    esp_io_expander_t base;
    i2c_master_dev_handle_t i2c_handle;
    struct
    {
        uint16_t direction;
        uint16_t output;
    } regs;
} esp_io_expander_tca6424_t;

static char *TAG = "tca6424";

static esp_err_t read_input_reg(esp_io_expander_handle_t handle, uint32_t *value);
static esp_err_t write_output_reg(esp_io_expander_handle_t handle, uint32_t value);
static esp_err_t read_output_reg(esp_io_expander_handle_t handle, uint32_t *value);
static esp_err_t write_direction_reg(esp_io_expander_handle_t handle, uint32_t value);
static esp_err_t read_direction_reg(esp_io_expander_handle_t handle, uint32_t *value);
static esp_err_t reset(esp_io_expander_t *handle);
static esp_err_t del(esp_io_expander_t *handle);

esp_err_t esp_io_expander_new_i2c_tca6424(i2c_master_bus_handle_t i2c_bus, uint32_t dev_addr, esp_io_expander_handle_t *handle_ret)
{
    ESP_RETURN_ON_FALSE(handle_ret != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle_ret");

    // Allocate memory for driver object
    esp_io_expander_tca6424_t *pca = (esp_io_expander_tca6424_t *)calloc(1, sizeof(esp_io_expander_tca6424_t));
    ESP_RETURN_ON_FALSE(pca, ESP_ERR_NO_MEM, TAG, "Malloc failed");

    // Add new I2C device
    esp_err_t ret = ESP_OK;
    const i2c_device_config_t i2c_dev_cfg = {
        .device_address = dev_addr,
        .scl_speed_hz = I2C_CLK_SPEED,
    };
    ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(i2c_bus, &i2c_dev_cfg, &pca->i2c_handle), err, TAG, "Add new I2C device failed");

    pca->base.config.io_count = IO_COUNT;
    pca->base.config.flags.dir_out_bit_zero = 1;
    pca->base.read_input_reg = read_input_reg;
    pca->base.write_output_reg = write_output_reg;
    pca->base.read_output_reg = read_output_reg;
    pca->base.write_direction_reg = write_direction_reg;
    pca->base.read_direction_reg = read_direction_reg;
    pca->base.del = del;
    pca->base.reset = reset;

    /* Reset configuration and register status */
    ESP_GOTO_ON_ERROR(reset(&pca->base), err, TAG, "Reset failed"); // Don't reset io_exp config

    *handle_ret = &pca->base;
    return ESP_OK;
err:
    free(pca);
    return ret;
}

static esp_err_t read_input_reg(esp_io_expander_handle_t handle, uint32_t *value)
{
    esp_io_expander_tca6424_t *pca = (esp_io_expander_tca6424_t *)__containerof(handle, esp_io_expander_tca6424_t, base);

    uint8_t temp[3] = {0, 0, 0};
    ESP_RETURN_ON_ERROR(i2c_master_transmit_receive(pca->i2c_handle, (uint8_t[]){INPUT0_REG_ADDR}, 1, temp, sizeof(temp), I2C_TIMEOUT_MS), TAG, "Read input reg failed");
    *value = (((uint32_t)temp[2]) << 16)|(((uint32_t)temp[1]) << 8) | (temp[0]);
    return ESP_OK;
}

static esp_err_t write_output_reg(esp_io_expander_handle_t handle, uint32_t value)
{
    esp_io_expander_tca6424_t *pca = (esp_io_expander_tca6424_t *)__containerof(handle, esp_io_expander_tca6424_t, base);
    value &= 0xffffff;

    uint8_t data[] = {OUTPUT0_REG_ADDR, value & 0xff, value >> 8,value >> 16};
    ESP_RETURN_ON_ERROR(i2c_master_transmit(pca->i2c_handle, data, sizeof(data), I2C_TIMEOUT_MS), TAG, "Write output reg failed");
    pca->regs.output = value;
    return ESP_OK;
}

static esp_err_t read_output_reg(esp_io_expander_handle_t handle, uint32_t *value)
{
    esp_io_expander_tca6424_t *pca = (esp_io_expander_tca6424_t *)__containerof(handle, esp_io_expander_tca6424_t, base);

    *value = pca->regs.output;
    return ESP_OK;
}

static esp_err_t write_direction_reg(esp_io_expander_handle_t handle, uint32_t value)
{
    esp_io_expander_tca6424_t *pca = (esp_io_expander_tca6424_t *)__containerof(handle, esp_io_expander_tca6424_t, base);
    value &= 0xffffff;

    uint8_t data[] = {CONFIGURATION0_REG_ADDR, value & 0xff, value >> 8, value >> 16};
    ESP_RETURN_ON_ERROR(i2c_master_transmit(pca->i2c_handle, data, sizeof(data), I2C_TIMEOUT_MS), TAG, "Write direction reg failed");
    pca->regs.direction = value;
    return ESP_OK;
}

static esp_err_t read_direction_reg(esp_io_expander_handle_t handle, uint32_t *value)
{
    esp_io_expander_tca6424_t *pca = (esp_io_expander_tca6424_t *)__containerof(handle, esp_io_expander_tca6424_t, base);

    *value = pca->regs.direction;
    return ESP_OK;
}

static esp_err_t reset(esp_io_expander_t *handle)
{
    ESP_RETURN_ON_ERROR(write_direction_reg(handle, DIR_REG_DEFAULT_VAL), TAG, "Write dir reg failed");
    ESP_RETURN_ON_ERROR(write_output_reg(handle, OUT_REG_DEFAULT_VAL), TAG, "Write output reg failed");
    return ESP_OK;
}

static esp_err_t del(esp_io_expander_t *handle)
{
    esp_io_expander_tca6424_t *pca = (esp_io_expander_tca6424_t *)__containerof(handle, esp_io_expander_tca6424_t, base);

    ESP_RETURN_ON_ERROR(i2c_master_bus_rm_device(pca->i2c_handle), TAG, "Remove I2C device failed");
    free(pca);
    return ESP_OK;
}