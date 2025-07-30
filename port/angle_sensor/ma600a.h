/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "common/base_classes/Sensor.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

#define PI                  3.14159265358979f

class MA600A : public Sensor {
public:
    /**
     * @brief Construct a new MA600A object
     *
     * @param spi_host SPI host device (e.g., SPI2_HOST)
     * @param sclk_io SPI clock pin
     * @param miso_io SPI MISO pin
     * @param mosi_io SPI MOSI pin
     * @param cs_io Chip select pin
     */
    MA600A(spi_host_device_t spi_host, gpio_num_t sclk_io, gpio_num_t miso_io, gpio_num_t mosi_io, gpio_num_t cs_io);
    
    /**
     * @brief Destroy the MA600A object
     */
    ~MA600A();
    
    /**
     * @brief Initialize SPI for MA600A
     */
    void init();
    
    /**
     * @brief Deinitialize SPI for MA600A
     */
    void deinit();
    
    /**
     * @brief Get the sensor angle in radians
     * 
     * @return float Current shaft angle in radians
     */
    float getSensorAngle() override;

private:
    /**
     * @brief Read raw position value from sensor
     * 
     * @return uint16_t Raw angle value (0-65535)
     */
    uint16_t readRawPosition();
    
    spi_host_device_t _spi_host;
    gpio_num_t _sclk_io;
    gpio_num_t _miso_io;
    gpio_num_t _mosi_io;
    gpio_num_t _cs_io;
    spi_device_handle_t _spi_device;
    bool _is_installed;
    
    static constexpr float COUNTS_TO_RADIANS = 2 * PI / 65536.0f;   // Conversion factor
};