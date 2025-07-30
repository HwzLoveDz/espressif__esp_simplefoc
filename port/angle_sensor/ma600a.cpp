/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ma600a.h"
#include "esp_log.h"
#include "esp_check.h"

static const char *TAG = "MA600A";

MA600A::MA600A(spi_host_device_t spi_host, gpio_num_t sclk_io, gpio_num_t miso_io, 
               gpio_num_t mosi_io, gpio_num_t cs_io) {
    _spi_host = spi_host;
    _sclk_io = sclk_io;
    _miso_io = miso_io;
    _mosi_io = mosi_io;
    _cs_io = cs_io;
    _is_installed = false;
}

MA600A::~MA600A() {
    if (_is_installed) {
        deinit();
    }
}

void MA600A::deinit() {
    esp_err_t ret;
    ret = spi_bus_remove_device(_spi_device);
    ESP_RETURN_ON_FALSE(ret == ESP_OK, , TAG, "SPI remove device fail");
    ret = spi_bus_free(_spi_host);
    ESP_RETURN_ON_FALSE(ret == ESP_OK, , TAG, "SPI free fail");
    _is_installed = false;
}

void MA600A::init() {
    esp_err_t ret;

    // Configuration for the SPI bus
    spi_bus_config_t buscfg = {
        .mosi_io_num = _mosi_io,
        .miso_io_num = _miso_io,
        .sclk_io_num = _sclk_io,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 1000,
    };
    
    ret = spi_bus_initialize(_spi_host, &buscfg, SPI_DMA_CH_AUTO);
    ESP_RETURN_ON_FALSE(ret == ESP_OK, , TAG, "SPI bus init fail");

    // Device configuration (SPI Mode 3: CPOL=1, CPHA=1)
    spi_device_interface_config_t devcfg = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .mode = 3,  // SPI Mode 3
        .cs_ena_pretrans = 1,
        .clock_speed_hz = 24 * 1000 * 1000, // MAX 24MHz
        .spics_io_num = _cs_io,
        .queue_size = 1,
    };

    ret = spi_bus_add_device(_spi_host, &devcfg, &_spi_device);
    ESP_RETURN_ON_FALSE(ret == ESP_OK, , TAG, "SPI bus add device fail");

    _is_installed = true;
    ESP_LOGI(TAG, "Initialized successfully");
}

uint16_t MA600A::readRawPosition() {
    if (!_is_installed) {
        ESP_LOGE(TAG, "SPI not initialized");
        return 0;
    }

    // SPI transaction for reading angle (command: 0x0000)
    uint8_t tx_buffer[2] = {0x00, 0x00}; // Command for angle reading
    uint8_t rx_buffer[2] = {0};
    
    spi_transaction_t t = {};
    t.length = 16; // 16 bits
    t.tx_buffer = tx_buffer;
    t.rx_buffer = rx_buffer;
    
    esp_err_t ret = spi_device_polling_transmit(_spi_device, &t);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI transmit failed: %s", esp_err_to_name(ret));
        return 0;
    }
    
    // Combine received bytes into 16-bit value
    return (rx_buffer[0] << 8) | rx_buffer[1];
}

float MA600A::getSensorAngle() {
    uint16_t raw_angle = readRawPosition();
    float angle = raw_angle * COUNTS_TO_RADIANS;    // Converts raw data into angle information in radians.
    return angle;
}