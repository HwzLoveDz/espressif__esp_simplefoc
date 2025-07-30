#include "esp_hal_usb_serial.h"
#include "driver/usb_serial_jtag.h"
#include <cstring>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

USBSerial SerialUSB;

#define USB_SERIAL_BUF_SIZE 256 * 2
#define USB_READ_TIMEOUT_MS 100

USBSerial::USBSerial() {
    _rxBufferSize = USB_SERIAL_BUF_SIZE;
    _txBufferSize = USB_SERIAL_BUF_SIZE;
    _usb_installed = false;
    _usb_ready = false;  // 初始化为未就绪状态
    setTimeout(100);
}

USBSerial::~USBSerial() {
    end();
}

bool USBSerial::begin(unsigned long baud, uart_port_t uart_num, 
                    int tx_io, int rx_io, 
                    uart_word_length_t wordLength, 
                    uart_parity_t parity, 
                    uart_stop_bits_t stopBits) {
    (void)uart_num;
    (void)tx_io;
    (void)rx_io;
    (void)wordLength;
    (void)parity;
    (void)stopBits;
    (void)baud;

    if (_usb_installed) {
        ESP_LOGW("USBSerial", "USB already initialized");
        return true;
    }

    usb_serial_jtag_driver_config_t cfg = {
        .tx_buffer_size = _txBufferSize,
        .rx_buffer_size = _rxBufferSize
    };
    
    esp_err_t ret = usb_serial_jtag_driver_install(&cfg);
    if (ret == ESP_OK) {
        _usb_installed = true;
        
        // 等待USB连接就绪
        int retries = 50;
        while (retries-- > 0 && !usb_serial_jtag_is_connected()) {
            vTaskDelay(pdMS_TO_TICKS(20));
        }
        
        if (usb_serial_jtag_is_connected()) {
            _usb_ready = true;
            ESP_LOGI("USBSerial", "USB Serial JTAG initialized and connected");
        } else {
            ESP_LOGW("USBSerial", "USB initialized but not connected");
            _usb_ready = false;
        }
        
        return true;
    } else {
        ESP_LOGE("USBSerial", "USB init failed: %s", esp_err_to_name(ret));
        _usb_installed = false;
        _usb_ready = false;
        return false;
    }
}

void USBSerial::end() {
    std::lock_guard<std::mutex> lock(_usb_mutex);
    if (_usb_installed) {
        usb_serial_jtag_driver_uninstall();
        _usb_installed = false;
        _usb_ready = false;
        ESP_LOGI("USBSerial", "USB Serial JTAG uninstalled");
    }
}

int USBSerial::available() {
    if (!_usb_installed) return 0;
    
    // Use the universal available function
    // return usb_serial_jtag_available();
    return -1;
}

int USBSerial::peek() {
    return -1;
}

size_t USBSerial::write(uint8_t c) {
    return write(&c, 1);
}

size_t USBSerial::write(const uint8_t *buffer, size_t size) {
    if (!_usb_ready || buffer == nullptr || size == 0) {
        return 0;
    }
    
    std::lock_guard<std::mutex> lock(_usb_mutex);
    size_t bytes_written = 0;
    const TickType_t timeout_ticks = pdMS_TO_TICKS(50);
    
    while (bytes_written < size) {
        // 检查USB连接状态
        if (!usb_serial_jtag_is_connected()) {
            _usb_ready = false;
            ESP_LOGW("USBSerial", "USB disconnected during write");
            break;
        }
        
        // 计算本次写入大小 (限制最大块大小为64字节)
        size_t to_write = size - bytes_written;
        if (to_write > 64) to_write = 64;
        
        // 执行写入
        esp_err_t ret = usb_serial_jtag_write_bytes(
            buffer + bytes_written, 
            to_write, 
            timeout_ticks
        );
        
        if (ret == ESP_OK) {
            bytes_written += to_write;
        } else {
            ESP_LOGE("USBSerial", "Write error: %s (0x%x)", esp_err_to_name(ret), ret);
            
            // 处理特定错误状态
            if (ret == ESP_ERR_INVALID_STATE || ret == ESP_ERR_INVALID_ARG) {
                // 短暂延迟后重试
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }
            
            // 其他错误直接退出循环
            break;
        }
    }
    
    if (bytes_written < size) {
        ESP_LOGW("USBSerial", "Partial write: %d/%d bytes", bytes_written, size);
    }
    
    return bytes_written;
}

int USBSerial::read() {
    uint8_t c;
    return (read(&c, 1) == 1) ? c : -1;
}

size_t USBSerial::read(uint8_t *buffer, size_t size) {
    if (!_usb_installed || buffer == nullptr || size == 0) {
        return 0;
    }
    
    // Universal read function
    int bytes_read = usb_serial_jtag_read_bytes(buffer, size, pdMS_TO_TICKS(USB_READ_TIMEOUT_MS));
    if (bytes_read < 0) {
        ESP_LOGE("USBSerial", "Read failed: returned %d", bytes_read);
        return 0;
    }
    return static_cast<size_t>(bytes_read);
}

size_t USBSerial::readBytes(uint8_t *buffer, size_t length) {
    return Stream::readBytes(buffer, length);
}

void USBSerial::flush() {
    if (_usb_installed) {
        // Universal flush function
        esp_err_t ret;
        int retries = 10;
        while (retries-- > 0) {
            ret = usb_serial_jtag_wait_tx_done(0);
            if (ret == ESP_OK) {
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        if (ret != ESP_OK) {
            ESP_LOGE("USBSerial", "Flush failed: %s", esp_err_to_name(ret));
        }
    }
}