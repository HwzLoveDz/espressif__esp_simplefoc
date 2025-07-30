#ifndef ESP_HAL_USB_SERIAL_H
#define ESP_HAL_USB_SERIAL_H

#include <driver/uart.h>
#include <driver/usb_serial_jtag.h>
#include "esp_app_print.h"
#include "esp_err.h"
#include "esp_log.h"
#include <mutex>  // 添加互斥锁支持

class USBSerial : public Stream {
public:
    USBSerial();
    ~USBSerial();

    bool begin(unsigned long baud = 0, 
               uart_port_t uart_num = (uart_port_t)-1,
               int tx_io = -1, 
               int rx_io = -1, 
               uart_word_length_t wordLength = UART_DATA_8_BITS, 
               uart_parity_t parity = UART_PARITY_DISABLE, 
               uart_stop_bits_t stopBits = UART_STOP_BITS_1);
    
    void end();
    int available() override;
    int peek();
    size_t write(uint8_t c) override;
    size_t write(const uint8_t *buffer, size_t size) override;
    int read() override;
    size_t read(uint8_t *buffer, size_t size);
    size_t readBytes(uint8_t *buffer, size_t length) override;
    void flush();

    using Print::write;
    size_t write(const char *str) { return Print::write(str); }
    size_t write(const char *buffer, size_t size) { return Print::write(buffer, size); }
    
    size_t write(unsigned long n) { return write((uint8_t)n); }
    size_t write(long n) { return write((uint8_t)n); }
    size_t write(unsigned int n) { return write((uint8_t)n); }
    size_t write(int n) { return write((uint8_t)n); }

    bool isConnected() const { return _usb_installed && _usb_ready; }  // 添加就绪状态检查

private:
    uint32_t _rxBufferSize;
    uint32_t _txBufferSize;
    bool _usb_installed;
    bool _usb_ready;  // 添加USB连接就绪标志
    std::mutex _usb_mutex;  // 添加互斥锁保护USB操作
};

extern USBSerial SerialUSB;

#endif