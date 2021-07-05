/*
Copyright (c) 2018-2019 Mika Tuupola
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include "software_i2c.h"

#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_err.h>
#include <esp_log.h>

#define LOW   0x00
#define HIGH  0x01

#define ACK   0x00
#define NAK   0x01

#define CLOCK_STRETCH_TIMEOUT   1000

static const char* TAG = "software_i2c";

/* https://esp-idf.readthedocs.io/en/latest/api-reference/peripherals/i2c.html#_CPPv211i2c_set_pin10i2c_port_tii13gpio_pullup_t13gpio_pullup_t10i2c_mode_t */

/* esp_err_t i2c_set_pin(i2c_port_t i2c_num, int sda_io_num, int scl_io_num, gpio_pullup_t sda_pullup_en, gpio_pullup_t scl_pullup_en, i2c_mode_t mode) */
esp_err_t sw_i2c_init(sw_i2c_handle* handle)
{
    ESP_LOGD(TAG, "Initializing software i2c with data pin %d.", handle->sda);
    gpio_set_direction(handle->sda, GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_set_pull_mode(handle->sda, GPIO_FLOATING);

    ESP_LOGD(TAG, "Initializing software i2c with clock pin %d.", handle->scl);
    gpio_set_direction(handle->scl, GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_set_pull_mode(handle->scl, GPIO_FLOATING);

    return ESP_OK;
}

/* esp_err_t i2c_master_start(i2c_cmd_handle_t cmd_handle) */
esp_err_t sw_i2c_master_start(sw_i2c_handle* handle)
{
    uint32_t stretch = CLOCK_STRETCH_TIMEOUT;

     /* If already started, do a restart condition. */
    if (handle->started) {
        gpio_set_level(handle->sda, HIGH);
        ets_delay_us(10);
        gpio_set_level(handle->scl, HIGH);
        while (gpio_get_level(handle->scl) == LOW && stretch--) {
            ets_delay_us(1);
        };
        ets_delay_us(10);
    }

    if (LOW == gpio_get_level(handle->sda)) {
        ESP_LOGD(TAG, "Arbitration lost in sw_i2c_master_start()");
    }

    /* Start bit is indicated by a high-to-low transition of SDA with SCL high. */
    gpio_set_level(handle->sda, LOW);
    ets_delay_us(10);
    gpio_set_level(handle->scl, LOW);

    handle->started = true;

    return ESP_OK;
}

/* esp_err_t i2c_master_stop(i2c_cmd_handle_t cmd_handle) */
esp_err_t sw_i2c_master_stop(sw_i2c_handle* handle)
{
    uint32_t stretch = CLOCK_STRETCH_TIMEOUT;

    /* The stop bit is indicated by a low-to-high transition of SDA with SCL high. */
    gpio_set_level(handle->sda, LOW);
    ets_delay_us(10);
    gpio_set_level(handle->scl, HIGH);

    while (gpio_get_level(handle->scl) == LOW && stretch--) {
        ets_delay_us(1);
    };

    ets_delay_us(10);
    gpio_set_level(handle->sda, HIGH);
    ets_delay_us(10);

    if (gpio_get_level(handle->sda) == LOW) {
        ESP_LOGD(TAG, "Arbitration lost in sw_i2c_master_stop()");
    }

    handle->started = false;

    return ESP_OK;
}

static void sw_i2c_write_bit(sw_i2c_handle* handle, bool bit)
{
    uint32_t stretch = CLOCK_STRETCH_TIMEOUT;

    gpio_set_level(handle->sda, bit);
    ets_delay_us(10); /* SDA change propagation delay */
    gpio_set_level(handle->scl, HIGH); /* New valid SDA value is available. */

    while (gpio_get_level(handle->scl) == LOW && stretch--) {
        ets_delay_us(1);
    };

    ets_delay_us(10); /* Wait for SDA value to be read by slave. */

    if (bit && (LOW == gpio_get_level(handle->sda))) {
        ESP_LOGD(TAG, "Arbitration lost in sw_i2c_write_bit()");
    }

    gpio_set_level(handle->scl, LOW); /* Prepare for next bit. */
}

static bool sw_i2c_read_bit(sw_i2c_handle* handle)
{
    uint32_t stretch = CLOCK_STRETCH_TIMEOUT;
    bool bit;

    gpio_set_level(handle->sda, HIGH); /* Let the slave drive data. */
    ets_delay_us(10); /* Wait for slave to write. */
    gpio_set_level(handle->scl, HIGH); /* New valid SDA value is available. */

    while (gpio_get_level(handle->scl) == LOW && stretch--) {
        ets_delay_us(1);
    };

    ets_delay_us(10); /* Wait for slave to write. */
    bit = gpio_get_level(handle->sda); /* SCL is high, read a bit. */
    gpio_set_level(handle->scl, LOW); /* Prepare for next bit. */

    return bit;
}

static uint8_t sw_i2c_read_byte(sw_i2c_handle* handle, bool ack)
{
    uint8_t byte = 0;
    uint8_t bit;

    for (bit = 0; bit < 8; ++bit) {
        byte = (byte << 1) | sw_i2c_read_bit(handle);
    }
    sw_i2c_write_bit(handle, ack);

    return byte;
}

static bool sw_i2c_write_byte(sw_i2c_handle* handle, uint8_t byte)
{
    uint8_t bit;
    bool ack;

    for (bit = 0; bit < 8; ++bit) {
        sw_i2c_write_bit(handle, (byte & 0x80) != 0);
        byte <<= 1;
    }
    ack = sw_i2c_read_bit(handle);
    return ack;
}

/* esp_err_t i2c_master_write_byte(i2c_cmd_handle_t cmd_handle, uint8_t data, bool ack_en) */
esp_err_t sw_i2c_master_write_byte(sw_i2c_handle* handle, uint8_t buffer)
{
    return sw_i2c_write_byte(handle, buffer);
    //return ESP_OK;
}

/* esp_err_t i2c_master_write(i2c_cmd_handle_t cmd_handle, uint8_t *data, size_t data_len, bool ack_en) */
esp_err_t sw_i2c_master_write(sw_i2c_handle* handle, uint8_t *buffer, uint8_t length) // bool ack_enable??
{
    while (length--) {
        sw_i2c_write_byte(handle, *buffer++);
    }

    return ESP_OK;
}

/* esp_err_t i2c_master_read_byte(i2c_cmd_handle_t cmd_handle, uint8_t *data, i2c_ack_type_t ack) */
esp_err_t sw_i2c_master_read_byte(sw_i2c_handle* handle, uint8_t *buffer, bool ack)
{
    *buffer = sw_i2c_read_byte(handle, ack);
    return ESP_OK;
};

/* esp_err_t i2c_master_read(i2c_cmd_handle_t cmd_handle, uint8_t *data, size_t data_len, i2c_ack_type_t ack) */
esp_err_t sw_i2c_master_read(sw_i2c_handle* handle, uint8_t *buffer, uint16_t length, bool ack)
{
    while(length) {
        *buffer = sw_i2c_read_byte(handle, ack);
        buffer++;
        length--;
    }

    return ESP_OK;
}
