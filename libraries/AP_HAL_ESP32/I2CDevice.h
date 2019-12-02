/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include <inttypes.h>

#include <AP_HAL/HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_HAL/utility/OwnPtr.h>

#include "Semaphores.h"
#include "Scheduler.h"
#include "DeviceBus.h"

#ifdef __cplusplus
extern "C" {
#endif

#include <esp_types.h>
#include "esp_err.h"
#include "esp_intr_alloc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/xtensa_api.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/ringbuf.h"
#include "driver/gpio.h"

#define I2C_APB_CLK_FREQ  APB_CLK_FREQ /*!< I2C source clock is APB clock, 80MHz */
#define I2C_FIFO_LEN   (32)  /*!< I2C hardware fifo length */
typedef enum{
    I2C_MODE_SLAVE = 0,   /*!< I2C slave mode */
    I2C_MODE_MASTER,      /*!< I2C master mode */
    I2C_MODE_MAX,
}i2c_mode_t;

typedef enum {
    I2C_MASTER_WRITE = 0,   /*!< I2C write data */
    I2C_MASTER_READ,        /*!< I2C read data */
} i2c_rw_t;

typedef enum {
    I2C_DATA_MODE_MSB_FIRST = 0,  /*!< I2C data msb first */
    I2C_DATA_MODE_LSB_FIRST = 1,  /*!< I2C data lsb first */
    I2C_DATA_MODE_MAX
} i2c_trans_mode_t;

typedef enum{
    I2C_CMD_RESTART = 0,   /*!<I2C restart command */
    I2C_CMD_WRITE,         /*!<I2C write command */
    I2C_CMD_READ,          /*!<I2C read command */
    I2C_CMD_STOP,          /*!<I2C stop command */
    I2C_CMD_END            /*!<I2C end command */
}i2c_opmode_t;

typedef enum{
    I2C_NUM_0 = 0,  /*!< I2C port 0 */
    I2C_NUM_1 ,     /*!< I2C port 1 */
    I2C_NUM_MAX
} i2c_port_t;

typedef enum {
    I2C_ADDR_BIT_7 = 0,    /*!< I2C 7bit address for slave mode */
    I2C_ADDR_BIT_10,       /*!< I2C 10bit address for slave mode */
    I2C_ADDR_BIT_MAX,
} i2c_addr_mode_t;

typedef enum {
    I2C_MASTER_ACK = 0x0,        /*!< I2C ack for each byte read */
    I2C_MASTER_NACK = 0x1,       /*!< I2C nack for each byte read */
    I2C_MASTER_LAST_NACK = 0x2,   /*!< I2C nack for the last byte*/
    I2C_MASTER_ACK_MAX,
} i2c_ack_type_t;

/**
 * @brief I2C initialization parameters
 */
typedef struct{
    i2c_mode_t mode;       /*!< I2C mode */
    gpio_num_t sda_io_num;        /*!< GPIO number for I2C sda signal */
    gpio_pullup_t sda_pullup_en;  /*!< Internal GPIO pull mode for I2C sda signal*/
    gpio_num_t scl_io_num;        /*!< GPIO number for I2C scl signal */
    gpio_pullup_t scl_pullup_en;  /*!< Internal GPIO pull mode for I2C scl signal*/

    union {
        struct {
            uint32_t clk_speed;     /*!< I2C clock frequency for master mode, (no higher than 1MHz for now) */
        } master;
        struct {
            uint8_t addr_10bit_en;  /*!< I2C 10bit address mode enable for slave mode */
            uint16_t slave_addr;    /*!< I2C address for slave mode */
        } slave;

    };
}i2c_config_t;

typedef struct {
    uint8_t byte_num;  /*!< cmd byte number */
    uint8_t ack_en;    /*!< ack check enable */
    uint8_t ack_exp;   /*!< expected ack level to get */
    uint8_t ack_val;   /*!< ack value to send */
    uint8_t* data;     /*!< data address */
    uint8_t byte_cmd;  /*!< to save cmd for one byte command mode */
    i2c_opmode_t op_code; /*!< haredware cmd type */
}i2c_cmd_t;

/**
 * @brief I2C driver install
 *
 * @param i2c_num I2C port number
 * @param mode I2C mode( master or slave )
 * @param slv_rx_buf_len receiving buffer size for slave mode
 *        @note
 *        Only slave mode will use this value, driver will ignore this value in master mode.
 * @param slv_tx_buf_len sending buffer size for slave mode
 *        @note
 *        Only slave mode will use this value, driver will ignore this value in master mode.
 * @param intr_alloc_flags Flags used to allocate the interrupt. One or multiple (ORred)
 *            ESP_INTR_FLAG_* values. See esp_intr_alloc.h for more info.
 *        @note
 *        In master mode, if the cache is likely to be disabled(such as write flash) and the slave is time-sensitive,
 *        `ESP_INTR_FLAG_IRAM` is suggested to be used. In this case, please use the memory allocated from internal RAM in i2c read and write function,
 *        because we can not access the psram(if psram is enabled) in interrupt handle function when cache is disabled.
 *
 * @return
 *     - ESP_OK   Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_FAIL Driver install error
 */
esp_err_t i2c_driver_install(i2c_port_t i2c_num, int intr_alloc_flags);

/**
 * @brief I2C driver delete
 *
 * @param i2c_num I2C port number
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t i2c_driver_delete(i2c_port_t i2c_num);

/**
 * @brief I2C parameter initialization
 *
 * @param i2c_num I2C port number
 * @param i2c_conf pointer to I2C parameter settings
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t i2c_param_config(i2c_port_t i2c_num, const i2c_config_t* i2c_conf);

/**
 * @brief reset I2C tx hardware fifo
 *
 * @param i2c_num I2C port number
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t i2c_reset_tx_fifo(i2c_port_t i2c_num);

/**
 * @brief reset I2C rx fifo
 *
 * @param i2c_num I2C port number
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t i2c_reset_rx_fifo(i2c_port_t i2c_num);

/**
 * @brief I2C isr handler register
 *
 * @param i2c_num I2C port number
 * @param fn isr handler function
 * @param arg parameter for isr handler function
 * @param intr_alloc_flags Flags used to allocate the interrupt. One or multiple (ORred)
 *            ESP_INTR_FLAG_* values. See esp_intr_alloc.h for more info.
 * @param handle handle return from esp_intr_alloc.
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t i2c_isr_register(i2c_port_t i2c_num, void (*fn)(void*), void * arg, int intr_alloc_flags, intr_handle_t *handle);

/**
 * @brief to delete and free I2C isr.
 *
 * @param handle handle of isr.
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t i2c_isr_free(intr_handle_t handle);

/**
 * @brief Configure GPIO signal for I2C sck and sda
 *
 * @param i2c_num I2C port number
 * @param sda_io_num GPIO number for I2C sda signal
 * @param scl_io_num GPIO number for I2C scl signal
 * @param sda_pullup_en Whether to enable the internal pullup for sda pin
 * @param scl_pullup_en Whether to enable the internal pullup for scl pin
 * @param mode I2C mode
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t i2c_set_pin(i2c_port_t i2c_num, int sda_io_num, int scl_io_num,
                      gpio_pullup_t sda_pullup_en, gpio_pullup_t scl_pullup_en);

/**
 * @brief Queue command for I2C master to generate a start signal
 *        @note
 *        Only call this function in I2C master mode
 *        Call i2c_master_cmd_begin() to send all queued commands
 *
 * @param cmd_handle I2C cmd link
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t i2c_master_start(i2c_cmd_t* cmd, size_t* n);

/**
 * @brief Queue command for I2C master to write one byte to I2C bus
 *        @note
 *        Only call this function in I2C master mode
 *        Call i2c_master_cmd_begin() to send all queued commands
 *
 * @param cmd_handle I2C cmd link
 * @param data I2C one byte command to write to bus
 * @param ack_en enable ack check for master
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t i2c_master_write_byte(i2c_cmd_t* cmd, size_t* n, uint8_t data, bool ack_en);

/**
 * @brief Queue command for I2C master to write buffer to I2C bus
 *        @note
 *        Only call this function in I2C master mode
 *        Call i2c_master_cmd_begin() to send all queued commands
 *
 * @param cmd_handle I2C cmd link
 * @param data data to send
 *        @note
 *        If the psram is enabled and intr_flag is `ESP_INTR_FLAG_IRAM`, please use the memory allocated from internal RAM.
 * @param data_len data length
 * @param ack_en enable ack check for master
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t i2c_master_write(i2c_cmd_t* cmd, size_t* n, uint8_t* data, size_t data_len, bool ack_en);

/**
 * @brief Queue command for I2C master to read one byte from I2C bus
 *        @note
 *        Only call this function in I2C master mode
 *        Call i2c_master_cmd_begin() to send all queued commands
 *
 * @param cmd_handle I2C cmd link
 * @param data pointer accept the data byte
 *        @note
 *        If the psram is enabled and intr_flag is `ESP_INTR_FLAG_IRAM`, please use the memory allocated from internal RAM.
 * @param ack ack value for read command
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t i2c_master_read_byte(i2c_cmd_t* cmd_handle, size_t* n, uint8_t* data, i2c_ack_type_t ack);

/**
 * @brief Queue command for I2C master to read data from I2C bus
 *        @note
 *        Only call this function in I2C master mode
 *        Call i2c_master_cmd_begin() to send all queued commands
 *
 * @param cmd_handle I2C cmd link
 * @param data data buffer to accept the data from bus
 *        @note
 *        If the psram is enabled and intr_flag is `ESP_INTR_FLAG_IRAM`, please use the memory allocated from internal RAM.
 * @param data_len read data length
 * @param ack ack value for read command
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t i2c_master_read(i2c_cmd_t* cmd_handle, size_t* n, uint8_t* data, size_t data_len, i2c_ack_type_t ack);

/**
 * @brief Queue command for I2C master to generate a stop signal
 *        @note
 *        Only call this function in I2C master mode
 *        Call i2c_master_cmd_begin() to send all queued commands
 *
 * @param cmd_handle I2C cmd link
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t i2c_master_stop(i2c_cmd_t* cmd, size_t* n);

/**
 * @brief I2C master send queued commands.
 *        This function will trigger sending all queued commands.
 *        The task will be blocked until all the commands have been sent out.
 *        The I2C APIs are not thread-safe, if you want to use one I2C port in different tasks,
 *        you need to take care of the multi-thread issue.
 *        @note
 *        Only call this function in I2C master mode
 *
 * @param i2c_num I2C port number
 * @param cmd_handle I2C command handler
 * @param ticks_to_wait maximum wait ticks.
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_FAIL Sending command error, slave doesn't ACK the transfer.
 *     - ESP_ERR_INVALID_STATE I2C driver not installed or not in master mode.
 *     - ESP_ERR_TIMEOUT Operation timeout because the bus is busy.
 */
esp_err_t IRAM_ATTR i2c_master_cmd_begin(i2c_port_t i2c_num, i2c_cmd_t* cmd, size_t n, TickType_t ticks_to_wait);

/**
 * @brief set I2C master clock period
 *
 * @param i2c_num I2C port number
 * @param high_period clock cycle number during SCL is high level, high_period is a 14 bit value
 * @param low_period clock cycle number during SCL is low level, low_period is a 14 bit value
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t i2c_set_period(i2c_port_t i2c_num, int high_period, int low_period);

/**
 * @brief get I2C master clock period
 *
 * @param i2c_num I2C port number
 * @param high_period pointer to get clock cycle number during SCL is high level, will get a 14 bit value
 * @param low_period pointer to get clock cycle number during SCL is low level, will get a 14 bit value
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t i2c_get_period(i2c_port_t i2c_num, int* high_period, int* low_period);

/**
 * @brief enable hardware filter on I2C bus
 *        Sometimes the I2C bus is disturbed by high frequency noise(about 20ns), or the rising edge of
 *        the SCL clock is very slow, these may cause the master state machine broken. enable hardware
 *        filter can filter out high frequency interference and make the master more stable.
 *        @note
 *        Enable filter will slow the SCL clock.
 *
 * @param i2c_num I2C port number
 * @param cyc_num the APB cycles need to be filtered(0<= cyc_num <=7).
 *        When the period of a pulse is less than cyc_num * APB_cycle, the I2C controller will ignore this pulse.
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t i2c_filter_enable(i2c_port_t i2c_num, uint8_t cyc_num);

/**
 * @brief disable filter on I2C bus
 *
 * @param i2c_num I2C port number
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t i2c_filter_disable(i2c_port_t i2c_num);

/**
 * @brief set I2C master start signal timing
 *
 * @param i2c_num I2C port number
 * @param setup_time clock number between the falling-edge of SDA and rising-edge of SCL for start mark, it's a 10-bit value.
 * @param hold_time clock num between the falling-edge of SDA and falling-edge of SCL for start mark, it's a 10-bit value.
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t i2c_set_start_timing(i2c_port_t i2c_num, int setup_time, int hold_time);

/**
 * @brief get I2C master start signal timing
 *
 * @param i2c_num I2C port number
 * @param setup_time pointer to get setup time
 * @param hold_time pointer to get hold time
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t i2c_get_start_timing(i2c_port_t i2c_num, int* setup_time, int* hold_time);

/**
 * @brief set I2C master stop signal timing
 *
 * @param i2c_num I2C port number
 * @param setup_time clock num between the rising-edge of SCL and the rising-edge of SDA, it's a 10-bit value.
 * @param hold_time clock number after the STOP bit's rising-edge, it's a 14-bit value.
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t i2c_set_stop_timing(i2c_port_t i2c_num, int setup_time, int hold_time);

/**
 * @brief get I2C master stop signal timing
 *
 * @param i2c_num I2C port number
 * @param setup_time pointer to get setup time.
 * @param hold_time pointer to get hold time.
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t i2c_get_stop_timing(i2c_port_t i2c_num, int* setup_time, int* hold_time);

/**
 * @brief set I2C data signal timing
 *
 * @param i2c_num I2C port number
 * @param sample_time clock number I2C used to sample data on SDA after the rising-edge of SCL, it's a 10-bit value
 * @param hold_time clock number I2C used to hold the data after the falling-edge of SCL, it's a 10-bit value
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t i2c_set_data_timing(i2c_port_t i2c_num, int sample_time, int hold_time);

/**
 * @brief get I2C data signal timing
 *
 * @param i2c_num I2C port number
 * @param sample_time pointer to get sample time
 * @param hold_time pointer to get hold time
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t i2c_get_data_timing(i2c_port_t i2c_num, int* sample_time, int* hold_time);

/**
 * @brief set I2C timeout value
 * @param i2c_num I2C port number
 * @param timeout timeout value for I2C bus (unit: APB 80Mhz clock cycle)
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t i2c_set_timeout(i2c_port_t i2c_num, int timeout);

/**
 * @brief get I2C timeout value
 * @param i2c_num I2C port number
 * @param timeout pointer to get timeout value
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t i2c_get_timeout(i2c_port_t i2c_num, int* timeout);
/**
 * @brief set I2C data transfer mode
 *
 * @param i2c_num I2C port number
 * @param tx_trans_mode I2C sending data mode
 * @param rx_trans_mode I2C receving data mode
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t i2c_set_data_mode(i2c_port_t i2c_num, i2c_trans_mode_t tx_trans_mode, i2c_trans_mode_t rx_trans_mode);

/**
 * @brief get I2C data transfer mode
 *
 * @param i2c_num I2C port number
 * @param tx_trans_mode pointer to get I2C sending data mode
 * @param rx_trans_mode pointer to get I2C receiving data mode
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t i2c_get_data_mode(i2c_port_t i2c_num, i2c_trans_mode_t *tx_trans_mode, i2c_trans_mode_t *rx_trans_mode);

#ifdef __cplusplus
}
#endif


namespace ESP32 {

struct I2CBusDesc {
    i2c_port_t port;
    gpio_num_t sda;
    gpio_num_t scl;
    uint32_t speed;
    bool internal;
};

class I2CBus : public  DeviceBus {
public:
    I2CBus():DeviceBus(Scheduler::I2C_PRIORITY) {};
    i2c_port_t port;
	uint32_t bus_clock;
};

class I2CDevice : public AP_HAL::I2CDevice {
public:
    static I2CDevice *from(AP_HAL::I2CDevice *dev)
    {
        return static_cast<I2CDevice*>(dev);
    }

    I2CDevice(uint8_t bus, uint8_t address, uint32_t bus_clock, bool use_smbus, uint32_t timeout_ms);
    ~I2CDevice();

    /* See AP_HAL::I2CDevice::set_address() */
    void set_address(uint8_t address) override
    {
        _address = address;
    }

    /* See AP_HAL::I2CDevice::set_retries() */
    void set_retries(uint8_t retries) override
    {
        _retries = retries;
    }

    /* See AP_HAL::Device::set_speed(): Empty implementation, not supported. */
    bool set_speed(enum Device::Speed speed) override
    {
        return true;
    }

    /* See AP_HAL::Device::transfer() */
    bool transfer(const uint8_t *send, uint32_t send_len,
                  uint8_t *recv, uint32_t recv_len) override;

    bool read_registers_multiple(uint8_t first_reg, uint8_t *recv,
                                 uint32_t recv_len, uint8_t times) override
    {
        return false;
    };

    /* See AP_HAL::Device::register_periodic_callback() */
    AP_HAL::Device::PeriodicHandle register_periodic_callback(
        uint32_t period_usec, AP_HAL::Device::PeriodicCb) override;

    /* See AP_HAL::Device::adjust_periodic_callback() */
    bool adjust_periodic_callback(AP_HAL::Device::PeriodicHandle h, uint32_t period_usec) override;

    AP_HAL::Semaphore* get_semaphore() override
    {
        // if asking for invalid bus number use bus 0 semaphore
        return &bus.semaphore;
    }

private:
    I2CBus &bus;
    uint8_t _retries;
    uint8_t _address;
    char *pname;

};

class I2CDeviceManager : public AP_HAL::I2CDeviceManager {
public:
    friend class I2CDevice;

    static I2CBus businfo[];

    // constructor
    I2CDeviceManager();

    static I2CDeviceManager *from(AP_HAL::I2CDeviceManager *i2c_mgr)
    {
        return static_cast<I2CDeviceManager*>(i2c_mgr);
    }

    AP_HAL::OwnPtr<AP_HAL::I2CDevice> get_device(uint8_t bus, uint8_t address,
            uint32_t bus_clock=400000,
            bool use_smbus = false,
            uint32_t timeout_ms=4) override;

    /*
      get mask of bus numbers for all configured I2C buses
     */
    uint32_t get_bus_mask(void) const override;

    /*
      get mask of bus numbers for all configured external I2C buses
     */
    uint32_t get_bus_mask_external(void) const override;

    /*
      get mask of bus numbers for all configured internal I2C buses
     */
    uint32_t get_bus_mask_internal(void) const override;
};
}
