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
#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32 && !defined(HAL_NO_UARTDRIVER)
#include "UARTDriver.h"
#include "GPIO.h"
#include <AP_Math/AP_Math.h>
#include <AP_InternalError/AP_InternalError.h>
#include <AP_Common/ExpandingString.h>
#include "Scheduler.h"

#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/uart.h"

extern const AP_HAL::HAL& hal;

using namespace ESP32;

UARTDesc uart_desc[] = {HAL_ESP32_UART_DEVICES};

const UARTDriver::SerialDef UARTDriver::_serial_tab[] = { HAL_UART_DEVICE_LIST };

// handle for UART handling thread
void* volatile UARTDriver::uart_rx_thread_ctx;

// table to find UARTDrivers from serial number, used for event handling
UARTDriver *UARTDriver::uart_drivers[UART_MAX_DRIVERS];

uint32_t UARTDriver::_last_stats_ms;

#ifndef HAL_UART_MIN_TX_SIZE
#define HAL_UART_MIN_TX_SIZE 512
#endif

#ifndef HAL_UART_MIN_RX_SIZE
#define HAL_UART_MIN_RX_SIZE 512
#endif

#ifndef HAL_UART_STACK_SIZE
#define HAL_UART_STACK_SIZE 320
#endif

#ifndef HAL_UART_RX_STACK_SIZE
#define HAL_UART_RX_STACK_SIZE 768
#endif

UARTDriver::UARTDriver(uint8_t _serial_num) :
serial_num(_serial_num),
sdef(_serial_tab[_serial_num]),
_baudrate(57600)
{
    configASSERT(serial_num < UART_MAX_DRIVERS, "too many UART drivers");
    uart_drivers[serial_num] = this;
}

/*
  thread for handling UART send/receive

  We use events indexed by serial_num to trigger a more rapid send for
  unbuffered_write uarts, and run at 1kHz for general UART handling
 */
#pragma GCC diagnostic push
#pragma GCC diagnostic error "-Wframe-larger-than=128"
void UARTDriver::uart_thread()
{
    uint32_t last_thread_run_us = 0; // last time we did a 1kHz run of this uart

    while (uart_thread_ctx == nullptr) {
        hal.scheduler->delay_microseconds(1000);
    }

    while (true) {
        eventmask_t mask = chEvtWaitAnyTimeout(EVT_TRANSMIT_DATA_READY | EVT_TRANSMIT_END | EVT_TRANSMIT_UNBUFFERED, chTimeMS2I(1));
        uint32_t now = AP_HAL::micros();
        bool need_tick = false;
        if (now - last_thread_run_us >= 1000) {
            // run the timer tick if it's been more than 1ms since we last run
            need_tick = true;
            last_thread_run_us = now;
        }

        // change the thread priority if requested - if unbuffered it should only have higher priority than the owner so that
        // handoff occurs immediately
        if (mask & EVT_TRANSMIT_UNBUFFERED) {
            chThdSetPriority(unbuffered_writes ? MIN(_uart_owner_thd->realprio + 1, APM_UART_UNBUFFERED_PRIORITY) : APM_UART_PRIORITY);
        }
        // send more data
        if (_tx_initialised && ((mask & EVT_TRANSMIT_DATA_READY) || need_tick || (hd_tx_active && (mask & EVT_TRANSMIT_END)))) {
            _tx_timer_tick();
        }
    }
}
#pragma GCC diagnostic pop

/*
  thread for handling UART receive

  We use events indexed by serial_num to trigger a more rapid send for
  unbuffered_write uarts, and run at 1kHz for general UART handling
 */
void UARTDriver::uart_rx_thread(void* arg)
{
    while (uart_rx_thread_ctx == nullptr) {
        hal.scheduler->delay_microseconds(1000);
    }

    while (true) {
        hal.scheduler->delay_microseconds(1000);

        for (uint8_t i=0; i<UART_MAX_DRIVERS; i++) {
            if (uart_drivers[i] == nullptr) {
                continue;
            }
            if (uart_drivers[i]->_rx_initialised) {
                uart_drivers[i]->_rx_timer_tick();
            }
        }
    }
}

/*
  initialise UART RX thread
 */
void UARTDriver::thread_rx_init(void)
{
    if (uart_rx_thread_ctx == nullptr) {
        xTaskCreate(uart_rx_thread,
                "UART_RX",
                HAL_UART_RX_STACK_SIZE,
                nullptr,
                APM_UART_PRIORITY,
                &uart_rx_thread_ctx);

        if (uart_rx_thread_ctx == nullptr) {
            AP_HAL::panic("Could not create UART RX thread\n");
        }
    }
}

/*
  initialise UART TX_thread
 */
void UARTDriver::thread_init(void)
{
    if (uart_thread_ctx == nullptr) {
        hal.util->snprintf(uart_thread_name, sizeof(uart_thread_name), sdef.is_usb ? "OTG%1u" : "UART%1u", sdef.instance);
        xTaskCreate(uart_thread_trampoline,
                uart_thread_name,
                HAL_UART_STACK_SIZE,
                this,
                unbuffered_writes ? APM_UART_UNBUFFERED_PRIORITY : APM_UART_PRIORITY,
                &uart_thread_ctx);
        if (uart_thread_ctx == nullptr) {
            AP_HAL::panic("Could not create UART TX thread\n");
        }
    }
}

void UARTDriver::uart_thread_trampoline(void* p)
{
    UARTDriver* uart = static_cast<UARTDriver*>(p);
    uart->uart_thread();
}

#ifndef HAL_STDOUT_SERIAL
/*
  hook to allow printf() to work on hal.console when we don't have a
  dedicated debug console
 */
static int hal_console_vprintf(const char *fmt, va_list arg)
{
/*
	uart_port_t p = uart_desc[uart_num].port;
	if (p == 0)
		esp_log_writev(ESP_LOG_INFO, "", fmt, ap);
	else
		AP_HAL::UARTDriver::vprintf(fmt, ap);
*/
    hal.console->vprintf(fmt, arg);
    return 1; // wrong length, but doesn't matter for what this is used for
}
#endif

void UARTDriver::begin(uint32_t b, uint16_t rxS, uint16_t txS)
{
    thread_rx_init();

    if (sdef.serial == nullptr) {
        return;
    }
    uint16_t min_tx_buffer = HAL_UART_MIN_TX_SIZE;
    uint16_t min_rx_buffer = HAL_UART_MIN_RX_SIZE;

    /*
      increase min RX size to ensure we can receive a fully utilised
      UART if we are running our receive loop at 40Hz. This means 25ms
      of data. Assumes 10 bits per byte, which is normal for most
      protocols
     */
    bool rx_size_by_baudrate = true;
    if (rx_size_by_baudrate) {
        min_rx_buffer = MAX(min_rx_buffer, b/(40*10));
    }

    if (sdef.is_usb) {
        // give more buffer space for log download on USB
        min_tx_buffer *= 2;
    }

#if HAL_MEM_CLASS >= HAL_MEM_CLASS_500
    // on boards with plenty of memory we can use larger buffers
    min_tx_buffer *= 2;
    min_rx_buffer *= 2;
#endif

    // on PX4 we have enough memory to have a larger transmit and
    // receive buffer for all ports. This means we don't get delays
    // while waiting to write GPS config packets
    if (txS < min_tx_buffer) {
        txS = min_tx_buffer;
    }
    if (rxS < min_rx_buffer) {
        rxS = min_rx_buffer;
    }

    /*
      allocate the read buffer
      we allocate buffers before we successfully open the device as we
      want to allocate in the early stages of boot, and cause minimum
      thrashing of the heap once we are up. The ttyACM0 driver may not
      connect for some time after boot
     */
    while (_in_rx_timer) {
        hal.scheduler->delay(1);
    }
    if (rxS != _readbuf.get_size()) {
        _rx_initialised = false;
        _readbuf.set_size(rxS);
    }

    bool clear_buffers = false;
    if (b != 0) {
        // clear buffers on baudrate change, but not on the console (which is usually USB)
        if (_baudrate != b && hal.console != this) {
            clear_buffers = true;
        }
        _baudrate = b;
    }

    if (clear_buffers) {
        _readbuf.clear();
    }

    /*
      allocate the write buffer
     */
    while (_in_tx_timer) {
        hal.scheduler->delay(1);
    }
    if (txS != _writebuf.get_size()) {
        _tx_initialised = false;
        _writebuf.set_size(txS);
    }

    if (clear_buffers) {
        _writebuf.clear();
    }

#if HAL_USE_SERIAL == TRUE
        if (_baudrate != 0) {
            sercfg.speed = _baudrate;

            // start with options from set_options()
            sercfg.cr1 = _cr1_options;
            sercfg.cr2 = _cr2_options;
            sercfg.cr3 = _cr3_options;

            if (!(sercfg.cr2 & USART_CR2_STOP2_BITS)) {
                sercfg.cr2 |= USART_CR2_STOP1_BITS;
            }
            sercfg.ctx = (void*)this;

            sdStart((SerialDriver*)sdef.serial, &sercfg);

        }
#endif // HAL_USE_SERIAL

    if (_writebuf.get_size()) {
        _tx_initialised = true;
    }
    if (_readbuf.get_size()) {
        _rx_initialised = true;
    }
    _uart_owner_thd = xTaskGetCurrentTaskHandle();
    // initialize the TX thread if necessary
    thread_init();

    // setup flow control
    set_flow_control(_flow_control);

    if (serial_num == 0 && _tx_initialised) {
#ifndef HAL_STDOUT_SERIAL
        // setup hal.console to take printf() output
        vprintf_console_hook = hal_console_vprintf;
#endif
    }

//SETUP ESP32
    if (uart_num < ARRAY_SIZE(uart_desc)) {
        uart_port_t p = uart_desc[uart_num].port;
        if (!_initialized) {

            uart_config_t config = {
                .baud_rate = (int)b,
                .data_bits = UART_DATA_8_BITS,
                .parity = UART_PARITY_DISABLE,
                .stop_bits = UART_STOP_BITS_1,
                .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            };
            uart_param_config(p, &config);
            uart_set_pin(p,
                         uart_desc[uart_num].tx,
                         uart_desc[uart_num].rx,
                         UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
            //uart_driver_install(p, 2*UART_FIFO_LEN, 0, 0, nullptr, 0);
            uart_driver_install(p, 2*UART_FIFO_LEN, 0, 0, nullptr, 0);
            _readbuf.set_size(RX_BUF_SIZE);
            _writebuf.set_size(TX_BUF_SIZE);

            _initialized = true;
        } else {
			flush();
            uart_set_baudrate(p, b);

        }
    }
}

void UARTDriver::end()
{
    if (_initialized) {
        uart_driver_delete(uart_desc[uart_num].port);
        _readbuf.set_size(0);
        _writebuf.set_size(0);
    }
    _initialized = false;
}

void UARTDriver::flush()
{
	uart_port_t p = uart_desc[uart_num].port;
	uart_flush(p);
}

bool UARTDriver::is_initialized()
{
    return _initialized;
}

void UARTDriver::set_blocking_writes(bool blocking)
{
    //blocking writes do not used anywhere
}

bool UARTDriver::tx_pending()
{
    return (_writebuf.available() > 0);
}


uint32_t UARTDriver::available()
{
    if (!_initialized) {
        return 0;
    }
    return _readbuf.available();
}

uint32_t UARTDriver::txspace()
{
    if (!_initialized) {
        return 0;
    }
    int result =  _writebuf.space();
    result -= TX_BUF_SIZE / 4;
    return MAX(result, 0);

}

int16_t IRAM_ATTR UARTDriver::read()
{
    if (!_initialized) {
        return -1;
    }
    uint8_t byte;
    if (!_readbuf.read_byte(&byte)) {
        return -1;
    }
    return byte;
}

void IRAM_ATTR UARTDriver::_timer_tick(void)
{
    if (!_initialized) {
        return;
    }
    read_data();
    write_data();
}

void IRAM_ATTR UARTDriver::read_data()
{
    uart_port_t p = uart_desc[uart_num].port;
    int count = 0;
    do {
        count = uart_read_bytes(p, _buffer, sizeof(_buffer), 0);
        if (count > 0) {
            _readbuf.write(_buffer, count);
        }
    } while (count > 0);
}

void IRAM_ATTR UARTDriver::write_data()
{
    uart_port_t p = uart_desc[uart_num].port;
    int count = 0;
    _write_mutex.take_blocking();
    do {
        count = _writebuf.peekbytes(_buffer, sizeof(_buffer));
        if (count > 0) {
            count = uart_tx_chars(p, (const char*) _buffer, count);
            _writebuf.advance(count);
        }
    } while (count > 0);
    _write_mutex.give();
}

size_t IRAM_ATTR UARTDriver::write(uint8_t c)
{
    return write(&c,1);
}

size_t IRAM_ATTR UARTDriver::write(const uint8_t *buffer, size_t size)
{
    if (!_initialized) {
        return 0;
    }

    _write_mutex.take_blocking();


    size_t ret = _writebuf.write(buffer, size);
    _write_mutex.give();
    return ret;
}

bool UARTDriver::discard_input()
{
    uart_port_t p = uart_desc[uart_num].port;
	return uart_flush_input(p) == ESP_OK;
}
#endif
