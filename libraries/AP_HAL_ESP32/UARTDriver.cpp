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
//#include "GPIO.h"
#include <AP_Math/AP_Math.h>
#include <AP_InternalError/AP_InternalError.h>
#include <AP_Common/ExpandingString.h>
#include "Scheduler.h"

#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include <esp_event.h>

extern const AP_HAL::HAL& hal;

ESP_EVENT_DEFINE_BASE(UART_EVENTS);

using namespace ESP32;

const UARTDriver::SerialDef UARTDriver::_serial_tab[] = {
    {.serial=UART_NUM_0, .instance=0, .tx_line=GPIO_NUM_1, .rx_line=GPIO_NUM_3  },
	{.serial=UART_NUM_1, .instance=1, .tx_line=GPIO_NUM_33, .rx_line=GPIO_NUM_39  },
	{.serial=UART_NUM_2, .instance=2, .tx_line=GPIO_NUM_25, .rx_line=GPIO_NUM_34 }
 };

// handle for UART handling thread
TaskHandle_t UARTDriver::uart_rx_thread_ctx;

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

UARTDriver::UARTDriver(uint8_t _serial_num)
    :   serial_num(_serial_num),
        sdef(_serial_tab[_serial_num]),
        _baudrate(57600)
{
    //configASSERT(serial_num < UART_MAX_DRIVERS, "too many UART drivers");
    uart_drivers[serial_num] = this;
}

/*
  thread for handling UART send/receive

  We use events indexed by serial_num to trigger a more rapid send for
  unbuffered_write uarts, and run at 1kHz for general UART handling
 */
#pragma GCC diagnostic push
#pragma GCC diagnostic error "-Wframe-larger-than=128"
void UARTDriver::uart_thread(uint32_t event_id)
{
    //uart_wait_tx_done(sdef.serial, 1* portTICK_PERIOD_MS);
    uint32_t now = AP_HAL::micros();
    bool need_tick = false;
    if (now - last_thread_run_us >= 1000) {
        // run the timer tick if it's been more than 1ms since we last run
        need_tick = true;
        last_thread_run_us = now;
    }

    if (event_id == EVT_TRANSMIT_UNBUFFERED) {
            //chThdSetPriority(unbuffered_writes ? MIN(_uart_owner_thd->realprio + 1, APM_UART_UNBUFFERED_PRIORITY) : APM_UART_PRIORITY);
    }

    if (_tx_initialised && (event_id ==  EVT_TRANSMIT_DATA_READY || need_tick)) {
        _tx_timer_tick();
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
        /*xTaskCreate(uart_thread_trampoline,
                uart_thread_name,
                HAL_UART_STACK_SIZE,
                this,
                unbuffered_writes ? APM_UART_UNBUFFERED_PRIORITY : APM_UART_PRIORITY,
                &uart_thread_ctx);*/

        esp_event_loop_args_t loop_with_task_args = {
            .queue_size = LOOP_EVENT_QUEUE_SIZE,
            .task_name = uart_thread_name, // task will be created
            .task_priority = (unsigned int)(unbuffered_writes ? APM_UART_UNBUFFERED_PRIORITY : APM_UART_PRIORITY),
            .task_stack_size = HAL_UART_STACK_SIZE,
            .task_core_id = tskNO_AFFINITY
        };
        ESP_ERROR_CHECK(esp_event_loop_create(&loop_with_task_args, &uart_thread_ctx));
        ESP_ERROR_CHECK(
        esp_event_handler_instance_register_with(uart_thread_ctx,
                                                 UART_EVENTS,
                                                 ESP_EVENT_ANY_ID,
                                                 uart_thread_trampoline,
                                                 (void*)this,
                                                 NULL));
        if (uart_thread_ctx == nullptr) {
            AP_HAL::panic("Could not create UART TX thread\n");
        }
    }
}

void UARTDriver::uart_thread_trampoline(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    UARTDriver* uart = static_cast<UARTDriver*>(event_handler_arg);
    uart->uart_thread(event_id);
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
            uart_config_t config = {
                .baud_rate = _baudrate,
                .data_bits = UART_DATA_8_BITS,
                .parity = UART_PARITY_DISABLE,
                .stop_bits = UART_STOP_BITS_1,
                .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            }; //TODO check clock
            uart_param_config(sdef.serial, &config);
            uart_set_pin(sdef.serial,
                         sdef.tx_line,
                         sdef.rx_line,
                         sdef.rts_line, //UART_PIN_NO_CHANGE,
                         sdef.cts_line);
            uart_driver_install(sdef.serial, 2*RX_BOUNCE_BUFSIZE, 2*TX_BOUNCE_BUFSIZE, 20, &serevt, 0);
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
        //vprintf_console_hook = hal_console_vprintf;
#endif
    }
}

void UARTDriver::begin(uint32_t b)
{
    begin(b, 0, 0);
}

void UARTDriver::end()
{
    while (_in_rx_timer) hal.scheduler->delay(1);
    _rx_initialised = false;
    while (_in_tx_timer) hal.scheduler->delay(1);
    _tx_initialised = false;

    if (sdef.is_usb) {
#ifdef HAVE_USB_SERIAL
        //TODO disable USB
#endif
    } else {
#if HAL_USE_SERIAL == TRUE
        uart_driver_delete(sdef.serial);
#endif
    }

    _readbuf.set_size(0);
    _writebuf.set_size(0);
}

void UARTDriver::flush()
{
	uart_flush(sdef.serial);
}

bool UARTDriver::is_initialized()
{
    return _tx_initialised && _rx_initialised;
}

void UARTDriver::set_blocking_writes(bool blocking)
{
    _blocking_writes = blocking;
}

bool UARTDriver::tx_pending() { return false; }

/* Empty implementations of Stream virtual methods */
uint32_t UARTDriver::available() {
    if (!_rx_initialised || lock_read_key) {
        return 0;
    }
    if (sdef.is_usb) {
#ifdef HAVE_USB_SERIAL

#endif
    }
    return _readbuf.available();
}

uint32_t UARTDriver::available_locked(uint32_t key)
{
    if (lock_read_key != 0 && key != lock_read_key) {
        return -1;
    }
    if (sdef.is_usb) {
#ifdef HAVE_USB_SERIAL
#endif
    }
    return _readbuf.available();
}

uint32_t UARTDriver::txspace()
{
    if (!_tx_initialised) {
        return 0;
    }
    return _writebuf.space();
}

bool UARTDriver::discard_input()
{
    if (lock_read_key != 0 || _uart_owner_thd != xTaskGetCurrentTaskHandle()){
        return false;
    }
    if (!_rx_initialised) {
        return false;
    }

    _readbuf.clear();

    if (!_rts_is_active) {
        update_rts_line();
    }

    return true;
}

ssize_t IRAM_ATTR UARTDriver::read(uint8_t *buffer, uint16_t count)
{
    if (lock_read_key != 0 || _uart_owner_thd != xTaskGetCurrentTaskHandle()){
        return -1;
    }
    if (!_rx_initialised) {
        return -1;
    }

    const uint32_t ret = _readbuf.read(buffer, count);
    if (ret == 0) {
        return 0;
    }

    if (!_rts_is_active) {
        update_rts_line();
    }

    return ret;
}

int16_t IRAM_ATTR UARTDriver::read()
{
    if (lock_read_key != 0 || _uart_owner_thd != xTaskGetCurrentTaskHandle()){
        return -1;
    }
    if (!_rx_initialised) {
        return -1;
    }

    uint8_t byte;
    if (!_readbuf.read_byte(&byte)) {
        return -1;
    }
    if (!_rts_is_active) {
        update_rts_line();
    }

    return byte;
}

int16_t UARTDriver::read_locked(uint32_t key)
{
    if (lock_read_key != 0 && key != lock_read_key) {
        return -1;
    }
    if (!_rx_initialised) {
        return -1;
    }
    uint8_t byte;
    if (!_readbuf.read_byte(&byte)) {
        return -1;
    }
    if (!_rts_is_active) {
        update_rts_line();
    }
    return byte;
}

/* write one byte to the port */
size_t UARTDriver::write(uint8_t c)
{
    if (lock_write_key != 0) {
        return 0;
    }
    _write_mutex.take_blocking();

    if (!_tx_initialised) {
        _write_mutex.give();
        return 0;
    }

    while (_writebuf.space() == 0) {
        if (!_blocking_writes || unbuffered_writes) {
            _write_mutex.give();
            return 0;
        }
        // release the semaphore while sleeping
        _write_mutex.give();
        hal.scheduler->delay(1);
        _write_mutex.take_blocking();
    }
    size_t ret = _writebuf.write(&c, 1);
    if (unbuffered_writes) {
        ESP_ERROR_CHECK(esp_event_post_to(uart_thread_ctx,
                                          UART_EVENTS,
                                          EVT_TRANSMIT_DATA_READY,
                                          NULL,
                                          0,
                                          portMAX_DELAY));
    }
    _write_mutex.give();
    return ret;
}

/* write a block of bytes to the port */
size_t UARTDriver::write(const uint8_t *buffer, size_t size)
{
    if (!_tx_initialised || lock_write_key != 0) {
		return 0;
	}

    if (_blocking_writes && !unbuffered_writes) {
        /*
          use the per-byte delay loop in write() above for blocking writes
         */
        size_t ret = 0;
        while (size--) {
            if (write(*buffer++) != 1) break;
            ret++;
        }
        return ret;
    }

    WITH_SEMAPHORE(_write_mutex);

    size_t ret = _writebuf.write(buffer, size);
    if (unbuffered_writes) {
        ESP_ERROR_CHECK(esp_event_post_to(uart_thread_ctx,
                                          UART_EVENTS,
                                          EVT_TRANSMIT_DATA_READY,
                                          NULL,
                                          0,
                                          portMAX_DELAY));
    }
    return ret;
}

/*
  lock the uart for exclusive use by write_locked() and read_locked() with the right key
 */
bool UARTDriver::lock_port(uint32_t write_key, uint32_t read_key)
{
    if (lock_write_key && write_key != lock_write_key && read_key != 0) {
        // someone else is using it
        return false;
    }
    if (lock_read_key && read_key != lock_read_key && read_key != 0) {
        // someone else is using it
        return false;
    }
    lock_write_key = write_key;
    lock_read_key = read_key;
    return true;
}

/*
   write to a locked port. If port is locked and key is not correct then 0 is returned
   and write is discarded. All writes are non-blocking
*/
size_t UARTDriver::write_locked(const uint8_t *buffer, size_t size, uint32_t key)
{
    if (lock_write_key != 0 && key != lock_write_key) {
        return 0;
    }
    WITH_SEMAPHORE(_write_mutex);
    return _writebuf.write(buffer, size);
}

/*
  wait for data to arrive, or a timeout. Return true if data has
  arrived, false on timeout
 */
bool UARTDriver::wait_timeout(uint16_t n, uint32_t timeout_ms)
{
    uint32_t t0 = AP_HAL::millis();
    while (available() < n) {
    //    chEvtGetAndClearEvents(EVT_DATA);
     //   _wait.n = n;
      //  _wait.thread_ctx = chThdGetSelfX();
        uint32_t now = AP_HAL::millis();
        if (now - t0 >= timeout_ms) {
            break;
        }
       // chEvtWaitAnyTimeout(EVT_DATA, chTimeMS2I(timeout_ms - (now - t0)));
        //TODO send / check event for optimisation
    }
    return available() >= n;
}

/*
  write any pending bytes to the device, non-DMA method
 */
void UARTDriver::write_pending_bytes_NODMA(uint32_t n)
{
    WITH_SEMAPHORE(_write_mutex);

    ByteBuffer::IoVec vec[2];
    uint16_t nwritten = 0;

    const auto n_vec = _writebuf.peekiovec(vec, n);
    for (int i = 0; i < n_vec; i++) {
        int ret = -1;
        if (sdef.is_usb) {
            ret = 0;
#ifdef HAVE_USB_SERIAL
            //ret = chnWriteTimeout((SerialUSBDriver*)sdef.serial, vec[i].data, vec[i].len, TIME_IMMEDIATE);
            //TODO USB
#endif
        } else {
#if HAL_USE_SERIAL == TRUE
            ret = uart_write_bytes(sdef.serial, vec[i].data, vec[i].len);
#endif
        }
        if (ret < 0) {
            break;
        }
        if (ret > 0) {
            _last_write_completed_us = AP_HAL::micros();
            nwritten += ret;
        }
        _writebuf.advance(ret);

        /* We wrote less than we asked for, stop */
        if ((unsigned)ret != vec[i].len) {
            break;
        }
    }

    _total_written += nwritten;
    _tx_stats_bytes += nwritten;
}

/*
  write any pending bytes to the device
 */
#pragma GCC diagnostic push
#pragma GCC diagnostic error "-Wframe-larger-than=128"
void UARTDriver::write_pending_bytes(void)
{
    // write any pending bytes
    uint32_t n = _writebuf.available();
    if (n <= 0) {
        return;
    }

    write_pending_bytes_NODMA(n);

    // handle AUTO flow control mode
    if (_flow_control == FLOW_CONTROL_AUTO) {
        if (_first_write_started_us == 0) {
            _first_write_started_us = AP_HAL::micros();
        }

        //TODO fix esp32
        // without DMA we need to look at the number of bytes written into the queue versus the
        // remaining queue space
        size_t space;
        uart_get_buffered_data_len(sdef.serial, &space);
        uint32_t used = SERIAL_BUFFERS_SIZE - space;
        // threshold is 8 for the GCS_Common code to unstick SiK radios, which
        // sends 6 bytes with flow control disabled
        const uint8_t threshold = 8;
        if (_total_written > used && _total_written - used > threshold) {
            _flow_control = FLOW_CONTROL_ENABLE;
            return;
        }
        if (AP_HAL::micros() - _first_write_started_us > 500*1000UL) {
            // it doesn't look like hw flow control is working
            hal.console->printf("disabling flow control on serial %u\n", sdef.get_index());
            set_flow_control(FLOW_CONTROL_DISABLE);
        }
    }
}
#pragma GCC diagnostic pop

/*
  push any pending bytes to/from the serial port. This is called at
  1kHz in the timer thread. Doing it this way reduces the system call
  overhead in the main task enormously.
 */
void UARTDriver::_rx_timer_tick(void)
{
    if (!_rx_initialised) {
        return;
    }

    _in_rx_timer = true;

    // don't try IO on a disconnected USB port
/*
    if (sdef.is_usb) {
#ifdef HAVE_USB_SERIAL
        if (((SerialUSBDriver*)sdef.serial)->config->usbp->state != USB_ACTIVE) {
            _in_rx_timer = false;
            return;
        }
#endif
    }
    if (sdef.is_usb) {
#ifdef HAVE_USB_SERIAL
        ((GPIO *)hal.gpio)->set_usb_connected();
#endif
    }
*/
    read_bytes_NODMA();

    if (_wait.thread_ctx && _readbuf.available() >= _wait.n) {
        ESP_ERROR_CHECK(esp_event_post_to(uart_thread_ctx,
                                          UART_EVENTS,
                                          EVT_DATA,
                                          NULL,
                                          0,
                                          portMAX_DELAY));
    }
    _in_rx_timer = false;
}

// regular serial read
void UARTDriver::read_bytes_NODMA()
{
    // try to fill the read buffer
    ByteBuffer::IoVec vec[2];

    const auto n_vec = _readbuf.reserve(vec, _readbuf.space());
    for (int i = 0; i < n_vec; i++) {
        int ret = 0;
        //Do a non-blocking read
        if (sdef.is_usb) {
#ifdef HAVE_USB_SERIAL
            //ret = chnReadTimeout((SerialUSBDriver*)sdef.serial, vec[i].data, vec[i].len, TIME_IMMEDIATE);
            //TODO USB
#endif
        } else {
#if HAL_USE_SERIAL == TRUE
            ret = uart_read_bytes(sdef.serial, vec[i].data, vec[i].len, 0)
#endif
        }
        if (ret < 0) {
            break;
        }

        /* stop reading as we read less than we asked for */
        if ((unsigned)ret < vec[i].len) {
            break;
        }
    }
}

/*
  push any pending bytes to/from the serial port. This is called at
  1kHz in the timer thread. Doing it this way reduces the system call
  overhead in the main task enormously.
 */
void UARTDriver::_tx_timer_tick(void)
{
    if (!_tx_initialised) {
        return;
    }

    _in_tx_timer = true;

    // don't try IO on a disconnected USB port
/*
    if (sdef.is_usb) {
#ifdef HAVE_USB_SERIAL
        if (((SerialUSBDriver*)sdef.serial)->config->usbp->state != USB_ACTIVE) {
            _in_tx_timer = false;
            return;
        }
#endif
    }
    if (sdef.is_usb) {
#ifdef HAVE_USB_SERIAL
        ((GPIO *)hal.gpio)->set_usb_connected();
#endif
    }
    */

    // now do the write
    write_pending_bytes();

    _in_tx_timer = false;
}

/*
  change flow control mode for port
 */
void UARTDriver::set_flow_control(enum flow_control flowcontrol)
{
    if (sdef.rts_line == 0 || sdef.is_usb) {
        // no hw flow control available
        return;
    }
    //TODO setup cts/rts line
    /*
    */
}

/*
  software update of rts line. We don't use the HW support for RTS as
  it has no hysteresis, so it ends up toggling RTS on every byte
 */
void UARTDriver::update_rts_line(void)
{
    if (sdef.rts_line == 0 || _flow_control == FLOW_CONTROL_DISABLE) {
        return;
    }
    //TODO setup rts
    /*uint16_t space = _readbuf.space();
    if (_rts_is_active && space < 16) {
        _rts_is_active = false;
        palSetLine(sdef.rts_line);
    } else if (!_rts_is_active && space > 32) {
        _rts_is_active = true;
        palClearLine(sdef.rts_line);
    }
    */
}

/*
   setup unbuffered writes for lower latency
 */
bool UARTDriver::set_unbuffered_writes(bool on)
{
    unbuffered_writes = on;
    ESP_ERROR_CHECK(esp_event_post_to(uart_thread_ctx,
                                      UART_EVENTS,
                                      EVT_TRANSMIT_UNBUFFERED,
                                      NULL,
                                      0,
                                      portMAX_DELAY));
    return true;
}

/*
  setup parity
 */
void UARTDriver::configure_parity(uint8_t v)
{
    if (sdef.is_usb) {
        // not possible
        return;
    }
    //TODO setup parity configuration
    /*
#if HAL_USE_SERIAL == TRUE
    // stop and start to take effect
    sdStop((SerialDriver*)sdef.serial);

#ifdef USART_CR1_M0
    // cope with F3 and F7 where there are 2 bits in CR1_M
    const uint32_t cr1_m0 = USART_CR1_M0;
#else
    const uint32_t cr1_m0 = USART_CR1_M;
#endif

    switch (v) {
    case 0:
        // no parity
        sercfg.cr1 &= ~(USART_CR1_PCE | USART_CR1_PS | USART_CR1_M);
        break;
    case 1:
        // odd parity
        // setting USART_CR1_M ensures extra bit is used as parity
        // not last bit of data
        sercfg.cr1 |= cr1_m0 | USART_CR1_PCE;
        sercfg.cr1 |= USART_CR1_PS;
        break;
    case 2:
        // even parity
        sercfg.cr1 |= cr1_m0 | USART_CR1_PCE;
        sercfg.cr1 &= ~USART_CR1_PS;
        break;
    }

    sdStart((SerialDriver*)sdef.serial, &sercfg);

#if CH_CFG_USE_EVENTS == TRUE
    if (parity_enabled) {
        chEvtUnregister(chnGetEventSource((SerialDriver*)sdef.serial), &ev_listener);
    }
    parity_enabled = (v != 0);
    if (parity_enabled) {
        chEvtRegisterMaskWithFlags(chnGetEventSource((SerialDriver*)sdef.serial),
                                   &ev_listener,
                                   EVT_PARITY,
                                   SD_PARITY_ERROR);
    }
#endif
*/

//#endif // HAL_USE_SERIAL
}

/*
  set stop bits
 */
void UARTDriver::set_stop_bits(int n)
{
    if (sdef.is_usb) {
        // not possible
        return;
    }
    //TODO setup stop bits
#if HAL_USE_SERIAL
/*
    // stop and start to take effect
    sdStop((SerialDriver*)sdef.serial);

    switch (n) {
    case 1:
        _cr2_options &= ~USART_CR2_STOP2_BITS;
        _cr2_options |= USART_CR2_STOP1_BITS;
        break;
    case 2:
        _cr2_options &= ~USART_CR2_STOP1_BITS;
        _cr2_options |= USART_CR2_STOP2_BITS;
        break;
    }
    sercfg.cr2 = _cr2_options;

    sdStart((SerialDriver*)sdef.serial, &sercfg);
    */
#endif // HAL_USE_SERIAL
}


// record timestamp of new incoming data
void UARTDriver::receive_timestamp_update(void)
{
    _receive_timestamp[_receive_timestamp_idx^1] = AP_HAL::micros64();
    _receive_timestamp_idx ^= 1;
}

/*
  return timestamp estimate in microseconds for when the start of
  a nbytes packet arrived on the uart. This should be treated as a
  time constraint, not an exact time. It is guaranteed that the
  packet did not start being received after this time, but it
  could have been in a system buffer before the returned time.

  This takes account of the baudrate of the link. For transports
  that have no baudrate (such as USB) the time estimate may be
  less accurate.

  A return value of zero means the HAL does not support this API
*/
uint64_t UARTDriver::receive_time_constraint_us(uint16_t nbytes)
{
    uint64_t last_receive_us = _receive_timestamp[_receive_timestamp_idx];
    if (_baudrate > 0 && !sdef.is_usb) {
        // assume 10 bits per byte. For USB we assume zero transport delay
        uint32_t transport_time_us = (1000000UL * 10UL / _baudrate) * (nbytes + available());
        last_receive_us -= transport_time_us;
    }
    return last_receive_us;
}

/*
 set user specified PULLUP/PULLDOWN options from SERIALn_OPTIONS
*/
void UARTDriver::set_pushpull(uint16_t options)
{
    //TODO setup pushpulls
#if HAL_USE_SERIAL == TRUE && !defined(STM32F1)
    /*
    if ((options & OPTION_PULLDOWN_RX) && arx_line) {
        palLineSetPushPull(arx_line, PAL_PUSHPULL_PULLDOWN);
    }
    if ((options & OPTION_PULLDOWN_TX) && atx_line) {
        palLineSetPushPull(atx_line, PAL_PUSHPULL_PULLDOWN);
    }
    if ((options & OPTION_PULLUP_RX) && arx_line) {
        palLineSetPushPull(arx_line, PAL_PUSHPULL_PULLUP);
    }
    if ((options & OPTION_PULLUP_TX) && atx_line) {
        palLineSetPushPull(atx_line, PAL_PUSHPULL_PULLUP);
    }
    */
#endif
}

// set optional features, return true on success
bool UARTDriver::set_options(uint16_t options)
{
    if (sdef.is_usb) {
        // no options allowed on USB
        return (options == 0);
    }
    bool ret = true;

    _last_options = options;
    //TODO make it

    return ret;
}

// get optional features
uint8_t UARTDriver::get_options(void) const
{
    return _last_options;
}

// request information on uart I/O for @SYS/uarts.txt
void UARTDriver::uart_info(ExpandingString &str)
{
    // a header to allow for machine parsers to determine format
    str.printf("UARTV1\n");

    uint32_t now_ms = AP_HAL::millis();
    for (uint8_t i = 0; i < UART_MAX_DRIVERS; i++) {
        UARTDriver* uart = uart_drivers[i];

        if (uart == nullptr || uart->uart_thread_ctx == nullptr) {
            continue;
        }

        const char* fmt = "%-8s TX%c=%8u RX%c=%8u TXBD=%6u RXBD=%6u\n";
        str.printf(fmt, uart->uart_thread_name, ' ', uart->_tx_stats_bytes,
            ' ', uart->_rx_stats_bytes,
            uart->_tx_stats_bytes * 10000 / (now_ms - _last_stats_ms), uart->_rx_stats_bytes * 10000 / (now_ms - _last_stats_ms));

        uart->_tx_stats_bytes = 0;
        uart->_rx_stats_bytes = 0;
    }

    _last_stats_ms = now_ms;
}

#endif
