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

#include "HAL_ESP32_Class.h"
#include <AP_HAL_Empty/AP_HAL_Empty_Private.h>
#include <AP_HAL_ESP32/AP_HAL_ESP32_Private.h>

#include "SdCard.h"
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_InternalError/AP_InternalError.h>
#ifndef HAL_BOOTLOADER_BUILD
#include <AP_Logger/AP_Logger.h>
#endif
#include <AP_Vehicle/AP_Vehicle_Type.h>

/*
static Empty::UARTDriver uartADriver;
static ESP32::UARTDriver cons(0);
static ESP32::UARTDriver uartBDriver(1);
#ifdef HAL_ESP32_WIFI
	#if HAL_ESP32_WIFI == 1
	static ESP32::WiFiDriver uartCDriver; //tcp, client should connect to 192.168.4.1 port 5760
	#elif HAL_ESP32_WIFI == 2
	static ESP32::WiFiUdpDriver uartCDriver; //udp
	#endif
#else
static Empty::UARTDriver uartCDriver;
#endif
static ESP32::UARTDriver uartDDriver(2);
static Empty::UARTDriver uartEDriver;
static Empty::UARTDriver uartFDriver;
static Empty::UARTDriver uartGDriver;
static Empty::UARTDriver uartHDriver;
static Empty::UARTDriver uartIDriver;
*/

#ifndef HAL_NO_UARTDRIVER
static HAL_UARTA_DRIVER;
static HAL_UARTB_DRIVER;
static HAL_UARTC_DRIVER;
static HAL_UARTD_DRIVER;
static HAL_UARTE_DRIVER;
static HAL_UARTF_DRIVER;
static HAL_UARTG_DRIVER;
static HAL_UARTH_DRIVER;
static HAL_UARTI_DRIVER;
#else
static Empty::UARTDriver uartADriver;
static Empty::UARTDriver uartBDriver;
static Empty::UARTDriver uartCDriver;
static Empty::UARTDriver uartDDriver;
static Empty::UARTDriver uartEDriver;
static Empty::UARTDriver uartFDriver;
static Empty::UARTDriver uartGDriver;
static Empty::UARTDriver uartHDriver;
static Empty::UARTDriver uartIDriver;
#endif
#if HAL_USE_I2C == TRUE && defined(HAL_I2C_DEVICE_LIST)
static ESP32::I2CDeviceManager i2cDeviceManager;
#else
static Empty::I2CDeviceManager i2cDeviceManager;
#endif

#if HAL_USE_SPI == TRUE
static ESP32::SPIDeviceManager spiDeviceManager;
#else
static Empty::SPIDeviceManager spiDeviceManager;
#endif

#if HAL_USE_ADC == TRUE && !defined(HAL_DISABLE_ADC_DRIVER)
static ESP32::AnalogIn analogIn;
#else
static Empty::AnalogIn analogIn;
#endif

#ifdef HAL_USE_EMPTY_STORAGE
static Empty::Storage storageDriver;
#else
static ESP32::Storage storageDriver;
#endif

static Empty::GPIO gpioDriver;
static ESP32::RCInput rcinDriver;

#if HAL_USE_PWM == TRUE
static ESP32::RCOutput rcoutDriver;
#else
static Empty::RCOutput rcoutDriver;
#endif

static ESP32::Scheduler schedulerInstance;
static ESP32::Util utilInstance;
static Empty::OpticalFlow opticalFlowDriver;

#if HAL_WITH_DSP
static ESP32::DSP dspDriver;
#else
static Empty::DSP dspDriver;
#endif

#ifndef HAL_NO_FLASH_SUPPORT
//static ESP32::Flash flashDriver;
static Empty::Flash flashDriver;
#else
static Empty::Flash flashDriver;
#endif

HAL_ESP32::HAL_ESP32() :
    AP_HAL::HAL(
        &uartADriver, //Console/mavlink
        &uartBDriver, //GPS 1
        &uartCDriver, //Telem 1
        &uartDDriver, //Telem 2
        &uartEDriver, //GPS 2
        &uartFDriver, //Extra 1
        &uartGDriver, //Extra 2
        &uartHDriver, //Extra 3
        &uartIDriver, //Extra 4
        &i2cDeviceManager,
        &spiDeviceManager,
        &analogIn,
        &storageDriver,
        &uartADriver,
        &gpioDriver,
        &rcinDriver,
        &rcoutDriver,
        &schedulerInstance,
        &utilInstance,
        &opticalFlowDriver,
        &flashDriver,
        &dspDriver,
        nullptr
    )
{}

static bool thread_running = false;        /**< Daemon status flag */
static void* daemon_task;              /**< Handle of daemon task / thread */

extern const AP_HAL::HAL& hal;

/*
  set the priority of the main APM task
 */
void hal_esp32_set_priority(uint8_t priority)
{
    //TODO change priority
    if (!daemon_task)
        return;

    vTaskPrioritySet(daemon_task, APM_MAIN_PRIORITY);
}

void* get_main_thread()
{
    return daemon_task;
}

static AP_HAL::HAL::Callbacks* g_callbacks;

static void IRAM_ATTR main_loop()
{
    daemon_task = xTaskGetCurrentTaskHandle();

    /*
      switch to high priority for main loop
     */
    vTaskPrioritySet(NULL, APM_MAIN_PRIORITY);

#ifdef HAL_I2C_CLEAR_BUS
    // Clear all I2C Buses. This can be needed on some boards which
    // can get a stuck I2C peripheral on boot
    ESP32::I2CBus::clear_all();
#endif

    hal.serial(0)->begin(115200);

#ifdef HAL_SPI_CHECK_CLOCK_FREQ
    // optional test of SPI clock frequencies
    ESP32::SPIDevice::test_clock_freq();
#endif

    hal.analogin->init();
    hal.scheduler->init();

    /*
      run setup() at low priority to ensure CLI doesn't hang the
      system, and to allow initial sensor read loops to run
     */
    hal_esp32_set_priority(APM_STARTUP_PRIORITY);

/*
    if (stm32_was_watchdog_reset()) {
        // load saved watchdog data
        stm32_watchdog_load((uint32_t *)&utilInstance.persistent_data, (sizeof(utilInstance.persistent_data)+3)/4);
        utilInstance.last_persistent_data = utilInstance.persistent_data;
    }*/

    schedulerInstance.hal_initialized();

    g_callbacks->setup();

#if HAL_ENABLE_SAVE_PERSISTENT_PARAMS
    utilInstance.apply_persistent_params();
#endif

    schedulerInstance.watchdog_pat();

    hal.scheduler->set_system_initialized();

    thread_running = true;

    /*
      switch to high priority for main loop
     */
    vTaskPrioritySet(NULL, APM_MAIN_PRIORITY);

    while (true) {
        g_callbacks->loop();

        /*
          give up 50 microseconds of time if the INS loop hasn't
          called delay_microseconds_boost(), to ensure low priority
          drivers get a chance to run. Calling
          delay_microseconds_boost() means we have already given up
          time from the main loop, so we don't need to do it again
          here
         */
#if !defined(HAL_DISABLE_LOOP_DELAY) && !APM_BUILD_TYPE(APM_BUILD_Replay)
        if (!schedulerInstance.check_called_boost()) {
            hal.scheduler->delay_microseconds(50);
        }
#endif
        schedulerInstance.watchdog_pat();
        //sched->print_stats(); // only runs every 60 seconds.
    }
    thread_running = false;
}

void HAL_ESP32::run(int argc, char * const argv[], Callbacks* callbacks) const
{
    /*
     * System initializations.
     * - ESP32 HAL initialization, this also initializes the configured device drivers
     *   and performs the board-specific initializations.
     * - Kernel initialization, the main() function becomes a thread and the
     *   RTOS is active.
     */

    g_callbacks = callbacks;

    //Takeover main
    main_loop();
}

const AP_HAL::HAL& AP_HAL::get_HAL() {
    static const HAL_ESP32 hal_esp32;
    return hal_esp32;
}

