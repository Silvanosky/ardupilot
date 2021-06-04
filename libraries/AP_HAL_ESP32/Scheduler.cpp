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

#include "AP_HAL_ESP32.h"

#include "Scheduler.h"
#include "RCInput.h"
#include "AnalogIn.h"

#include "AP_Math/AP_Math.h"
#include "SdCard.h"
#include "Profile.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "soc/rtc_wdt.h"
#include "esp_int_wdt.h"
#include "esp_task_wdt.h"

#include <AP_HAL/AP_HAL.h>
#include <stdio.h>

//#define SCHEDULERDEBUG 1

using namespace ESP32;

extern const AP_HAL::HAL& hal;

Scheduler::Scheduler()
{
    _initialized = false;
}

void disableCore0WDT(){
    TaskHandle_t idle_0 = xTaskGetIdleTaskHandleForCPU(0);
    if(idle_0 == NULL || esp_task_wdt_delete(idle_0) != ESP_OK){
        //print("Failed to remove Core 0 IDLE task from WDT");
    }
}
void disableCore1WDT(){
    TaskHandle_t idle_1 = xTaskGetIdleTaskHandleForCPU(1);
    if(idle_1 == NULL || esp_task_wdt_delete(idle_1) != ESP_OK){
        //print("Failed to remove Core 1 IDLE task from WDT");
    }
}

void Scheduler::init()
{

#ifdef SCHEDDEBUG
printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif

#ifndef HAL_NO_TIMER_THREAD
    xTaskCreate(_timer_thread,
                "APM_TIMER",
                TIMER_THD_WA_SIZE,
                this,
                APM_TIMER_PRIORITY,
                &_timer_task_handle);
#endif
#ifndef HAL_NO_RCOUT_THREAD
    xTaskCreate(_rcout_thread,
                "APM_RCOUT",
                RCOUT_THD_WA_SIZE,
                this,
                APM_RCOUT_PRIORITY,
                &_rcout_task_handle);
#endif
#ifndef HAL_NO_RCIN_THREAD
    xTaskCreate(_rcin_thread,
                "APM_RCIN",
                RCIN_THD_WA_SIZE,
                this,
                APM_RCIN_PRIORITY,
                &_rcin_task_handle);
#endif
#ifndef HAL_USE_EMPTY_IO
    xTaskCreate(_io_thread,
                "APM_IO",
                IO_THD_WA_SIZE,
                this,
                APM_IO_PRIORITY,
                &_io_task_handle);
#endif
#ifndef HAL_USE_EMPTY_STORAGE
    xTaskCreate(_storage_thread,
                "APM_STORAGE",
                STORAGE_THD_WA_SIZE,
                this,
                APM_STORAGE_PRIORITY,
                &_storage_task_handle);
#endif
 //   xTaskCreate(_print_profile, "APM_PROFILE", IO_SS, this, IO_PRIO, nullptr);

  //disableCore0WDT();
  //disableCore1WDT();

}

void Scheduler::delay_microseconds(uint16_t us)
{
    if (us <= 100) {
        ets_delay_us(us);
    } else {
        uint32_t tick = portTICK_PERIOD_MS * 1000;
        vTaskDelay((us+tick-1)/tick);
    }
}
/*
  wrapper around sem_post that boosts main thread priority
 */
static void set_high_priority()
{
#if APM_MAIN_PRIORITY_BOOST != APM_MAIN_PRIORITY
    hal_esp32_set_priority(APM_MAIN_PRIORITY_BOOST);
#endif
}

void Scheduler::boost_end()
{
#if APM_MAIN_PRIORITY_BOOST != APM_MAIN_PRIORITY
    if (in_main_thread() && _priority_boosted) {
        _priority_boosted = false;
        hal_esp32_set_priority(APM_MAIN_PRIORITY);
    }
#endif

}

/*
  a variant of delay_microseconds that boosts priority to
  APM_MAIN_PRIORITY_BOOST for APM_MAIN_PRIORITY_BOOST_USEC
  microseconds when the time completes. This significantly improves
  the regularity of timing of the main loop
 */
void Scheduler::delay_microseconds_boost(uint16_t usec)
{
    if (!_priority_boosted && in_main_thread()) {
        set_high_priority();
        _priority_boosted = true;
        _called_boost = true;
    }
    delay_microseconds(usec); //Suspends Thread for desired microseconds
}

bool Scheduler::check_called_boost()
{
    if (!_called_boost) {
        return false;
    }
    _called_boost = false;
    return true;

}

void Scheduler::delay(uint16_t ms)
{
    uint64_t start = AP_HAL::micros64();
    while ((AP_HAL::micros64() - start)/1000 < ms) {
        delay_microseconds(1000);
        if (_min_delay_cb_ms <= ms) {
            if (in_main_thread()) {
                call_delay_cb();
            }
        }
    }
}

void Scheduler::register_timer_process(AP_HAL::MemberProc proc)
{
#ifdef SCHEDDEBUG
printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif
	_timer_sem.take_blocking();
    for (uint8_t i = 0; i < _num_timer_procs; i++) {
        if (_timer_proc[i] == proc) {
			_timer_sem.give();
            return;
        }
    }
    if (_num_timer_procs < ESP32_SCHEDULER_MAX_TIMER_PROCS) {
        _timer_proc[_num_timer_procs] = proc;
        _num_timer_procs++;
    } else {
        printf("Out of timer processes\n");
    }
	_timer_sem.give();
}

void Scheduler::register_io_process(AP_HAL::MemberProc proc)
{
#ifdef SCHEDDEBUG
printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif
	_io_sem.take_blocking();
    for (uint8_t i = 0; i < _num_io_procs; i++) {
        if (_io_proc[i] == proc) {
			_io_sem.give();
            return;
        }
    }
    if (_num_io_procs < ESP32_SCHEDULER_MAX_IO_PROCS) {
        _io_proc[_num_io_procs] = proc;
        _num_io_procs++;
    } else {
        printf("Out of IO processes\n");
    }
	_io_sem.give();
}

void Scheduler::register_timer_failsafe(AP_HAL::Proc failsafe, uint32_t period_us)
{
    _failsafe = failsafe;
}

void Scheduler::reboot(bool hold_in_bootloader)
{
    printf("Restarting now...\n");
    hal.rcout->force_safety_on();

    unmount_sdcard();

    esp_restart();
}

void Scheduler::_run_timers()
{
#ifdef SCHEDULERDEBUG
printf("%s:%d start \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    if (_in_timer_proc) {
        return;
    }
#ifdef SCHEDULERDEBUG
printf("%s:%d _in_timer_proc \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    _in_timer_proc = true;

    int num_procs = 0;
    _timer_sem.take_blocking();
    num_procs = _num_timer_procs;
    _timer_sem.give();
    // now call the timer based drivers
    for (int i = 0; i < num_procs; i++) {
        if (_timer_proc[i]) {
            _timer_proc[i]();
        }
    }

    // and the failsafe, if one is setup
    if (_failsafe != nullptr) {
        _failsafe();
    }

#if HAL_USE_ADC == TRUE && !defined(HAL_DISABLE_ADC_DRIVER)
    // process analog input
    ((AnalogIn *)hal.analogin)->_timer_tick();
#endif

    _in_timer_proc = false;
}

void Scheduler::_timer_thread(void *arg)
{
#ifdef SCHEDDEBUG
	printf("%s:%d start\n", __PRETTY_FUNCTION__, __LINE__);
#endif
	Scheduler *sched = (Scheduler *)arg;
	while (!sched->_hal_initialized) {
		sched->delay_microseconds(1000);
	}
#ifdef SCHEDDEBUG
	printf("%s:%d initialised\n", __PRETTY_FUNCTION__, __LINE__);
#endif
	while (true) {
		sched->delay_microseconds(1000);

		sched->_run_timers();

		if (sched->in_expected_delay()) {
			sched->watchdog_pat();
		}
	}
}

void Scheduler::_rcout_thread(void *arg)
{
#ifndef HAL_NO_RCOUT_THREAD
    Scheduler *sched = (Scheduler *)arg;
    while (!sched->_hal_initialized) {
        sched->delay_microseconds(1000);
    }

#if HAL_USE_PWM == TRUE
    // trampoline into the rcout thread
    //((RCOutput*)hal.rcout)->rcout_thread();

    while (true) {
        sched->delay_microseconds(1000);
        hal.rcout->timer_tick();
    }
#endif
#endif
}

/*
  return true if we are in a period of expected delay. This can be
  used to suppress error messages
*/
bool Scheduler::in_expected_delay() const
{
    if (!_initialized) {
        // until setup() is complete we expect delays
        return true;
    }

    if (expect_delay_start != 0) {
        uint32_t now = AP_HAL::millis();
        if (now - expect_delay_start <= expect_delay_length) {
            return true;
        }
    }

    return false;
}

void Scheduler::_rcin_thread(void *arg)
{
    Scheduler *sched = (Scheduler *)arg;
    while (!sched->_hal_initialized) {
        sched->delay_microseconds(20000);
    }
    while (true) {
        sched->delay_microseconds(1000);
        ((RCInput *)hal.rcin)->_timer_tick();
    }
}

void Scheduler::_run_io()
{
#ifdef SCHEDULERDEBUG
printf("%s:%d start \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    if (_in_io_proc) {
        return;
    }
#ifdef SCHEDULERDEBUG
printf("%s:%d initialised \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    _in_io_proc = true;

    int num_procs = 0;
    _io_sem.take_blocking();
    num_procs = _num_io_procs;
    _io_sem.give();
    // now call the IO based drivers
    for (int i = 0; i < num_procs; i++) {
        if (_io_proc[i]) {
            _io_proc[i]();
        }
    }

    _in_io_proc = false;
}

void Scheduler::_io_thread(void* arg)
{
#ifdef SCHEDDEBUG
printf("%s:%d start \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    Scheduler *sched = (Scheduler *)arg;
    while (!sched->_initialized) {
        sched->delay_microseconds(1000);
    }

#ifndef HAL_NO_LOGGING
    uint32_t last_sd_start_ms = AP_HAL::millis();
#endif

#ifdef SCHEDDEBUG
printf("%s:%d initialised \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    while (true) {
        sched->delay_microseconds(1000);

        // run registered IO processes
        sched->_run_io();

#if !defined(HAL_NO_LOGGING)
        uint32_t now = AP_HAL::millis();
#endif

#ifndef HAL_NO_LOGGING
		if (!hal.util->get_soft_armed()) {
            // if sdcard hasn't mounted then retry it every 3s in the IO
            // thread when disarmed
            if (now - last_sd_start_ms > 3000) {
                last_sd_start_ms = now;
                sdcard_retry();
                //AP::FS().retry_mount(); //TODO check filesystem implementation
            }
        }
#endif
    }
}

void Scheduler::_storage_thread(void* arg)
{
#ifdef SCHEDDEBUG
printf("%s:%d start \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    Scheduler *sched = (Scheduler *)arg;
    while (!sched->_hal_initialized) {
        sched->delay_microseconds(10000);
    }
#ifdef SCHEDDEBUG
printf("%s:%d initialised \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    while (true) {
        sched->delay_microseconds(10000);

        // process any pending storage writes
        hal.storage->_timer_tick();
    }
}

void Scheduler::set_system_initialized()
{
#ifdef SCHEDDEBUG
printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    if (_initialized) {
        AP_HAL::panic("PANIC: scheduler::system_initialized called more than once");
    }

    _initialized = true;
}

void Scheduler::_print_profile(void* arg)
{
    Scheduler *sched = (Scheduler *)arg;
    while (!sched->_initialized) {
        sched->delay_microseconds(10000);
    }

    while (true)
    {
        sched->delay(10000);
        print_profile();
    }

}

/*
void Scheduler::_uart_thread(void *arg)
{
#ifdef SCHEDDEBUG
printf("%s:%d start \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    Scheduler *sched = (Scheduler *)arg;
    while (!_initialized) {
        sched->delay_microseconds(20000);
    }
#ifdef SCHEDDEBUG
printf("%s:%d initialised\n", __PRETTY_FUNCTION__, __LINE__);
#endif
    while (true) {
        sched->delay_microseconds(1000);
        hal.serial(0)->_timer_tick();
        hal.serial(1)->_timer_tick();
        hal.serial(2)->_timer_tick();
        hal.serial(3)->_timer_tick();
        hal.console->_timer_tick();
    }
}*/


    // get the active main loop rate
uint16_t Scheduler::get_loop_rate_hz(void) {
        if (_active_loop_rate_hz == 0) {
            _active_loop_rate_hz = _loop_rate_hz;
        }
        return _active_loop_rate_hz;
}

// once every 60 seconds, print some stats...
void Scheduler::print_stats(void)
{
    static int64_t last_run = 0;
    if (AP_HAL::millis64() - last_run > 60000) {
        char buffer[1024];
        vTaskGetRunTimeStats(buffer);
        printf("\n\n%s\n", buffer);
        heap_caps_print_heap_info(0);
        last_run = AP_HAL::millis64();
    }

   // printf("loop_rate_hz: %d",get_loop_rate_hz());
}

template <typename T>
void executor(T oui)
{
	oui();
}

void Scheduler::thread_create_trampoline(void *ctx)
{
    AP_HAL::MemberProc *t = (AP_HAL::MemberProc *)ctx;
    (*t)();
    free(t);
}

// calculates an integer to be used as the priority for a newly-created thread
uint8_t Scheduler::calculate_thread_priority(priority_base base, int8_t priority) const
{
    uint8_t thread_priority = APM_IO_PRIORITY;
    static const struct {
        priority_base base;
        uint8_t p;
    } priority_map[] = {
        { PRIORITY_BOOST, APM_MAIN_PRIORITY_BOOST},
        { PRIORITY_MAIN, APM_MAIN_PRIORITY},
        { PRIORITY_SPI, APM_SPI_PRIORITY},
        { PRIORITY_I2C, APM_I2C_PRIORITY},
        { PRIORITY_CAN, APM_CAN_PRIORITY},
        { PRIORITY_TIMER, APM_TIMER_PRIORITY},
        { PRIORITY_RCOUT, APM_RCOUT_PRIORITY},
        { PRIORITY_RCIN, APM_RCIN_PRIORITY},
        { PRIORITY_IO, APM_IO_PRIORITY},
        { PRIORITY_UART, APM_UART_PRIORITY},
        { PRIORITY_STORAGE, APM_STORAGE_PRIORITY},
        { PRIORITY_SCRIPTING, APM_SCRIPTING_PRIORITY},

    };
    for (uint8_t i=0; i<ARRAY_SIZE(priority_map); i++) {
        if (priority_map[i].base == base) {
#ifdef SCHEDDEBUG
printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif
            thread_priority = constrain_int16(priority_map[i].p + priority, LOWPRIO, HIGHPRIO);
            break;
        }
    }
    return thread_priority;
}

/*
  create a new thread
*/
bool Scheduler::thread_create(AP_HAL::MemberProc proc, const char *name, uint32_t stack_size, priority_base base, int8_t priority)
{
#ifdef SCHEDDEBUG
printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    // take a copy of the MemberProc, it is freed after thread exits
    AP_HAL::MemberProc *tproc = (AP_HAL::MemberProc *)malloc(sizeof(proc));
    if (!tproc) {
        return false;
    }
    *tproc = proc;

    const uint8_t thread_priority = calculate_thread_priority(base, priority);

	void* xhandle;
    BaseType_t xReturned = xTaskCreate(thread_create_trampoline,
                                       name,
                                       stack_size,
                                       tproc,
                                       thread_priority,
                                       &xhandle);

    if (xReturned != pdPASS) {
        free(tproc);
        return false;
    }
    return true;
}

/*
  inform the scheduler that we are calling an operation from the
  main thread that may take an extended amount of time. This can
  be used to prevent watchdog reset during expected long delays
  A value of zero cancels the previous expected delay
*/
void Scheduler::_expect_delay_ms(uint32_t ms)
{
    if (!in_main_thread()) {
        // only for main thread
        return;
    }

    // pat once immediately
    watchdog_pat();

    WITH_SEMAPHORE(expect_delay_sem);

    if (ms == 0) {
        if (expect_delay_nesting > 0) {
            expect_delay_nesting--;
        }
        if (expect_delay_nesting == 0) {
            expect_delay_start = 0;
        }
    } else {
        uint32_t now = AP_HAL::millis();
        if (expect_delay_start != 0) {
            // we already have a delay running, possibly extend it
            uint32_t done = now - expect_delay_start;
            if (expect_delay_length > done) {
                ms = MAX(ms, expect_delay_length - done);
            }
        }
        expect_delay_start = now;
        expect_delay_length = ms;
        expect_delay_nesting++;

        // also put our priority below timer thread if we are boosted
        boost_end();
    }
}

/*
  this is _expect_delay_ms() with check that we are in the main thread
 */
void Scheduler::expect_delay_ms(uint32_t ms)
{
    if (!in_main_thread()) {
        // only for main thread
        return;
    }
    _expect_delay_ms(ms);
}

// pat the watchdog
void Scheduler::watchdog_pat(void)
{
    //TODO inform watchdog about futur delay
    esp_task_wdt_reset();
    last_watchdog_pat_ms = AP_HAL::millis();
}


bool Scheduler::in_main_thread() const
{
    return get_main_thread() == xTaskGetCurrentTaskHandle();
}

