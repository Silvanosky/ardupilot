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
 *
 * Code by Charles "Silvanosky" Villard and David "Buzz" Bussenschutt
 *
 */

#include "RCOutput.h"

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

#include "driver/rtc_io.h"

#include <stdio.h>

using namespace ESP32;

#ifdef HAL_ESP32_RCOUT

gpio_num_t outputs_pins[] = HAL_ESP32_RCOUT;

//If the RTC source is not required, then GPIO32/Pin12/32K_XP and GPIO33/Pin13/32K_XN can be used as digital GPIOs.

#else
gpio_num_t outputs_pins[] = {};

#endif

#define MAX_CHANNELS ARRAY_SIZE(outputs_pins)

struct RCOutput::pwm_out RCOutput::pwm_group_list[MAX_CHANNELS];

void RCOutput::init()
{
	_max_channels = MAX_CHANNELS;


    //32 and 33 are special as they dont default to gpio, but can be if u disable their rtc setup:
    rtc_gpio_deinit(GPIO_NUM_32); 
    rtc_gpio_deinit(GPIO_NUM_33); 

printf("oooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo\n");
	printf("RCOutput::init() - channels available: %d \n",(int)MAX_CHANNELS);
printf("oooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo\n");

	static const mcpwm_io_signals_t signals[] = {
		MCPWM0A,
		MCPWM0B,
		MCPWM1A,
		MCPWM1B,
		MCPWM2A,
		MCPWM2B
	};
	static const mcpwm_timer_t timers[] = {
		MCPWM_TIMER_0,
		MCPWM_TIMER_1,
		MCPWM_TIMER_2

	};
	static const mcpwm_unit_t units[] = {
		MCPWM_UNIT_0,
		MCPWM_UNIT_1
	};
	static const mcpwm_operator_t operators[] = {
		MCPWM_OPR_A,
		MCPWM_OPR_B
	};

	for (uint8_t i = 0; i < MAX_CHANNELS; ++i)
	{
		auto unit = units[i/6];
		auto signal = signals[i % 6];
		auto timer = timers[i/2];

		//Save struct infos
		pwm_out &out = pwm_group_list[i];
		out.gpio_num = outputs_pins[i];
		out.unit_num = unit;
		out.timer_num = timer;
		out.io_signal = signal;
		out.op = operators[i%2];
		out.chan = i;

		//Setup gpio
		mcpwm_gpio_init(unit, signal, outputs_pins[i]);
		//Setup MCPWM module
		mcpwm_config_t pwm_config;
		pwm_config.frequency = 50;    //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
		pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
		pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
		pwm_config.counter_mode = MCPWM_UP_COUNTER;
		pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
		mcpwm_init(unit, timer, &pwm_config);
		mcpwm_start(unit, timer);
	}

	_initialized = true;
}



void RCOutput::set_freq(uint32_t chmask, uint16_t freq_hz)
{
	if (!_initialized)
	{
		return;
	}

	for (uint8_t i = 0; i < MAX_CHANNELS; i++) {
		if (chmask & 1 << i) {
			pwm_out &out = pwm_group_list[i];
			mcpwm_set_frequency(out.unit_num, out.timer_num, freq_hz);
		}
	}
}

void RCOutput::set_default_rate(uint16_t freq_hz)
{
    if (!_initialized) {
        return;
    }
	set_freq(0xFFFFFFFF, freq_hz);
}

uint16_t RCOutput::get_freq(uint8_t chan)
{
	if (!_initialized || chan >= MAX_CHANNELS) {
		return 50;
	}

	pwm_out &out = pwm_group_list[chan];
	return mcpwm_get_frequency(out.unit_num, out.timer_num);
}

void RCOutput::enable_ch(uint8_t chan)
{
	if (chan >= MAX_CHANNELS)
		return;

	pwm_out &out = pwm_group_list[chan];
	mcpwm_start(out.unit_num, out.timer_num);
}

void RCOutput::disable_ch(uint8_t chan)
{
	if (chan >= MAX_CHANNELS)
		return;

	write(chan, 0);
	pwm_out &out = pwm_group_list[chan];
	mcpwm_stop(out.unit_num, out.timer_num);
}

void RCOutput::write(uint8_t chan, uint16_t period_us)
{
	if (chan >= MAX_CHANNELS)
		return;

	if(_corked) {
		_pending[chan] = period_us;
		_pending_mask |= (1U<<chan);
	} else {
		write_int(chan, period_us);
	}

}

uint16_t RCOutput::read(uint8_t chan)
{
	if (chan >= MAX_CHANNELS || !_initialized)
		return 0;

	pwm_out &out = pwm_group_list[chan];
	double freq = mcpwm_get_frequency(out.unit_num, out.timer_num);
	double dprc = mcpwm_get_duty(out.unit_num, out.timer_num, out.op);
	return (1000000.0 * (dprc / 100.)) / freq;
}

void RCOutput::read(uint16_t *period_us, uint8_t len)
{
    for (int i = 0; i < MIN(len, _max_channels); i++) {
        period_us[i] = read(i);
    }
}

void RCOutput::cork()
{
	_corked = true;
}

void RCOutput::push()
{
	if (!_corked)
	{
		return;
	}

	for (uint8_t i = 0; i < MAX_CHANNELS; i++)
	{
		if ((1U<<i) & _pending_mask)
		{
			write_int(i, _pending[i]);
		}
	}

	_corked = false;
}

void RCOutput::write_int(uint8_t chan, uint16_t period_us)
{
	if (chan >= MAX_CHANNELS)
		return;
	pwm_out &out = pwm_group_list[chan];
	mcpwm_set_duty_in_us(out.unit_num, out.timer_num, out.op, period_us);
}

