/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file pwm_control_main.cpp
 *
 * PWM control driver for autonomous ground rover
 *
 * This driver handles PWM output control for main motor and steering motor.
 * It reads actuator control commands and outputs appropriate PWM signals.
 *
 * @author Atkan
 * @date 2024
 */

#include "pwm_control.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>

using namespace time_literals;

PWMControl::PWMControl() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
	// Initialize PWM outputs
	_main_motor_pwm = 1000; // Neutral position
	_steering_motor_pwm = 1500; // Center position
	_emergency_stop = false;
}

PWMControl::~PWMControl()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool PWMControl::init()
{
	if (!_actuator_control_sub.registerCallback()) {
		PX4_ERR("actuator_control callback registration failed");
		return false;
	}

	// Initialize PWM hardware
	if (!init_pwm_hardware()) {
		PX4_ERR("PWM hardware initialization failed");
		return false;
	}

	ScheduleOnInterval(20_ms); // 50Hz update rate

	return true;
}

void PWMControl::Run()
{
	if (should_exit()) {
		ScheduleClear();
		_actuator_control_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	// Read actuator control commands
	actuator_controls_status_s actuator_controls_status;
	if (_actuator_control_sub.update(&actuator_controls_status)) {
		// ATKAN: Buradan değiştirmeler yapılmalı - PWM kontrol algoritması
		// Convert actuator control to PWM values
		convert_actuator_to_pwm(actuator_controls_status);

		// Apply safety limits
		apply_safety_limits();

		// Output PWM signals
		output_pwm_signals();
	}

	perf_end(_loop_perf);
}

bool PWMControl::init_pwm_hardware()
{
	// ATKAN: Buradan değiştirmeler yapılmalı - PWM donanım başlatma
	// Initialize PWM pins for main motor and steering motor
	// This is a placeholder - actual implementation depends on hardware

	PX4_INFO("PWM hardware initialized");
	return true;
}

void PWMControl::convert_actuator_to_pwm(const actuator_controls_status_s &actuator_controls_status)
{
	// ATKAN: Buradan değiştirmeler yapılmalı - Aktüatör kontrolü PWM'e çevirme
	// Convert throttle control (-1.0 to 1.0) to PWM (1000-2000 us)
	float throttle = actuator_controls_status.control[0];
	_main_motor_pwm = 1000 + (throttle + 1.0f) * 500.0f;

	// Convert steering control (-1.0 to 1.0) to PWM (1000-2000 us)
	float steering = actuator_controls_status.control[1];
	_steering_motor_pwm = 1000 + (steering + 1.0f) * 500.0f;

	// Apply deadzone for steering
	if (fabsf(steering) < 0.1f) {
		_steering_motor_pwm = 1500; // Center position
	}
}

void PWMControl::apply_safety_limits()
{
	// ATKAN: Buradan değiştirmeler yapılmalı - Güvenlik limitleri
	// Emergency stop
	if (_emergency_stop) {
		_main_motor_pwm = 1000; // Stop motor
		_steering_motor_pwm = 1500; // Center steering
		return;
	}

	// Limit PWM range
	_main_motor_pwm = math::constrain(_main_motor_pwm, 1000, 2000);
	_steering_motor_pwm = math::constrain(_steering_motor_pwm, 1000, 2000);

	// Apply maximum throttle limit
	float max_throttle = _param_max_throttle.get();
	if (max_throttle < 1.0f) {
		float throttle_range = (2000 - 1000) * max_throttle;
		_main_motor_pwm = 1000 + throttle_range * ((_main_motor_pwm - 1000) / 1000.0f);
	}
}

void PWMControl::output_pwm_signals()
{
	// ATKAN: Buradan değiştirmeler yapılmalı - PWM sinyallerini çıkışa gönder
	// Output PWM signals to hardware
	// This is a placeholder - actual implementation depends on hardware

	// For now, just log the PWM values
	PX4_DEBUG("Main Motor PWM: %d, Steering PWM: %d", _main_motor_pwm, _steering_motor_pwm);
}

int PWMControl::print_status()
{
	PX4_INFO("PWM Control Status:");
	PX4_INFO("  Main Motor PWM: %d us", _main_motor_pwm);
	PX4_INFO("  Steering PWM: %d us", _steering_motor_pwm);
	PX4_INFO("  Emergency Stop: %s", _emergency_stop ? "YES" : "NO");

	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);

	return 0;
}

int PWMControl::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		print_usage("not running");
		return 1;
	}

	if (!strcmp(argv[0], "emergency_stop")) {
		_emergency_stop = true;
		PX4_INFO("Emergency stop activated");
		return 0;
	}

	if (!strcmp(argv[0], "reset_emergency_stop")) {
		_emergency_stop = false;
		PX4_INFO("Emergency stop reset");
		return 0;
	}

	if (!strcmp(argv[0], "test_pwm")) {
		if (argc < 3) {
			PX4_ERR("Usage: test_pwm <main_pwm> <steering_pwm>");
			return 1;
		}

		_main_motor_pwm = atoi(argv[1]);
		_steering_motor_pwm = atoi(argv[2]);
		PX4_INFO("Test PWM set: Main=%d, Steering=%d", _main_motor_pwm, _steering_motor_pwm);
		return 0;
	}

	return print_usage("unknown command");
}

int PWMControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
PWM control driver for autonomous ground rover.

This driver handles PWM output control for main motor and steering motor.
It reads actuator control commands and outputs appropriate PWM signals.

### Examples
CLI usage example:
$ pwm_control start
$ pwm_control status
$ pwm_control emergency_stop
$ pwm_control reset_emergency_stop
$ pwm_control test_pwm 1500 1500
$ pwm_control stop

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("pwm_control", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND("stop");
	PRINT_MODULE_USAGE_COMMAND("status");
	PRINT_MODULE_USAGE_COMMAND("emergency_stop");
	PRINT_MODULE_USAGE_COMMAND("reset_emergency_stop");
	PRINT_MODULE_USAGE_COMMAND("test_pwm");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int pwm_control_main(int argc, char *argv[])
{
	return PWMControl::main(argc, argv);
}
