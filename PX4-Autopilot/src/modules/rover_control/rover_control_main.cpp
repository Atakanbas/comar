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
 * @file rover_control_main.cpp
 *
 * Autonomous ground rover control module
 *
 * This module handles the main control logic for autonomous ground vehicles.
 * It reads sensor data, processes control algorithms, and outputs motor commands.
 *
 * @author Atakan
 * @date 2024
 */

#include "rover_control.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>

using namespace time_literals;

RoverControl::RoverControl() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
	// Initialize control parameters
	_throttle_control = 0.0f;
	_steering_control = 0.0f;
	_emergency_stop = false;
	_control_mode = CONTROL_MODE_MANUAL;
}

RoverControl::~RoverControl()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool RoverControl::init()
{
	if (!_vehicle_attitude_sub.registerCallback()) {
		PX4_ERR("vehicle_attitude callback registration failed");
		return false;
	}

	if (!_vehicle_local_position_sub.registerCallback()) {
		PX4_ERR("vehicle_local_position callback registration failed");
		return false;
	}

	if (!_obstacle_distance_sub.registerCallback()) {
		PX4_ERR("obstacle_distance callback registration failed");
		return false;
	}

	if (!_distance_sensor_sub.registerCallback()) {
		PX4_ERR("distance_sensor callback registration failed");
		return false;
	}

	ScheduleNow();

	return true;
}

void RoverControl::Run()
{
	if (should_exit()) {
		ScheduleClear();
		_vehicle_attitude_sub.unregisterCallback();
		_vehicle_local_position_sub.unregisterCallback();
		_obstacle_distance_sub.unregisterCallback();
		_distance_sensor_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	// Read sensor data
	read_sensor_data();

	// Run control algorithm
	run_control_algorithm();

	// Safety checks
	safety_checks();

	// Publish control outputs
	publish_control_outputs();

	perf_end(_loop_perf);
}

void RoverControl::read_sensor_data()
{
	// Read vehicle attitude
	vehicle_attitude_s vehicle_attitude;
	if (_vehicle_attitude_sub.update(&vehicle_attitude)) {
		_vehicle_attitude = vehicle_attitude;
	}

	// Read local position
	vehicle_local_position_s vehicle_local_position;
	if (_vehicle_local_position_sub.update(&vehicle_local_position)) {
		_vehicle_local_position = vehicle_local_position;
	}

	// Read obstacle distance data
	obstacle_distance_s obstacle_distance;
	if (_obstacle_distance_sub.update(&obstacle_distance)) {
		_obstacle_distance = obstacle_distance;
	}

	// Read distance sensor data
	distance_sensor_s distance_sensor;
	if (_distance_sensor_sub.update(&distance_sensor)) {
		_distance_sensor = distance_sensor;
	}
}

void RoverControl::run_control_algorithm()
{
	// ATKAN: Buradan değiştirmeler yapılmalı - Ana kontrol algoritması
	// Bu fonksiyon sensör verilerini okuyup motor komutlarını hesaplar

	// Basic control logic for now
	// TODO: Implement proper control algorithms

	// Calculate throttle based on speed setpoint
	float speed_setpoint = 0.0f; // TODO: Get from mission or manual control
	float current_speed = _vehicle_local_position.vx;

	// Simple PID controller for speed
	float speed_error = speed_setpoint - current_speed;
	_throttle_control = speed_error * _param_speed_p.get();

	// Limit throttle
	_throttle_control = math::constrain(_throttle_control, -1.0f, 1.0f);

	// Calculate steering based on heading setpoint
	float heading_setpoint = 0.0f; // TODO: Get from mission or manual control
	float current_heading = atan2f(_vehicle_attitude.q[1], _vehicle_attitude.q[0]);

	// Simple PID controller for heading
	float heading_error = heading_setpoint - current_heading;
	_steering_control = heading_error * _param_heading_p.get();

	// Limit steering
	_steering_control = math::constrain(_steering_control, -1.0f, 1.0f);
}

void RoverControl::safety_checks()
{
	// ATKAN: Buradan değiştirmeler yapılmalı - Güvenlik kontrolleri
	// Emergency stop conditions
	if (_emergency_stop) {
		_throttle_control = 0.0f;
		_steering_control = 0.0f;
		return;
	}

	// Check for obstacles
	if (_obstacle_distance.timestamp > 0) {
		// Find minimum distance
		float min_distance = FLT_MAX;
		for (int i = 0; i < 72; i++) {
			if (_obstacle_distance.distances[i] > 0 &&
			    _obstacle_distance.distances[i] < min_distance) {
				min_distance = _obstacle_distance.distances[i];
			}
		}

		// Emergency stop if too close to obstacle
		if (min_distance < _param_min_obstacle_distance.get()) {
			_throttle_control = 0.0f;
			PX4_WARN("Emergency stop: obstacle too close (%.2f m)", (double)min_distance);
		}
	}

	// Check for sensor failures
	if (hrt_elapsed_time(&_vehicle_attitude.timestamp) > 1_s) {
		PX4_ERR("Attitude sensor timeout");
		_emergency_stop = true;
	}
}

void RoverControl::publish_control_outputs()
{
	// ATKAN: Buradan değiştirmeler yapılmalı - Motor komutlarını yayınla
	// Publish actuator control commands
	actuator_controls_status_s actuator_controls_status{};
	actuator_controls_status.timestamp = hrt_absolute_time();

	// Main motor control (throttle)
	actuator_controls_status.control[0] = _throttle_control;

	// Steering motor control
	actuator_controls_status.control[1] = _steering_control;

	// Set other controls to zero
	for (int i = 2; i < 8; i++) {
		actuator_controls_status.control[i] = 0.0f;
	}

	_actuator_control_pub.publish(actuator_controls_status);

	// Publish vehicle thrust setpoint for rover position control
	vehicle_thrust_setpoint_s vehicle_thrust_setpoint{};
	vehicle_thrust_setpoint.timestamp = hrt_absolute_time();
	vehicle_thrust_setpoint.xyz[0] = _throttle_control;
	vehicle_thrust_setpoint.xyz[1] = 0.0f;
	vehicle_thrust_setpoint.xyz[2] = 0.0f;
	_vehicle_thrust_setpoint_pub.publish(vehicle_thrust_setpoint);

	// Publish vehicle torque setpoint for steering
	vehicle_torque_setpoint_s vehicle_torque_setpoint{};
	vehicle_torque_setpoint.timestamp = hrt_absolute_time();
	vehicle_torque_setpoint.xyz[0] = 0.0f;
	vehicle_torque_setpoint.xyz[1] = 0.0f;
	vehicle_torque_setpoint.xyz[2] = _steering_control;
	_vehicle_torque_setpoint_pub.publish(vehicle_torque_setpoint);
}

int RoverControl::print_status()
{
	PX4_INFO("Rover Control Status:");
	PX4_INFO("  Control Mode: %d", _control_mode);
	PX4_INFO("  Throttle: %.2f", (double)_throttle_control);
	PX4_INFO("  Steering: %.2f", (double)_steering_control);
	PX4_INFO("  Emergency Stop: %s", _emergency_stop ? "YES" : "NO");

	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);

	return 0;
}

int RoverControl::custom_command(int argc, char *argv[])
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

	return print_usage("unknown command");
}

int RoverControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Autonomous ground rover control module.

This module handles the main control logic for autonomous ground vehicles.
It reads sensor data, processes control algorithms, and outputs motor commands.

### Examples
CLI usage example:
$ rover_control start
$ rover_control status
$ rover_control emergency_stop
$ rover_control reset_emergency_stop
$ rover_control stop

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("rover_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND("stop");
	PRINT_MODULE_USAGE_COMMAND("status");
	PRINT_MODULE_USAGE_COMMAND("emergency_stop");
	PRINT_MODULE_USAGE_COMMAND("reset_emergency_stop");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int rover_control_main(int argc, char *argv[])
{
	return RoverControl::main(argc, argv);
}
