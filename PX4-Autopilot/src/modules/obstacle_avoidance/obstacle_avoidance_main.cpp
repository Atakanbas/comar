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
 * @file obstacle_avoidance_main.cpp
 *
 * Obstacle avoidance module for autonomous ground rover
 *
 * This module processes obstacle distance data and generates avoidance commands.
 * It implements simple obstacle avoidance algorithms for ground vehicles.
 *
 * @author Atkan
 * @date 2024
 */

#include "obstacle_avoidance.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>

using namespace time_literals;

ObstacleAvoidance::ObstacleAvoidance() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
	// Initialize obstacle avoidance state
	_avoidance_active = false;
	_avoidance_direction = 0.0f;
	_avoidance_speed = 0.0f;
	_obstacle_detected = false;
}

ObstacleAvoidance::~ObstacleAvoidance()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool ObstacleAvoidance::init()
{
	if (!_obstacle_distance_sub.registerCallback()) {
		PX4_ERR("obstacle_distance callback registration failed");
		return false;
	}

	if (!_distance_sensor_sub.registerCallback()) {
		PX4_ERR("distance_sensor callback registration failed");
		return false;
	}

	ScheduleOnInterval(50_ms); // 20Hz update rate

	return true;
}

void ObstacleAvoidance::Run()
{
	if (should_exit()) {
		ScheduleClear();
		_obstacle_distance_sub.unregisterCallback();
		_distance_sensor_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	// Read obstacle data
	read_obstacle_data();

	// Process obstacle avoidance
	process_obstacle_avoidance();

	// Publish avoidance commands
	publish_avoidance_commands();

	perf_end(_loop_perf);
}

void ObstacleAvoidance::read_obstacle_data()
{
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

void ObstacleAvoidance::process_obstacle_avoidance()
{
	// ATKAN: Buradan değiştirmeler yapılmalı - Engel kaçma algoritması
	// Simple obstacle avoidance algorithm

	_obstacle_detected = false;
	_avoidance_active = false;
	_avoidance_direction = 0.0f;
	_avoidance_speed = 0.0f;

	// Check obstacle distance data
	if (_obstacle_distance.timestamp > 0) {
		// Find minimum distance in front sector (0-30 degrees)
		float min_distance = FLT_MAX;
		int min_angle = 0;

		for (int i = 0; i < 30; i++) {
			if (_obstacle_distance.distances[i] > 0 &&
			    _obstacle_distance.distances[i] < min_distance) {
				min_distance = _obstacle_distance.distances[i];
				min_angle = i;
			}
		}

		// Check if obstacle is too close
		if (min_distance < _param_min_obstacle_distance.get()) {
			_obstacle_detected = true;
			_avoidance_active = true;

			// Determine avoidance direction
			// Check left and right sectors
			float left_distance = 0.0f;
			float right_distance = 0.0f;

			// Left sector (330-360 degrees)
			for (int i = 330; i < 360; i++) {
				if (_obstacle_distance.distances[i] > 0) {
					left_distance += _obstacle_distance.distances[i];
				}
			}
			left_distance /= 30.0f;

			// Right sector (0-30 degrees)
			for (int i = 0; i < 30; i++) {
				if (_obstacle_distance.distances[i] > 0) {
					right_distance += _obstacle_distance.distances[i];
				}
			}
			right_distance /= 30.0f;

			// Choose direction with more space
			if (left_distance > right_distance) {
				_avoidance_direction = -1.0f; // Turn left
			} else {
				_avoidance_direction = 1.0f;  // Turn right
			}

			// Reduce speed based on obstacle distance
			_avoidance_speed = (min_distance / _param_min_obstacle_distance.get()) * 0.5f;
			_avoidance_speed = math::constrain(_avoidance_speed, 0.1f, 0.5f);

			PX4_INFO("Obstacle detected at %.2f m, avoiding %s",
				(double)min_distance, _avoidance_direction > 0 ? "right" : "left");
		}
	}

	// Check distance sensor data
	if (_distance_sensor.timestamp > 0) {
		if (_distance_sensor.current_distance > 0 &&
		    _distance_sensor.current_distance < _param_min_obstacle_distance.get()) {
			_obstacle_detected = true;
			_avoidance_active = true;
			_avoidance_direction = 1.0f; // Default turn right
			_avoidance_speed = 0.2f; // Slow down

			PX4_INFO("Obstacle detected by distance sensor at %.2f m",
				(double)_distance_sensor.current_distance);
		}
	}
}

void ObstacleAvoidance::publish_avoidance_commands()
{
	// ATKAN: Buradan değiştirmeler yapılmalı - Kaçma komutlarını yayınla
	// Publish obstacle avoidance commands
	obstacle_avoidance_s obstacle_avoidance{};
	obstacle_avoidance.timestamp = hrt_absolute_time();
	obstacle_avoidance.avoidance_active = _avoidance_active;
	obstacle_avoidance.avoidance_direction = _avoidance_direction;
	obstacle_avoidance.avoidance_speed = _avoidance_speed;
	obstacle_avoidance.obstacle_detected = _obstacle_detected;

	_obstacle_avoidance_pub.publish(obstacle_avoidance);
}

int ObstacleAvoidance::print_status()
{
	PX4_INFO("Obstacle Avoidance Status:");
	PX4_INFO("  Avoidance Active: %s", _avoidance_active ? "YES" : "NO");
	PX4_INFO("  Obstacle Detected: %s", _obstacle_detected ? "YES" : "NO");
	PX4_INFO("  Avoidance Direction: %.2f", (double)_avoidance_direction);
	PX4_INFO("  Avoidance Speed: %.2f", (double)_avoidance_speed);

	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);

	return 0;
}

int ObstacleAvoidance::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		print_usage("not running");
		return 1;
	}

	if (!strcmp(argv[0], "test_avoidance")) {
		_avoidance_active = true;
		_avoidance_direction = 1.0f;
		_avoidance_speed = 0.3f;
		PX4_INFO("Test avoidance activated");
		return 0;
	}

	if (!strcmp(argv[0], "reset_avoidance")) {
		_avoidance_active = false;
		_avoidance_direction = 0.0f;
		_avoidance_speed = 0.0f;
		PX4_INFO("Avoidance reset");
		return 0;
	}

	return print_usage("unknown command");
}

int ObstacleAvoidance::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Obstacle avoidance module for autonomous ground rover.

This module processes obstacle distance data and generates avoidance commands.
It implements simple obstacle avoidance algorithms for ground vehicles.

### Examples
CLI usage example:
$ obstacle_avoidance start
$ obstacle_avoidance status
$ obstacle_avoidance test_avoidance
$ obstacle_avoidance reset_avoidance
$ obstacle_avoidance stop

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("obstacle_avoidance", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND("stop");
	PRINT_MODULE_USAGE_COMMAND("status");
	PRINT_MODULE_USAGE_COMMAND("test_avoidance");
	PRINT_MODULE_USAGE_COMMAND("reset_avoidance");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int obstacle_avoidance_main(int argc, char *argv[])
{
	return ObstacleAvoidance::main(argc, argv);
}
