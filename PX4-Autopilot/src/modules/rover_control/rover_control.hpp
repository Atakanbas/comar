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
 * @file rover_control.hpp
 *
 * Autonomous ground rover control module header
 *
 * @author Atkan
 * @date 2024
 */

#pragma once

#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/obstacle_distance.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/actuator_controls_status.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>

#include <lib/perf/perf_counter.h>
#include <lib/parameters/param.h>

using namespace time_literals;

class RoverControl : public ModuleBase<RoverControl>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	RoverControl();
	~RoverControl() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void Run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

	bool init();

private:
	// Control modes
	enum ControlMode {
		CONTROL_MODE_MANUAL = 0,
		CONTROL_MODE_AUTO = 1,
		CONTROL_MODE_EMERGENCY = 2
	};

	// Sensor data
	vehicle_attitude_s _vehicle_attitude{};
	vehicle_local_position_s _vehicle_local_position{};
	obstacle_distance_s _obstacle_distance{};
	distance_sensor_s _distance_sensor{};

	// Control outputs
	float _throttle_control{0.0f};
	float _steering_control{0.0f};
	bool _emergency_stop{false};
	ControlMode _control_mode{CONTROL_MODE_MANUAL};

	// Subscriptions
	uORB::SubscriptionCallbackWorkItem _vehicle_attitude_sub{this, ORB_ID(vehicle_attitude)};
	uORB::SubscriptionCallbackWorkItem _vehicle_local_position_sub{this, ORB_ID(vehicle_local_position)};
	uORB::SubscriptionCallbackWorkItem _obstacle_distance_sub{this, ORB_ID(obstacle_distance)};
	uORB::SubscriptionCallbackWorkItem _distance_sensor_sub{this, ORB_ID(distance_sensor)};

	// Publications
	uORB::Publication<actuator_controls_status_s> _actuator_control_pub{ORB_ID(actuator_controls_status)};
	uORB::Publication<vehicle_thrust_setpoint_s> _vehicle_thrust_setpoint_pub{ORB_ID(vehicle_thrust_setpoint)};
	uORB::Publication<vehicle_torque_setpoint_s> _vehicle_torque_setpoint_pub{ORB_ID(vehicle_torque_setpoint)};

	// Performance counters
	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME ": cycle")};
	perf_counter_t _loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME ": interval")};

	// Control functions
	void read_sensor_data();
	void run_control_algorithm();
	void safety_checks();
	void publish_control_outputs();

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::RVR_SPEED_P>) _param_speed_p,
		(ParamFloat<px4::params::RVR_HEADING_P>) _param_heading_p,
		(ParamFloat<px4::params::RVR_MIN_DIST>) _param_min_obstacle_distance
	)
};
