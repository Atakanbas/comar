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
 * @file obstacle_avoidance.hpp
 *
 * Obstacle avoidance module header for autonomous ground rover
 *
 * @author Atkan
 * @date 2024
 */

#pragma once

#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/obstacle_distance.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/obstacle_avoidance.h>

#include <lib/perf/perf_counter.h>
#include <lib/parameters/param.h>

using namespace time_literals;

class ObstacleAvoidance : public ModuleBase<ObstacleAvoidance>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	ObstacleAvoidance();
	~ObstacleAvoidance() override;

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
	// Obstacle data
	obstacle_distance_s _obstacle_distance{};
	distance_sensor_s _distance_sensor{};

	// Avoidance state
	bool _avoidance_active{false};
	float _avoidance_direction{0.0f}; // -1.0 to 1.0 (left to right)
	float _avoidance_speed{0.0f};     // 0.0 to 1.0 (slow to normal)
	bool _obstacle_detected{false};

	// Subscriptions
	uORB::SubscriptionCallbackWorkItem _obstacle_distance_sub{this, ORB_ID(obstacle_distance)};
	uORB::SubscriptionCallbackWorkItem _distance_sensor_sub{this, ORB_ID(distance_sensor)};

	// Publications
	uORB::Publication<obstacle_avoidance_s> _obstacle_avoidance_pub{ORB_ID(obstacle_avoidance)};

	// Performance counters
	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME ": cycle")};
	perf_counter_t _loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME ": interval")};

	// Control functions
	void read_obstacle_data();
	void process_obstacle_avoidance();
	void publish_avoidance_commands();

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::OBS_MIN_DIST>) _param_min_obstacle_distance
	)
};
