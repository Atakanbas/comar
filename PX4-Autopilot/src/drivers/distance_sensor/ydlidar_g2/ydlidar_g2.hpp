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
 * @file ydlidar_g2.hpp
 * @author PX4 Development Team
 *
 * Driver for the YDLidar G2 laser rangefinder
 */

#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/drivers/rangefinder/PX4Rangefinder.hpp>
#include <drivers/drv_hrt.h>
#include <drivers/device/device.h>
#include <lib/parameters/param.h>
#include <lib/perf/perf_counter.h>

#include <uORB/Publication.hpp>
#include <uORB/topics/obstacle_distance.h>
#include <uORB/topics/distance_sensor.h>

using namespace time_literals;

/* Configuration Constants */
#define YDLIDAR_G2_MAX_PAYLOAD 1024
#define YDLIDAR_G2_DEFAULT_PORT "/dev/ttyS3"

// YDLidar G2 Protocol Commands
#define YDLIDAR_G2_CMD_START_SCAN    0x60
#define YDLIDAR_G2_CMD_STOP_SCAN     0x65
#define YDLIDAR_G2_CMD_GET_INFO      0x50
#define YDLIDAR_G2_CMD_GET_HEALTH    0x52
#define YDLIDAR_G2_CMD_RESET         0x40

// YDLidar G2 Protocol Constants
#define YDLIDAR_G2_HEADER_1          0xAA
#define YDLIDAR_G2_HEADER_2          0x55
#define YDLIDAR_G2_PACKET_SIZE       723  // 2 header + 720 data + 1 checksum
#define YDLIDAR_G2_DATA_SIZE         720  // 360 degrees * 2 bytes per measurement

class YDLidarG2 : public px4::ScheduledWorkItem
{
public:
	YDLidarG2(const char *port, uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING);
	~YDLidarG2() override;

	int init();
	void print_info();

protected:
	obstacle_distance_s _obstacle_map_msg{};
	uORB::Publication<obstacle_distance_s> _obstacle_distance_pub{ORB_ID(obstacle_distance)};

private:
	void start();
	void stop();
	void Run() override;
	int measure();
	int collect();

	// YDLidar G2 protocol functions
	int ydlidar_write_all(int fd, const uint8_t *data, int len);
	void process_scan_data(const uint8_t *buffer, int len);
	void update_obstacle_distance(float distance_m, float angle_deg);
	uint8_t convert_angle_to_bin(float angle_deg);

	PX4Rangefinder _px4_rangefinder;

	char _port[20] {};
	int _interval{100000};  // 100ms
	bool _collect_phase{false};
	int _fd{-1};
	uint8_t _readbuf[YDLIDAR_G2_MAX_PAYLOAD] {};
	unsigned _linebuf_index{0};
	hrt_abstime _last_read{0};

	// YDLidar G2 specific data
	bool _scan_started{false};
	bool _motor_started{false};
	uint8_t _start_attempts{0};
	hrt_abstime _last_start_cmd{0};
	uint8_t _scan_buffer[YDLIDAR_G2_PACKET_SIZE] {};
	unsigned _scan_buffer_len{0};
	uint16_t _distances[360] {};
	uint8_t _previous_bin{0};

	// Performance counters
	perf_counter_t _sample_perf;
	perf_counter_t _comms_errors;
};
