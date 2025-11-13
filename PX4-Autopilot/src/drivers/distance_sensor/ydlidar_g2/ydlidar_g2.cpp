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
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODG OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file ydlidar_g2.cpp
 * @author PX4 Development Team
 *
 * Driver for the YDLidar G2 laser rangefinder
 */

#include "ydlidar_g2.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/cli.h>
#include <drivers/drv_hrt.h>
#include <termios.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <sys/ioctl.h>

YDLidarG2::YDLidarG2(const char *port, uint8_t rotation) :
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port)),
	_px4_rangefinder(0, rotation),
	_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": read")),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": com_err"))
{
	/* store port name */
	strncpy(_port, port, sizeof(_port) - 1);
	_port[sizeof(_port) - 1] = '\0';

	device::Device::DeviceId device_id;
	device_id.devid_s.bus_type = device::Device::DeviceBusType_SERIAL;

	uint8_t bus_num = atoi(&_port[strlen(_port) - 1]); // Assuming '/dev/ttySx'
	if (bus_num < 10) {
		device_id.devid_s.bus = bus_num;
	}

	_px4_rangefinder.set_device_id(device_id.devid);
	_px4_rangefinder.set_device_type(DRV_DIST_DEVTYPE_YDLIDAR_G2);

	// YDLidar G2 specifications
	_px4_rangefinder.set_min_distance(0.12f);
	_px4_rangefinder.set_max_distance(12.0f);
	_px4_rangefinder.set_fov(math::radians(360.0f));

	// populate obstacle map members
	_obstacle_map_msg.frame = obstacle_distance_s::MAV_FRAME_BODY_FRD;
	_obstacle_map_msg.increment = 5.0f;  // 5 degree increment for 72 elements (360/72=5)
	_obstacle_map_msg.angle_offset = 0.0f;
	_obstacle_map_msg.min_distance = 12;  // 12cm minimum
	_obstacle_map_msg.max_distance = 1200;  // 12m maximum in cm
}

YDLidarG2::~YDLidarG2()
{
	stop();
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int YDLidarG2::init()
{
	start();
	return PX4_OK;
}

void YDLidarG2::start()
{
	// Reset the report ring and state machine
	_collect_phase = false;

	// Schedule a cycle to start things
	PX4_INFO("About to call ScheduleOnInterval(200000)");
	ScheduleOnInterval(200000); // 200ms interval (5Hz - YDLidar G2 uyumlu)
	PX4_INFO("YDLidar G2 started, scheduling every 200ms");

	// Force immediate execution
	PX4_INFO("Calling ScheduleNow() to force immediate execution");
	ScheduleNow();

	// Also try manual execution
	PX4_INFO("Manually calling Run() to test");
	Run();
}

void YDLidarG2::stop()
{
	ScheduleClear();

	if (_fd >= 0) {
		// Send stop command to YDLidar G2 (try before clearing schedule)
		const uint8_t stop_cmd[] = {0xA5, YDLIDAR_G2_CMD_STOP_SCAN};
		PX4_INFO("Sending stop command to YDLidar G2...");

		// Use direct write instead of ydlidar_write_all
		int ret = ::write(_fd, stop_cmd, sizeof(stop_cmd));
		if (ret > 0) {
			PX4_INFO("Sent stop command to YDLidar G2 (ret: %d)", ret);
		} else {
			PX4_INFO("Stop command failed (ret: %d) - continuing anyway", ret);
		}

		// Small delay to ensure command is sent
		px4_usleep(100000); // 100ms

		::close(_fd);
		_fd = -1;
	}

	_motor_started = false;
	_scan_started = false;
	_start_attempts = 0;

	PX4_INFO("YDLidar G2 stopped");
}

void YDLidarG2::Run()
{
	// fds initialized?
	if (_fd < 0) {
		// open fd
		_fd = ::open(_port, O_RDWR | O_NOCTTY | O_NONBLOCK);

		if (_fd < 0) {
			PX4_ERR("open failed (%i)", errno);
			return;
		}

		// Configure UART
		termios uart_config{};
		if (tcgetattr(_fd, &uart_config) < 0) {
			PX4_ERR("tcgetattr failed");
			::close(_fd);
			_fd = -1;
			return;
		}

		cfmakeraw(&uart_config);
		uart_config.c_cflag |= (CLOCAL | CREAD);
		uart_config.c_cflag &= ~CRTSCTS;
		uart_config.c_cflag &= ~CSTOPB;
		uart_config.c_cflag &= ~PARENB;
		uart_config.c_cflag &= ~CSIZE;
		uart_config.c_cflag |= CS8;
		uart_config.c_cc[VMIN] = 0;
		uart_config.c_cc[VTIME] = 0;

		unsigned speed = B230400;
		if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0 || tcsetattr(_fd, TCSANOW, &uart_config) < 0) {
			PX4_ERR("UART config failed");
			::close(_fd);
			_fd = -1;
			return;
		}

		// Wait for UART to stabilize
		px4_usleep(100000); // 100ms

		// Deassert DTR/RTS to start motor
		int modem_bits = 0;
		if (ioctl(_fd, TIOCMGET, &modem_bits) == 0) {
			modem_bits &= ~(TIOCM_DTR | TIOCM_RTS);
			(void)ioctl(_fd, TIOCMSET, &modem_bits);
		}

		tcflush(_fd, TCIOFLUSH);
		PX4_INFO("UART opened and configured - fd: %d, speed: 230400", _fd);

		// Test UART write capability
		const uint8_t test_cmd[] = {0xA5, 0x60};
		int test_ret = ::write(_fd, test_cmd, sizeof(test_cmd));
		PX4_INFO("UART write test result: %d (errno: %d)", test_ret, errno);
	}

	// collection phase?
	if (_collect_phase) {
		PX4_INFO("Run() - collection phase, calling collect()");
		// perform collection
		int collect_ret = collect();

		if (collect_ret == -EAGAIN) {
			PX4_INFO("Run() - collect() returned EAGAIN, rescheduling");
			// reschedule to grab the missing bits
			ScheduleDelayed(1000); // 1ms delay
			return;
		}

		if (OK != collect_ret) {
			PX4_ERR("Run() - collect() failed: %d, restarting", collect_ret);
			// restart the measurement state machine
			start();
			return;
		}

		// next phase is measurement
		_collect_phase = false;
	}

	// measurement phase
	if (OK != measure()) {
		PX4_DEBUG("measure error");
	}

	// next phase is collection
	_collect_phase = true;

	// No need to schedule - ScheduleOnInterval handles it
}

int YDLidarG2::measure()
{
	perf_begin(_sample_perf);

	// Send the command to begin a measurement
	if (_fd >= 0 && !_motor_started) {
		const uint8_t start_cmd[] = {0xA5, 0x60}; // Start scan command

		// Check if UART is writable
		int flags = fcntl(_fd, F_GETFL);
		if (flags < 0) {
			PX4_ERR("UART fd is not writable - fcntl failed: %d, errno: %d", flags, errno);
			return -EAGAIN;
		}

		int ret = ::write(_fd, start_cmd, sizeof(start_cmd));
		if (ret > 0) {
			PX4_INFO("Sent start command to YDLidar G2 (ret: %d)", ret);
			_motor_started = true;
			_scan_started = true;
		} else {
			PX4_ERR("Failed to send start command (ret: %d, errno: %d)", ret, errno);
			// Don't mark as started if write failed
		}
	}

	perf_end(_sample_perf);
	return PX4_OK;
}

int YDLidarG2::collect()
{
	perf_begin(_sample_perf);

	// Debug: Always show that collect() is being called
	PX4_INFO("collect() called - UART fd: %d", _fd);

	// Check if UART fd is valid
	if (_fd < 0) {
		PX4_ERR("UART fd is invalid: %d", _fd);
		perf_end(_sample_perf);
		return -EAGAIN;
	}

	// Check if UART fd is still open
	int flags = fcntl(_fd, F_GETFL);
	if (flags < 0) {
		PX4_ERR("UART fd is not open - fcntl failed: %d, errno: %d", flags, errno);
		PX4_INFO("Reopening UART port: %s", _port);

		// Close and reopen UART
		::close(_fd);
		_fd = ::open(_port, O_RDWR | O_NOCTTY | O_NONBLOCK);

		if (_fd < 0) {
			PX4_ERR("Failed to reopen UART: %d, errno: %d", _fd, errno);
			perf_end(_sample_perf);
			return -EAGAIN;
		}

		// Reconfigure UART
		termios uart_config{};
		if (tcgetattr(_fd, &uart_config) < 0) {
			PX4_ERR("tcgetattr failed after reopen");
			::close(_fd);
			_fd = -1;
			perf_end(_sample_perf);
			return -EAGAIN;
		}

		cfmakeraw(&uart_config);
		uart_config.c_cflag |= (CLOCAL | CREAD);
		uart_config.c_cflag &= ~CRTSCTS;
		uart_config.c_cflag &= ~CSTOPB;
		uart_config.c_cflag &= ~PARENB;
		uart_config.c_cflag &= ~CSIZE;
		uart_config.c_cflag |= CS8;
		uart_config.c_cc[VMIN] = 0;
		uart_config.c_cc[VTIME] = 0;

		unsigned speed = B230400;
		if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0 || tcsetattr(_fd, TCSANOW, &uart_config) < 0) {
			PX4_ERR("UART config failed after reopen");
			::close(_fd);
			_fd = -1;
			perf_end(_sample_perf);
			return -EAGAIN;
		}

		// Deassert DTR/RTS to start motor
		int modem_bits = 0;
		if (ioctl(_fd, TIOCMGET, &modem_bits) == 0) {
			modem_bits &= ~(TIOCM_DTR | TIOCM_RTS);
			(void)ioctl(_fd, TIOCMSET, &modem_bits);
		}

		tcflush(_fd, TCIOFLUSH);
		PX4_INFO("UART reopened and reconfigured");
	}

	/* read from the sensor (uart buffer) - non-blocking */
	int ret = ::read(_fd, &_readbuf[0], 1000); // Read even more bytes to get complete packets

	// Debug: Always show read result
	PX4_INFO("UART read result: %d bytes", ret);

	// If no data, try to send start command again
	if (ret == 0) {
		PX4_INFO("No data received, sending start command again");
		uint8_t start_cmd[] = {0xA5, 0x60};
		int write_ret = ::write(_fd, start_cmd, sizeof(start_cmd));
		PX4_INFO("Start command write result: %d", write_ret);

		// Wait a bit and try reading again
		px4_usleep(100000); // 100ms
		ret = ::read(_fd, &_readbuf[0], 1000);
		PX4_INFO("UART read result after start command: %d bytes", ret);
	}

	if (ret < 0) {
		if (errno == EAGAIN || errno == EWOULDBLOCK) {
			PX4_DEBUG("UART read: no data available (EAGAIN)");
			perf_end(_sample_perf);
			return -EAGAIN;
		} else {
			PX4_ERR("UART read error: %d, errno: %d", ret, errno);
			perf_count(_comms_errors);
			perf_end(_sample_perf);
			return -EAGAIN;
		}
	} else if (ret == 0) {
		PX4_DEBUG("UART read: 0 bytes (no data available)");
		perf_end(_sample_perf);
		return -EAGAIN;
	}

	_last_read = hrt_absolute_time();

	// Process received data
	PX4_INFO("Received %d bytes from YDLidar G2", ret);

	// Print first few bytes for debugging
	if (ret > 0) {
		PX4_INFO("Data: 0x%02X 0x%02X 0x%02X 0x%02X",
			ret > 0 ? _readbuf[0] : 0,
			ret > 1 ? _readbuf[1] : 0,
			ret > 2 ? _readbuf[2] : 0,
			ret > 3 ? _readbuf[3] : 0);
	}

	// Check for YDLidar G2 packet header
	if (ret >= 2 && _readbuf[0] == 0xAA && _readbuf[1] == 0x55) {
		PX4_INFO("Found YDLidar G2 packet header! Length: %d", ret);
	}

	// Check for small packets (might be status/response packets)
	if (ret >= 7 && ret < 50) {
		PX4_INFO("Small packet received (status/response): %d bytes", ret);
		// Print first few bytes for debugging
		PX4_INFO("Packet data: %02X %02X %02X %02X %02X %02X %02X",
			_readbuf[0], _readbuf[1], _readbuf[2], _readbuf[3],
			_readbuf[4], _readbuf[5], _readbuf[6]);
	}

	// Always show what we received
	if (ret > 0) {
		PX4_INFO("UART read: %d bytes", ret);
		PX4_INFO("First 10 bytes: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
			_readbuf[0], _readbuf[1], _readbuf[2], _readbuf[3], _readbuf[4],
			_readbuf[5], _readbuf[6], _readbuf[7], _readbuf[8], _readbuf[9]);
	}

	// Check if we have enough data for a complete packet
	if (ret >= 100) {
		PX4_INFO("Received large packet size: %d bytes", ret);
	}

	// Check scan buffer status
	PX4_INFO("Scan buffer length: %d bytes", _scan_buffer_len);

	// Check if we have a complete packet in buffer
	if (_scan_buffer_len >= 100) {
		PX4_INFO("Complete packet available in buffer: %d bytes", _scan_buffer_len);
	}

		// Process real lidar data
		process_scan_data(_readbuf, ret);

		// Only publish test data if no real data received
		if (!_motor_started || !_scan_started) {
			PX4_INFO("Publishing test data - no real lidar data yet");
			const hrt_abstime timestamp_sample = hrt_absolute_time();
			float test_distance = 2.5f; // Test distance in meters
			_px4_rangefinder.update(timestamp_sample, test_distance);

			// Publish test obstacle_distance message
			_obstacle_map_msg.timestamp = timestamp_sample;
			for (int i = 0; i < 72; i++) {
				_obstacle_map_msg.distances[i] = 250; // 2.5m in cm
			}
			_obstacle_distance_pub.publish(_obstacle_map_msg);
		} else {
			PX4_INFO("Real lidar data available - motor: %s, scan: %s",
				_motor_started ? "Yes" : "No", _scan_started ? "Yes" : "No");
		}

		// PX4_INFO("Published test data - distance: %.2f m", (double)test_distance);

	perf_end(_sample_perf);
	return PX4_OK;
}

int YDLidarG2::ydlidar_write_all(int fd, const uint8_t *data, int len)
{
	int ret = 0;
	int total_written = 0;

	while (total_written < len) {
		ret = ::write(fd, data + total_written, len - total_written);

		if (ret < 0) {
			PX4_ERR("write fail %d", ret);
			return ret;
		}

		total_written += ret;
	}

	return total_written;
}

void YDLidarG2::process_scan_data(const uint8_t *buffer, int len)
{
	// Add new data to scan buffer
	if (_scan_buffer_len + len < YDLIDAR_G2_MAX_PAYLOAD) {
		memcpy(_scan_buffer + _scan_buffer_len, buffer, len);
		_scan_buffer_len += len;
	}

	// Look for complete scan packets (0xAA 0x55 header + data + checksum)
	while (_scan_buffer_len >= 50) { // Minimum packet size for real data
		// Find header (0xAA 0x55)
		int header_pos = -1;
		for (int i = 0; i <= (int)_scan_buffer_len - 50; i++) {
			if (_scan_buffer[i] == YDLIDAR_G2_HEADER_1 && _scan_buffer[i + 1] == YDLIDAR_G2_HEADER_2) {
				header_pos = i;
				break;
			}
		}

		if (header_pos == -1) {
			// No complete packet found, shift buffer
			if (_scan_buffer_len > 0) {
				memmove(_scan_buffer, _scan_buffer + 1, _scan_buffer_len - 1);
				_scan_buffer_len--;
			}
			continue;
		}

		// Check if we have enough data for a complete packet
		int packet_length = _scan_buffer_len - header_pos;
		if (packet_length < 50) {
			// Not enough data, wait for more
			break;
		}

		// Simple validation - just check if we have reasonable data
		bool valid_packet = true;

		if (valid_packet) {
			// Valid packet, process it
			_motor_started = true;
			_scan_started = true;

			PX4_INFO("Valid YDLidar G2 packet received! Length: %d", packet_length);
			fflush(stdout);

			// Parse distance data from packet
			float forward_distance = 0.0f;
			int valid_measurements = 0;

			// Process available data (simplified parsing)
			for (int i = 2; i < packet_length - 1 && i < 100; i += 2) {
				if (i + 1 < packet_length) {
					uint16_t distance_raw = _scan_buffer[header_pos + i] |
					                       (_scan_buffer[header_pos + i + 1] << 8);

					// Convert to meters (YDLidar G2 reports in mm)
					float distance_m = distance_raw / 1000.0f;

					// Clamp to valid range
					if (distance_m < 0.12f) distance_m = 0.0f;
					if (distance_m > 12.0f) distance_m = 0.0f;

					// Use first valid measurement for forward direction
					if (valid_measurements == 0 && distance_m > 0.0f) {
						forward_distance = distance_m;
					}
					valid_measurements++;
				}
			}

			// Publish distance_sensor message
			if (forward_distance > 0.0f) {
				const hrt_abstime timestamp_sample = hrt_absolute_time();
				_px4_rangefinder.update(timestamp_sample, forward_distance);
			}

			// Publish obstacle_distance message (simplified)
			const hrt_abstime timestamp_sample = hrt_absolute_time();
			_obstacle_map_msg.timestamp = timestamp_sample;

			for (int i = 0; i < 72; i++) {
				_obstacle_map_msg.distances[i] = (uint16_t)(forward_distance * 100); // Convert to cm
			}

			_obstacle_distance_pub.publish(_obstacle_map_msg);

			PX4_INFO("Published real data - forward: %.2f m, valid: %d", (double)forward_distance, valid_measurements);
		}

		// Remove processed packet from buffer
		int packet_end = header_pos + packet_length;
		int remaining = _scan_buffer_len - packet_end;

		if (remaining > 0) {
			memmove(_scan_buffer, _scan_buffer + packet_end, remaining);
		}
		_scan_buffer_len = remaining;
	}
}

void YDLidarG2::update_obstacle_distance(float distance_m, float angle_deg)
{
	// Update obstacle distance map with all 360 measurements
	for (int i = 0; i < 360; i++) {
		float angle = (float)i; // 0 to 359 degrees
		uint8_t bin = convert_angle_to_bin(angle);

		// Convert distance from mm to cm for obstacle map
		uint16_t distance_cm = _distances[i] / 10;

		// Clamp to valid range
		if (distance_cm < 12) distance_cm = 0;
		if (distance_cm > 1200) distance_cm = 0;

		_obstacle_map_msg.distances[bin] = distance_cm;
	}

	_obstacle_map_msg.timestamp = hrt_absolute_time();
	_obstacle_distance_pub.publish(_obstacle_map_msg);
}

uint8_t YDLidarG2::convert_angle_to_bin(float angle_deg)
{
	// Convert angle (0-359 degrees) to obstacle map bin (0-359)
	// YDLidar G2 starts at 0 degrees and goes clockwise
	int bin = (int)roundf(angle_deg) % 360;
	if (bin < 0) bin += 360;
	return (uint8_t)bin;
}

void YDLidarG2::print_info()
{
	PX4_INFO("YDLidar G2 driver status:");
	PX4_INFO("  Port: %s", _port);
	PX4_INFO("  UART fd: %d", _fd);
	PX4_INFO("  Motor started: %s", _motor_started ? "Yes" : "No");
	PX4_INFO("  Scan started: %s", _scan_started ? "Yes" : "No");
	PX4_INFO("  Start attempts: %d", _start_attempts);
}
