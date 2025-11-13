/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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
 * @file aux_edge_counter.cpp
 * Quadrature Encoder Driver for Motor Position Tracking
 * Supports A/B channel encoder with direction detection
 *
 * @author PX4 Development Team
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <drivers/drv_hrt.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/wheel_encoders.h>

#include <board_config.h>

extern "C" __EXPORT int aux_edge_counter_main(int argc, char *argv[]);

class AuxEdgeCounter : public ModuleBase<AuxEdgeCounter>, public px4::ScheduledWorkItem
{
public:
	AuxEdgeCounter();
	~AuxEdgeCounter() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::print_status() */
	int print_status() override;

	void Run() override;

	bool init();

private:
	static int gpio_interrupt_callback_a(int irq, void *context, void *arg);
	static int gpio_interrupt_callback_b(int irq, void *context, void *arg);
	void handle_interrupt_a();
	void handle_interrupt_b();

	uint32_t _gpio_pin_a{0};  // Channel A (AUX6)
	uint32_t _gpio_pin_b{0};  // Channel B (AUX7)
	
	volatile int32_t _encoder_position{0};  // Signed position (can go negative)
	volatile uint32_t _total_pulses{0};     // Total pulse count (always positive)
	
	hrt_abstime _last_edge_time{0};
	hrt_abstime _last_publish_time{0};
	
	bool _initialized{false};
	bool _last_a_state{false};
	bool _last_b_state{false};
	
	// Encoder parameters
	static constexpr uint32_t PULSES_PER_REVOLUTION = 360;  // Adjust based on your encoder
	static constexpr float WHEEL_RADIUS_M = 0.05f;          // 5cm wheel radius (adjust!)
	
	// uORB publisher for wheel encoder data
	uORB::Publication<wheel_encoders_s> _wheel_encoders_pub{ORB_ID(wheel_encoders)};
};

AuxEdgeCounter::AuxEdgeCounter() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

AuxEdgeCounter::~AuxEdgeCounter()
{
	// Unregister interrupts
	if (_initialized) {
		if (_gpio_pin_a != 0) {
			px4_arch_gpiosetevent(_gpio_pin_a, false, false, false, nullptr, nullptr);
		}
		if (_gpio_pin_b != 0) {
			px4_arch_gpiosetevent(_gpio_pin_b, false, false, false, nullptr, nullptr);
		}
	}
}

int AuxEdgeCounter::gpio_interrupt_callback_a(int irq, void *context, void *arg)
{
	AuxEdgeCounter *dev = reinterpret_cast<AuxEdgeCounter *>(arg);

	if (dev != nullptr) {
		dev->handle_interrupt_a();
	}
	
	return OK;
}

int AuxEdgeCounter::gpio_interrupt_callback_b(int irq, void *context, void *arg)
{
	AuxEdgeCounter *dev = reinterpret_cast<AuxEdgeCounter *>(arg);

	if (dev != nullptr) {
		dev->handle_interrupt_b();
	}
	
	return OK;
}

void AuxEdgeCounter::handle_interrupt_a()
{
	hrt_abstime now = hrt_absolute_time();
	
	// Debounce: minimum 50us between edges
	if ((now - _last_edge_time) < 50) {
		return;
	}
	
	_last_edge_time = now;
	
	// Read both channel states
	bool a_state = px4_arch_gpioread(_gpio_pin_a);
	bool b_state = px4_arch_gpioread(_gpio_pin_b);
	
	// Quadrature decoding logic
	// When A rises:
	//   - If B is LOW  → CW  (forward)  → increment
	//   - If B is HIGH → CCW (backward) → decrement
	
	if (a_state && !_last_a_state) {  // Rising edge on A
		if (!b_state) {
			_encoder_position++;  // Clockwise (forward)
		} else {
			_encoder_position--;  // Counter-clockwise (backward)
		}
		_total_pulses++;
	}
	
	_last_a_state = a_state;
}

void AuxEdgeCounter::handle_interrupt_b()
{
	// Optional: Can also decode on B channel for 4x resolution
	// For now, we only use A channel for 1x resolution
	bool b_state = px4_arch_gpioread(_gpio_pin_b);
	_last_b_state = b_state;
}

bool AuxEdgeCounter::init()
{
	// Use AUX6 (FMU_CH6) for Channel A
	// Use AUX7 (FMU_CH7) for Channel B
	
#ifdef GPIO_FMU_CH6
	_gpio_pin_a = GPIO_FMU_CH6;
#else
	PX4_ERR("GPIO_FMU_CH6 not defined for this board");
	return false;
#endif

#ifdef GPIO_FMU_CH7
	_gpio_pin_b = GPIO_FMU_CH7;
#else
	PX4_ERR("GPIO_FMU_CH7 not defined for this board");
	return false;
#endif

	// Configure pins as input
	px4_arch_configgpio(_gpio_pin_a);
	px4_arch_configgpio(_gpio_pin_b);
	
	// Read initial states
	_last_a_state = px4_arch_gpioread(_gpio_pin_a);
	_last_b_state = px4_arch_gpioread(_gpio_pin_b);

	// Register interrupt on both edges of Channel A
	int ret_a = px4_arch_gpiosetevent(_gpio_pin_a, true, true, false, 
	                                   &AuxEdgeCounter::gpio_interrupt_callback_a, this);

	if (ret_a != OK) {
		PX4_ERR("Failed to register GPIO interrupt on Channel A");
		return false;
	}
	
	// Register interrupt on Channel B (optional, for better noise immunity)
	int ret_b = px4_arch_gpiosetevent(_gpio_pin_b, true, true, false, 
	                                   &AuxEdgeCounter::gpio_interrupt_callback_b, this);

	if (ret_b != OK) {
		PX4_WARN("Failed to register GPIO interrupt on Channel B (continuing anyway)");
	}

	_initialized = true;
	_last_publish_time = hrt_absolute_time();
	
	PX4_INFO("Quadrature Encoder initialized");
	PX4_INFO("Channel A: AUX6 (FMU_CH6)");
	PX4_INFO("Channel B: AUX7 (FMU_CH7)");
	PX4_INFO("Pulses per revolution: %lu", (unsigned long)PULSES_PER_REVOLUTION);
	PX4_INFO("Wheel radius: %.3f m", (double)WHEEL_RADIUS_M);

	return true;
}

void AuxEdgeCounter::Run()
{
	if (!_initialized) {
		return;
	}
	
	hrt_abstime now = hrt_absolute_time();
	float dt = (now - _last_publish_time) / 1e6f;  // seconds
	
	// Publish wheel encoder data to uORB
	wheel_encoders_s wheel_enc{};
	wheel_enc.timestamp = now;
	
	// Calculate wheel position in radians
	float position_rad = (2.0f * M_PI_F * _encoder_position) / (float)PULSES_PER_REVOLUTION;
	
	// Calculate wheel speed (rad/s)
	static int32_t last_position = 0;
	int32_t delta_position = _encoder_position - last_position;
	float delta_rad = (2.0f * M_PI_F * delta_position) / (float)PULSES_PER_REVOLUTION;
	float speed_rad_s = delta_rad / dt;
	
	// Calculate linear velocity (m/s)
	float linear_velocity = speed_rad_s * WHEEL_RADIUS_M;
	
	wheel_enc.wheel_speed[0] = speed_rad_s;      // Left wheel (or single wheel)
	wheel_enc.wheel_angle[0] = position_rad;     // Wheel angle
	wheel_enc.wheel_speed[1] = 0.0f;             // Right wheel (unused for single encoder)
	wheel_enc.wheel_angle[1] = 0.0f;
	
	_wheel_encoders_pub.publish(wheel_enc);
	
	// Print status
	PX4_INFO("Pos: %ld | Total: %lu | Rad: %.2f | Speed: %.2f rad/s | Vel: %.3f m/s",
	         (long)_encoder_position,
	         (unsigned long)_total_pulses,
	         (double)position_rad,
	         (double)speed_rad_s,
	         (double)linear_velocity);
	
	last_position = _encoder_position;
	_last_publish_time = now;
}

int AuxEdgeCounter::task_spawn(int argc, char *argv[])
{
	AuxEdgeCounter *instance = new AuxEdgeCounter();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;

	if (!instance->init()) {
		delete instance;
		_object.store(nullptr);
		_task_id = -1;
		return PX4_ERROR;
	}

	// Schedule to run at 10 Hz (100ms interval)
	instance->ScheduleOnInterval(100000);

	return PX4_OK;
}

int AuxEdgeCounter::custom_command(int argc, char *argv[])
{
	if (!strcmp(argv[0], "reset")) {
		AuxEdgeCounter *instance = (AuxEdgeCounter *)_object.load();
		
		if (instance != nullptr) {
			instance->_encoder_position = 0;
			instance->_total_pulses = 0;
			PX4_INFO("Encoder position reset to 0");
			return 0;
		}
		
		return -1;
	}

	return print_usage("unknown command");
}

int AuxEdgeCounter::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Quadrature encoder driver for motor position tracking.
Uses AUX6 (Channel A) and AUX7 (Channel B) for direction detection.
Publishes wheel_encoders uORB topic for odometry feedback.

### Wiring
- AUX6 (FMU_CH6): Encoder Channel A
- AUX7 (FMU_CH7): Encoder Channel B
- GND: Common ground

### Configuration
Edit the driver source to adjust:
- PULSES_PER_REVOLUTION: Encoder pulses per wheel rotation
- WHEEL_RADIUS_M: Wheel radius in meters

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("aux_edge_counter", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND_DESCR("reset", "Reset encoder position to 0");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int AuxEdgeCounter::print_status()
{
	PX4_INFO("Running");
	PX4_INFO("Channel A Pin: %lu", (unsigned long)_gpio_pin_a);
	PX4_INFO("Channel B Pin: %lu", (unsigned long)_gpio_pin_b);
	PX4_INFO("Encoder Position: %ld", (long)_encoder_position);
	PX4_INFO("Total Pulses: %lu", (unsigned long)_total_pulses);
	PX4_INFO("Last Edge: %llu us", _last_edge_time);
	
	float position_rad = (2.0f * M_PI_F * _encoder_position) / (float)PULSES_PER_REVOLUTION;
	float revolutions = (float)_encoder_position / (float)PULSES_PER_REVOLUTION;
	
	PX4_INFO("Position (rad): %.2f", (double)position_rad);
	PX4_INFO("Revolutions: %.2f", (double)revolutions);

	return 0;
}

int aux_edge_counter_main(int argc, char *argv[])
{
	return AuxEdgeCounter::main(argc, argv);
}
