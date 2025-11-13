# DIR_OUT Module

This module converts RC input to PWM direction outputs for motor control systems.

## Overview

The `dir_out` module reads RC channels and generates direction PWM signals on AUX outputs. This is useful for controlling brushed DC motors or stepper motors that require separate direction and speed signals.

**AUTOMATIC STARTUP**: This module starts automatically when the board boots up - no manual intervention required!

## Features

- **Drive Direction**: Forward/Reverse control via throttle stick
- **Steer Direction**: Left/Right control via roll stick (optional)
- **Independent Control**: Both directions work independently
- **Safety Features**: Deadband, switch delay, center detection
- **Configurable**: All parameters adjustable via QGC

## Pin Mapping (PX4 FMU-v6x)

### AUX Output Pins:
- **AUX1**: PI0 (Physical Pin) - **DEFAULT DRIVE DIRECTION OUTPUT**
- **AUX2**: PH12 (Physical Pin) - **DEFAULT STEER DIRECTION OUTPUT**
- **AUX3**: PH11 (Physical Pin)
- **AUX4**: PH10 (Physical Pin)
- **AUX5**: PD13 (Physical Pin)
- **AUX6**: PD14 (Physical Pin)
- **AUX7**: PH6 (Physical Pin)
- **AUX8**: PH9 (Physical Pin)

### MAIN Output Pins:
- **MAIN1**: PI0 (Physical Pin) - **DEFAULT SPEED OUTPUT**
- **MAIN2**: PH12 (Physical Pin)
- **MAIN3**: PH11 (Physical Pin)
- **MAIN4**: PH10 (Physical Pin)
- **MAIN5**: PD13 (Physical Pin)
- **MAIN6**: PD14 (Physical Pin)
- **MAIN7**: PH6 (Physical Pin)
- **MAIN8**: PH9 (Physical Pin)

## Parameters

### Drive Direction (Throttle):
- `DIR_RC_MAP_THR`: RC channel for throttle (default: -1 = use RC_MAP_THROTTLE)
- `DIR_OUT_BUS`: Output bus (0=MAIN, 1=AUX, default: 1)
- `DIR_OUT_CH`: Output channel (1-8, default: 1)
- `DIR_DB_US`: Deadband in microseconds (default: 40)
- `DIR_HIGH_PWM`: Forward PWM value (default: 2000)
- `DIR_LOW_PWM`: Reverse PWM value (default: 1000)
- `DIR_SW_DELAY`: Direction change delay in ms (default: 150)
- `DIR_POLARITY`: Polarity (1=forward=HIGH, 0=forward=LOW, default: 1)

### Steer Direction (Roll):
- `SDIR_RC_MAP`: RC channel for steer (default: -1 = use RC_MAP_ROLL)
- `SDIR_OUT_BUS`: Output bus (0=MAIN, 1=AUX, default: 1)
- `SDIR_OUT_CH`: Output channel (1-8, default: 0 = disabled)
- `SDIR_DB_US`: Deadband in microseconds (default: 40)
- `SDIR_HIGH_PWM`: Right PWM value (default: 2000)
- `SDIR_LOW_PWM`: Left PWM value (default: 1000)
- `SDIR_SW_DELAY`: Direction change delay in ms (default: 150)
- `SDIR_POLARITY`: Polarity (1=right=HIGH, 0=right=LOW, default: 1)

## Usage

**NO MANUAL CONFIGURATION REQUIRED!** The module starts automatically.

### Default Configuration:
- **Drive Direction**: AUX1 (PI0 pin) - 1000us (reverse) / 2000us (forward)
- **Steer Direction**: AUX2 (PH12 pin) - 1000us (left) / 2000us (right) - **DISABLED BY DEFAULT**
- **Speed Output**: MAIN1 (PI0 pin) - 1000-2000us based on RC throttle
- **RC Input**: Uses system RC_MAP_THROTTLE and RC_MAP_ROLL settings

### QGC Parameters (Optional):
- Set `RC_MAP_THROTTLE` to the correct RC channel
- Set `RC_MAP_ROLL` to the correct RC channel (for steer)
- Set `PWM_MAIN_FUNC1 = Motor1` for speed control
- Set `PWM_AUX_FUNC1 = Disabled` for drive direction control
- Set `PWM_AUX_FUNC2 = Disabled` for steer direction control
- Configure PWM ranges: `PWM_MAIN_MIN=1000`, `PWM_MAIN_MAX=2000`, `PWM_MAIN_DISARMED=1500`

### Testing:
- **Forward stick**: MAIN1 outputs 1500-2000us, AUX1 outputs 2000us
- **Reverse stick**: MAIN1 outputs 1000-1500us, AUX1 outputs 1000us
- **Right stick**: AUX2 outputs 2000us (if enabled)
- **Left stick**: AUX2 outputs 1000us (if enabled)
- **Center stick**: AUX outputs maintain last direction

## Hardware Setup

- **MAIN1 output (PI0)**: Connect to motor speed controller (PWM to 0-5V converter)
- **AUX1 output (PI0)**: Connect to drive direction control (PWM to HIGH/LOW converter)
- **AUX2 output (PH12)**: Connect to steer direction control (PWM to HIGH/LOW converter)
- External gate circuit needed to convert PWM to solid HIGH/LOW signals

## RC Channel Mapping

### Drive (Throttle):
1. If `DIR_RC_MAP_THR >= 0`: Use that channel
2. Else: Use `RC_MAP_THROTTLE` system parameter
3. Fallback: Channel 1

### Steer (Roll):
1. If `SDIR_RC_MAP >= 0`: Use that channel
2. Else: Use `RC_MAP_ROLL` system parameter
3. Fallback: Use `RC_MAP_YAW` system parameter
4. If `SDIR_OUT_CH = 0`: Steer disabled

## Automatic Operation

The module automatically:
1. Starts when the board boots
2. Reads RC throttle and roll input
3. Generates drive direction PWM on AUX1
4. Generates steer direction PWM on AUX2 (if enabled)
5. Generates speed PWM on MAIN1
6. Handles direction changes with safety delays
7. Maintains direction during center stick position
8. Works independently for drive and steer directions
