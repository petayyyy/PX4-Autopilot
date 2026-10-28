// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 PX4 Development Team.
#include <px4_platform_common/param.h>

/**
 * Enable NRA24 native CAN rangefinder
 *
 * Reserves CAN1 for native NRA24 frames. DroneCAN remains available on CAN2.
 * Configure bitrate, radar ID, limits and orientation before enabling; reboot
 * is required to change the CAN port allocation or bitrate.
 *
 * @boolean
 * @reboot_required true
 * @group Sensors
 */
PARAM_DEFINE_INT32(SENS_NRA24_EN, 0);

/**
 * NRA24 radar ID
 *
 * @min 0
 * @max 15
 * @reboot_required true
 * @group NRA24
 */
PARAM_DEFINE_INT32(NRA24_ID, 0);

/**
 * NRA24 CAN bitrate
 *
 * @value 125000 125 kbit/s
 * @value 250000 250 kbit/s
 * @value 500000 500 kbit/s
 * @value 1000000 1 Mbit/s
 * @reboot_required true
 * @group NRA24
 */
PARAM_DEFINE_INT32(NRA24_BITRATE, 500000);

/**
 * NRA24 minimum valid distance
 *
 * @unit m
 * @min 0.1
 * @max 10
 * @decimal 2
 * @reboot_required true
 * @group NRA24
 */
PARAM_DEFINE_FLOAT(NRA24_MIN, 0.5f);

/**
 * NRA24 maximum valid distance
 *
 * Set according to the actual radar variant and application.
 *
 * @unit m
 * @min 1
 * @max 200
 * @decimal 1
 * @reboot_required true
 * @group NRA24
 */
PARAM_DEFINE_FLOAT(NRA24_MAX, 200.0f);

/**
 * NRA24 mounting orientation
 *
 * @value 0 Forward
 * @value 2 Right
 * @value 4 Backward
 * @value 6 Left
 * @value 24 Upward
 * @value 25 Downward
 * @min 0
 * @max 25
 * @reboot_required true
 * @group NRA24
 */
PARAM_DEFINE_INT32(NRA24_ROT, 25);

/**
 * NRA24 range and heartbeat timeout
 *
 * Zero-target status invalidates a reading immediately, independently of timeout.
 *
 * @unit ms
 * @min 50
 * @max 2000
 * @reboot_required true
 * @group NRA24
 */
PARAM_DEFINE_INT32(NRA24_TIMEOUT, 200);
