// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 PX4 Development Team.
#pragma once
#include <cmath>
#include <cstdint>

struct RadarDistanceEncoding {
	uint16_t centimetres;
	uint8_t signal_quality;
};

inline RadarDistanceEncoding encode_radar_distance(float metres, int8_t quality)
{
	// MAVLink has an integer distance field. Use an out-of-range placeholder
	// plus explicit invalid quality, rather than casting a NaN or emitting zero.
	if (!std::isfinite(metres) || metres < 0.f || metres > 655.34f || quality == 0) {
		return {UINT16_MAX, 1};
	}

	// uORB: -1 unknown, 0 invalid; MAVLink: 0 unknown, 1 invalid.
	const uint8_t encoded_quality = quality < 0 ? 0 : (quality < 2 ? 2 : (quality > 100 ? 100 : quality));
	return {uint16_t(metres * 100.f + 0.5f), encoded_quality};
}

inline uint8_t encode_radar_covariance(float variance_m2)
{
	if (!std::isfinite(variance_m2) || variance_m2 <= 0.f) { return UINT8_MAX; }

	const float cm2 = variance_m2 * 10000.f;
	return cm2 < 254.f ? uint8_t(cm2) : 254;
}
