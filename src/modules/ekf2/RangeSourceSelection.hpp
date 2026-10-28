// SPDX-License-Identifier: BSD-3-Clause
#pragma once
#include <cmath>
#include <cstdint>

namespace range_selection
{
enum class Source : uint8_t { NONE, SHORT_RANGE, LONG_RANGE };
struct Candidate {
	int index{-1};
	uint64_t timestamp{0};
	float distance{NAN};
};
inline bool valid(uint64_t now, uint64_t timestamp, uint64_t timeout,
		  float distance, float minimum, float maximum, int quality)
{
	// A newly published sample can be slightly newer than the triggering IMU
	// sample (or be published between reading 'now' and copying the topic).
	const bool fresh = timestamp > now ? timestamp - now <= 100000 : now - timestamp <= timeout;
	return timestamp != 0 && fresh
	       && quality != 0 && std::isfinite(distance) && std::isfinite(minimum)
	       && std::isfinite(maximum) && minimum >= 0.f && maximum > minimum
	       && distance >= minimum && distance <= maximum;
}
struct Choice {
	Source source{Source::NONE};
	int index{-1};
};
class Selector
{
public:
	Choice choose(uint64_t now, const Candidate &short_sensor, const Candidate &long_sensor,
		      float short_limit, float hysteresis)
	{
		const float difference = short_limit - hysteresis;
		const float reentry = difference > 0.f ? difference : 0.f;
		const bool short_ok = short_sensor.index >= 0 && short_sensor.distance <= short_limit;
		const bool long_ok = long_sensor.index >= 0;
		Choice desired;

		if (_choice.source == Source::LONG_RANGE && long_ok) {
			desired = short_ok && short_sensor.distance <= reentry
				  ? Choice{Source::SHORT_RANGE, short_sensor.index}
				  : Choice{Source::LONG_RANGE, long_sensor.index};

		} else if (short_ok) {
			desired = Choice{Source::SHORT_RANGE, short_sensor.index};

		} else if (long_ok) {
			desired = Choice{Source::LONG_RANGE, long_sensor.index};
		}

		const bool current_healthy =
			(_choice.source == Source::SHORT_RANGE && short_ok && _choice.index == short_sensor.index)
			|| (_choice.source == Source::LONG_RANGE && long_ok && _choice.index == long_sensor.index);

		if (desired.index != _choice.index || desired.source != _choice.source) {
			if (!current_healthy || now - _last_switch >= 1000000) {
				_choice = desired;
				_last_switch = now;
			}
		}

		return _choice;
	}
private:
	Choice _choice{};
	uint64_t _last_switch{0};
};
}
