// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <cmath>
#include <cstdint>

namespace range_sensor
{
enum class Role : uint8_t { NONE, SHORT, LONG };

// Device IDs are opaque identities here: never decode their bus or model.
// Consumers take a snapshot at startup; changing assignments needs a reboot.
struct Roles {
	uint32_t short_id{0};
	uint32_t long_id{0};

	bool configured() const { return short_id != 0 || long_id != 0; }
	bool valid() const { return short_id == 0 || short_id != long_id; }
	bool allowEmbeddedFlowRange() const { return !configured(); }

	Role classify(uint32_t device_id) const
	{
		if (!valid() || device_id == 0) { return Role::NONE; }

		if (device_id == short_id) { return Role::SHORT; }

		if (device_id == long_id) { return Role::LONG; }

		return Role::NONE;
	}
};

// Resolve one usable publisher, independent of advertisement order. Two live
// publishers with the same assigned ID are ambiguous, not interchangeable.
class UniqueInstance
{
public:
	void add(int index)
	{
		if (_index >= 0) { _ambiguous = true; }

		_index = index;
	}

	int index() const { return _ambiguous ? -1 : _index; }
	bool ambiguous() const { return _ambiguous; }
private:
	int _index{-1};
	bool _ambiguous{false};
};

inline bool validSample(uint64_t now, uint64_t timestamp, uint64_t timeout,
			float distance, float minimum, float maximum, int quality)
{
	// A publisher can run just after the triggering IMU sample was captured.
	const bool fresh = timestamp > now ? timestamp - now <= 100000 : now - timestamp <= timeout;
	return timestamp != 0 && fresh
	       && quality != 0 && std::isfinite(distance) && std::isfinite(minimum)
	       && std::isfinite(maximum) && minimum >= 0.f && maximum > minimum
	       && distance >= minimum && distance <= maximum;
}
}
