// SPDX-License-Identifier: BSD-3-Clause
#include "RangeSourceSelection.hpp"
#include "FlowSourceSelection.hpp"
#include <cassert>
#include <cstdio>
using namespace range_selection;
enum class Flow { NONE, CAN, MAV };
int main()
{
	assert(valid(1000000, 990000, 300000, 2.f, .1f, 200.f, -1));
	assert(!valid(1000000, 990000, 300000, NAN, .1f, 200.f, -1));
	assert(!valid(1000000, 990000, 300000, INFINITY, .1f, 200.f, 100));
	assert(!valid(1000000, 990000, 300000, 2.f, .1f, 200.f, 0));
	assert(!valid(1000000, 600000, 300000, 2.f, .1f, 200.f, 100));
	assert(valid(1000000, 1000001, 300000, 2.f, .1f, 200.f, 100));
	assert(!valid(1000000, 1100001, 300000, 2.f, .1f, 200.f, 100));
	assert(!valid(1000000, 990000, 300000, .05f, .1f, 200.f, 100));
	assert(!valid(1000000, 990000, 300000, 201.f, .1f, 200.f, 100));
	assert(!valid(1000000, 990000, 300000, 2.f, NAN, 200.f, 100));
	Selector selector;
	Candidate short_sensor{0, 1000000, 2.f}, radar{1, 1000000, 2.1f}, none{};
	assert(selector.choose(1000000, short_sensor, radar, 20.f, 2.f).index == 0);
	short_sensor.distance = 20.f;
	assert(selector.choose(1100000, short_sensor, radar, 20.f, 2.f).index == 0);
	short_sensor.distance = 20.1f;
	assert(selector.choose(1200000, short_sensor, radar, 20.f, 2.f).index == 1);
	short_sensor.distance = 19.f;
	assert(selector.choose(1300000, short_sensor, radar, 20.f, 2.f).index == 1);
	short_sensor.distance = 17.9f;
	assert(selector.choose(1400000, short_sensor, radar, 20.f, 2.f).index == 1);
	assert(selector.choose(2300000, short_sensor, radar, 20.f, 2.f).index == 0);
	assert(selector.choose(2400000, none, radar, 20.f, 2.f).index == 1);
	// If radar loses targets, use a valid short sensor even in the hysteresis band.
	short_sensor.distance = 19.f;
	assert(selector.choose(2500000, short_sensor, none, 20.f, 2.f).index == 0);
	assert(selector.choose(2600000, none, none, 20.f, 2.f).index == -1);
	assert(selector.choose(2700000, none, radar, 20.f, 2.f).source == Source::LONG_RANGE);

	using flow_selection::choose;
	// Explicit lock must never substitute MAVLink flow, even when fresh.
	assert(choose(Flow::CAN, Flow::MAV, Flow::NONE, false, true, false, true) == Flow::NONE);
	assert(choose(Flow::CAN, Flow::MAV, Flow::MAV, false, true, false, true) == Flow::NONE);
	assert(choose(Flow::CAN, Flow::MAV, Flow::MAV, true, true, false, false) == Flow::CAN);
	assert(choose(Flow::CAN, Flow::MAV, Flow::CAN, true, true, false, true) == Flow::CAN);
	// Preserve the branch's optional failover mode for other configurations.
	assert(choose(Flow::CAN, Flow::MAV, Flow::CAN, false, true, true, true) == Flow::MAV);
	assert(choose(Flow::CAN, Flow::MAV, Flow::MAV, true, true, true, false) == Flow::MAV);
	assert(choose(Flow::CAN, Flow::MAV, Flow::MAV, true, true, true, true) == Flow::CAN);
	assert(choose(Flow::CAN, Flow::MAV, Flow::MAV, false, false, true, true) == Flow::NONE);
	std::puts("PASS range boundaries, hysteresis, immediate failure fallback, invalid/stale rejection; H-Flow-only lock and optional flow failover");
}
