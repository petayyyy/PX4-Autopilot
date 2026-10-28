// SPDX-License-Identifier: BSD-3-Clause
#include "NRA24CanPort.hpp"
#include <cassert>
#include <cstdio>

class FakeIface : public uavcan::ICanIface
{
public:
	unsigned sent{0};
	uavcan::int16_t send(const uavcan::CanFrame &, uavcan::MonotonicTime, uavcan::CanIOFlags) override { ++sent; return 1; }
	uavcan::int16_t receive(uavcan::CanFrame &, uavcan::MonotonicTime &, uavcan::UtcTime &, uavcan::CanIOFlags &) override { return 0; }
	uavcan::int16_t configureFilters(const uavcan::CanFilterConfig *, uavcan::uint16_t) override { return 0; }
	uavcan::uint16_t getNumFilters() const override { return 14; }
	uavcan::uint64_t getErrorCount() const override { return 0; }
};
class FakeDriver : public uavcan::ICanDriver
{
public:
	FakeIface ports[2];
	const uavcan::CanFrame *expected{nullptr};
	uavcan::ICanIface *getIface(uavcan::uint8_t index) override { return index < 2 ? &ports[index] : nullptr; }
	uavcan::uint8_t getNumIfaces() const override { return 2; }
	uavcan::int16_t select(uavcan::CanSelectMasks &masks,
			       const uavcan::CanFrame * (&pending)[uavcan::MaxCanIfaces], uavcan::MonotonicTime) override
	{
		assert(masks.read == 2 && masks.write == 2);
		assert(pending[0] == nullptr && pending[1] == expected);
		masks.read = 3;
		masks.write = 3;
		return 1;
	}
};
int main()
{
	FakeDriver physical;
	NRA24DroneCanPort dronecan(physical);
	assert(dronecan.getNumIfaces() == 1);
	assert(dronecan.getIface(0) == &physical.ports[1]);
	assert(dronecan.getIface(1) == nullptr);
	uavcan::CanFrame frame;
	frame.id = 123;
	assert(dronecan.getIface(0)->send(frame, uavcan::MonotonicTime(), 0) == 1);
	assert(physical.ports[0].sent == 0 && physical.ports[1].sent == 1);
	physical.expected = &frame;
	const uavcan::CanFrame *pending[uavcan::MaxCanIfaces] {};
	pending[0] = &frame;
	uavcan::CanSelectMasks masks;
	masks.read = masks.write = 1;
	assert(dronecan.select(masks, pending, uavcan::MonotonicTime()) == 1);
	assert(masks.read == 1 && masks.write == 1);
	std::puts("PASS DroneCAN logical interface maps only to physical CAN2; TX and select masks exclude native CAN1");
}
