// SPDX-License-Identifier: BSD-3-Clause
#pragma once
#include <uavcan/driver/can.hpp>

// Expose physical CAN2 as logical interface 0 to DroneCAN. Native CAN1 is
// deliberately absent from both the receive and transmit interface masks.
class NRA24DroneCanPort : public uavcan::ICanDriver
{
public:
	explicit NRA24DroneCanPort(uavcan::ICanDriver &driver) : _driver(driver) {}
	uavcan::ICanIface *getIface(uavcan::uint8_t index) override
	{
		return index == 0 ? _driver.getIface(1) : nullptr;
	}
	uavcan::uint8_t getNumIfaces() const override { return 1; }
	uavcan::int16_t select(uavcan::CanSelectMasks &masks,
			       const uavcan::CanFrame * (&pending)[uavcan::MaxCanIfaces],
			       uavcan::MonotonicTime deadline) override
	{
		uavcan::CanSelectMasks physical;
		physical.read = (masks.read & 1U) << 1U;
		physical.write = (masks.write & 1U) << 1U;
		const uavcan::CanFrame *physical_pending[uavcan::MaxCanIfaces] {};
		physical_pending[1] = pending[0];
		const auto result = _driver.select(physical, physical_pending, deadline);
		masks.read = (physical.read >> 1U) & 1U;
		masks.write = (physical.write >> 1U) & 1U;
		return result;
	}
private:
	uavcan::ICanDriver &_driver;
};
