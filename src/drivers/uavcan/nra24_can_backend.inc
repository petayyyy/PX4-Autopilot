// SPDX-License-Identifier: BSD-3-Clause
// Included after UavcanNode's shared CanInitHelper declaration.
#include "nra24_can_backend.h"
#include "NRA24CanPort.hpp"
#include <cerrno>
#include <pthread.h>

static pthread_mutex_t native_can_mutex = PTHREAD_MUTEX_INITIALIZER;
static const void *native_can_owner = nullptr;
static bool native_can_initialized = false;
static bool native_can_split = false;
static uint32_t native_can_bitrate = 0;
static NRA24DroneCanPort *native_dronecan_port = nullptr;

class NativeCanLock
{
public:
	NativeCanLock() { pthread_mutex_lock(&native_can_mutex); }
	~NativeCanLock() { pthread_mutex_unlock(&native_can_mutex); }
	NativeCanLock(const NativeCanLock &) = delete;
	NativeCanLock &operator=(const NativeCanLock &) = delete;
};

// Both controllers share an RCC reset and message RAM. Initialize them once,
// with separate bit timings, before either consumer can start. Starting or
// stopping a module afterwards must never reset the shared CAN peripheral.
static int nra24_can_initialize()
{
	if (native_can_initialized) { return 0; }

	int32_t enabled = 0, radar_bitrate = 500000, dronecan_bitrate = 1000000;

	if (param_get(param_find("SENS_NRA24_EN"), &enabled) != 0
	    || param_get(param_find("NRA24_BITRATE"), &radar_bitrate) != 0
	    || param_get(param_find("UAVCAN_BITRATE"), &dronecan_bitrate) != 0) { return -EINVAL; }

	if (radar_bitrate != 125000 && radar_bitrate != 250000
	    && radar_bitrate != 500000 && radar_bitrate != 1000000) { return -EINVAL; }

	const uint32_t interfaces = board_get_can_interfaces();

	if ((interfaces & 3U) != 3U) { return -ENODEV; }

	if (!can) { can = new UavcanNode::CanInitHelper(interfaces); }

	if (!can) { return -ENOMEM; }

	if (enabled && !native_dronecan_port) { native_dronecan_port = new NRA24DroneCanPort(can->driver); }

	if (enabled && !native_dronecan_port) { return -ENOMEM; }

	(void)UAVCAN_DRIVER::SystemClock::instance();
	const int result = enabled
			   ? can->driver.init(radar_bitrate, UAVCAN_DRIVER::CanIface::NormalMode, interfaces, dronecan_bitrate)
			   : can->init(dronecan_bitrate);

	if (result < 0) { return result; }

	native_can_split = enabled != 0;
	native_can_bitrate = radar_bitrate;
	native_can_initialized = true;
	return 0;
}

static uavcan::ICanDriver *nra24_can_prepare_dronecan()
{
	NativeCanLock lock;
	const int result = nra24_can_initialize();

	if (result < 0) { PX4_ERR("shared CAN initialization: %d", result); return nullptr; }

	return native_can_split ? static_cast<uavcan::ICanDriver *>(native_dronecan_port) : &can->driver;
}

int nra24_can_acquire(const void *owner, uint32_t bitrate)
{
	NativeCanLock lock;
	int32_t enabled = 0;

	if (!owner || param_get(param_find("SENS_NRA24_EN"), &enabled) != 0 || !enabled) { return -EINVAL; }

	if (native_can_owner) { return -EBUSY; }

	const int result = nra24_can_initialize();

	if (result < 0) { return result; }

	if (!native_can_split || bitrate != native_can_bitrate) { return -EBUSY; }

	// Discard queued frames from a previous stopped instance, not the hardware
	// state. CAN2 remains operational throughout a radar stop/start cycle.
	uavcan::ICanIface *iface = can->driver.getIface(0);
	uavcan::CanFrame frame;
	uavcan::MonotonicTime monotonic;
	uavcan::UtcTime utc;
	uavcan::CanIOFlags flags = 0;

	for (unsigned i = 0; i < 64 && iface->receive(frame, monotonic, utc, flags) > 0; ++i) {}

	native_can_owner = owner;
	return 0;
}

int nra24_can_receive(const void *owner, nra24_can_frame &output)
{
	NativeCanLock lock;

	if (!owner || owner != native_can_owner || !can || !native_can_split) { return -EPERM; }

	uavcan::ICanIface *iface = can->driver.getIface(0);

	if (!iface) { return -ENODEV; }

	uavcan::CanFrame frame;
	uavcan::MonotonicTime monotonic;
	uavcan::UtcTime utc;
	uavcan::CanIOFlags flags = 0;
	const int result = iface->receive(frame, monotonic, utc, flags);

	if (result > 0) {
		output.id = frame.id & uavcan::CanFrame::MaskExtID;
		output.length = frame.dlc;
		output.flags = (frame.isExtended() ? 1 : 0) | (frame.isRemoteTransmissionRequest() ? 2 : 0)
			       | (frame.isErrorFrame() ? 4 : 0);
		memcpy(output.data, frame.data, sizeof(output.data));
	}

	return result;
}

void nra24_can_release(const void *owner)
{
	NativeCanLock lock;

	if (owner && native_can_owner == owner) { native_can_owner = nullptr; }

	// CAN1 remains reserved, receiving and ACKing, until reboot. In particular,
	// do not reset FDCAN here: it would also disconnect the H-Flow on CAN2.
}

int nra24_can_counters(const void *owner, uint64_t &errors, uint32_t &overflows)
{
	NativeCanLock lock;

	if (!owner || owner != native_can_owner || !can || !can->driver.getIface(0)) { return -EPERM; }

	auto *iface = can->driver.getIface(0);
	errors = iface->getErrorCount();
	overflows = iface->getRxQueueOverflowCount();
	return 0;
}
