// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 PX4 Development Team.

#include "NRA24Parser.hpp"
#include <nra24_can_backend.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_sensor.h>
#include <lib/drivers/device/Device.hpp>
#include <lib/drivers/rangefinder/PX4Rangefinder.hpp>
#include <px4_platform_common/module.h>
#include <px4_platform_common/param.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_armed.h>
#include <inttypes.h>

class NRA24Can : public ModuleBase<NRA24Can>, public px4::ScheduledWorkItem
{
public:
	NRA24Can() : ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::uavcan) {}
	~NRA24Can() override { nra24_can_release(this); }
	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]) { return print_usage("unknown command"); }
	static int print_usage(const char *reason = nullptr);
	int print_status() override;
	bool init();

private:
	void Run() override;
	void publish(hrt_abstime now, bool force);
	PX4Rangefinder _rangefinder{0};
	nra24::Parser _parser{0, 0.5f, 200.f, 200000};
	int32_t _radar_id{0}, _bitrate{500000}, _rotation{25}, _timeout_ms{200};
	float _minimum{0.5f}, _maximum{200.f};
	hrt_abstime _last_publish{0};
	uint32_t _frames{0}, _receive_errors{0}, _overflows{0};
	uint64_t _can_errors{0};
	bool _published_valid{false};
};

bool NRA24Can::init()
{
	uORB::SubscriptionData<actuator_armed_s> armed{ORB_ID(actuator_armed)};
	armed.update();

	if (armed.get().armed) { PX4_ERR("disarm before changing CAN ownership"); return false; }

	if (param_get(param_find("NRA24_ID"), &_radar_id) != PX4_OK
	    || param_get(param_find("NRA24_BITRATE"), &_bitrate) != PX4_OK
	    || param_get(param_find("NRA24_ROT"), &_rotation) != PX4_OK
	    || param_get(param_find("NRA24_TIMEOUT"), &_timeout_ms) != PX4_OK
	    || param_get(param_find("NRA24_MIN"), &_minimum) != PX4_OK
	    || param_get(param_find("NRA24_MAX"), &_maximum) != PX4_OK) {
		PX4_ERR("parameter lookup failed"); return false;
	}

	if (_radar_id < 0 || _radar_id > 15 || _rotation < 0 || _rotation > 25
	    || _timeout_ms < 50 || _timeout_ms > 2000 || !std::isfinite(_minimum) || !std::isfinite(_maximum)
	    || _minimum < 0.1f || _maximum <= _minimum || _maximum > 200.f) {
		PX4_ERR("invalid NRA24 parameters"); return false;
	}

	const int result = nra24_can_acquire(this, _bitrate);

	if (result < 0) {
		PX4_ERR("CAN1 acquisition failed: %d; enable SENS_NRA24_EN and reboot", result); return false;
	}

	_parser = nra24::Parser(uint8_t(_radar_id), _minimum, _maximum, uint64_t(_timeout_ms) * 1000);
	device::Device::DeviceId id{};
	id.devid_s.bus_type = device::Device::DeviceBusType_UAVCAN;
	id.devid_s.bus = 1;
	id.devid_s.address = _radar_id;
	id.devid_s.devtype = DRV_DIST_DEVTYPE_NRA24;
	_rangefinder.set_device_id(id.devid);
	_rangefinder.set_rangefinder_type(distance_sensor_s::MAV_DISTANCE_SENSOR_RADAR);
	_rangefinder.set_orientation(uint8_t(_rotation));
	_rangefinder.set_min_distance(_minimum);
	_rangefinder.set_max_distance(_maximum);
	_rangefinder.set_hfov(28.f * 3.14159265359f / 180.f);
	_rangefinder.set_vfov(18.f * 3.14159265359f / 180.f);
	publish(hrt_absolute_time(), true);
	ScheduleOnInterval(2000);
	return true;
}

void NRA24Can::publish(hrt_abstime now, bool force)
{
	const bool valid = _parser.valid();

	if (valid || force || _published_valid || now - _last_publish >= 100000) {
		// Reserved sensor fields are not interpreted as calibrated signal quality.
		_rangefinder.update(now, valid ? _parser.distance() : NAN, valid ? -1 : 0);
		_last_publish = now;
		_published_valid = valid;
	}
}

void NRA24Can::Run()
{
	if (should_exit()) {
		ScheduleClear();
		_parser.invalidate(nra24::State::Disconnected);
		publish(hrt_absolute_time(), true);
		nra24_can_release(this);
		exit_and_cleanup();
		return;
	}

	uint64_t errors = 0;
	uint32_t overflows = 0;

	if (nra24_can_counters(this, errors, overflows) == 0) {
		if (overflows != _overflows) {
			_parser.discard_cycle(nra24::State::Incomplete);
			publish(hrt_absolute_time(), true);
		}

		_can_errors = errors;
		_overflows = overflows;
	}

	for (unsigned i = 0; i < 64; ++i) {
		nra24_can_frame frame{};
		const int received = nra24_can_receive(this, frame);

		if (received < 0) {
			++_receive_errors;
			_parser.invalidate(nra24::State::Disconnected);
			publish(hrt_absolute_time(), true);
			break;
		}

		if (received == 0) { break; }

		++_frames;
		const hrt_abstime now = hrt_absolute_time();

		if (_parser.consume(frame.id, frame.data, frame.length, frame.flags, now)) { publish(now, false); }
	}

	const hrt_abstime now = hrt_absolute_time();

	if (_parser.tick(now)) { publish(now, true); }

	if (!_parser.valid()) { publish(now, false); }
}

int NRA24Can::print_status()
{
	PX4_INFO("CAN1, bitrate %" PRId32 ", radar ID %" PRId32 ", orientation %" PRId32, _bitrate, _radar_id, _rotation);
	PX4_INFO("state: %s, distance: %.2f m, uORB instance: %d", nra24::Parser::name(_parser.state()),
		 double(_parser.distance()), _rangefinder.get_instance());
	PX4_INFO("frames: %" PRIu32 ", heartbeats: %" PRIu32 ", target status: %" PRIu32,
		 _frames, _parser.heartbeats(), _parser.statuses());
	PX4_INFO("range frames: %" PRIu32 ", valid cycles: %" PRIu32 ", zero-target: %" PRIu32 ", rejected: %" PRIu32,
		 _parser.range_frames(), _parser.valid_cycles(), _parser.zero_targets(), _parser.rejected_ranges());
	PX4_INFO("CAN errors: %" PRIu64 ", overflows: %" PRIu32 ", receive errors: %" PRIu32,
		 _can_errors, _overflows, _receive_errors);
	return 0;
}

int NRA24Can::task_spawn(int argc, char *argv[])
{
	(void)argc;
	(void)argv;
	NRA24Can *instance = new NRA24Can();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) { return PX4_OK; }
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;
	return PX4_ERROR;
}

int NRA24Can::print_usage(const char *reason)
{
	if (reason) { PX4_WARN("%s", reason); }

	PRINT_MODULE_DESCRIPTION("Native classic CAN NRA24 rangefinder. Configure NRA24_* parameters before start.\n"
				 "SENS_NRA24_EN reserves CAN1 at boot. DroneCAN remains on CAN2. Reboot after changing bitrate.");
	PRINT_MODULE_USAGE_NAME("nra24_can", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int nra24_can_main(int argc, char *argv[])
{
	return NRA24Can::main(argc, argv);
}
