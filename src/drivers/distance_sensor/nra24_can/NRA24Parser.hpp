// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 PX4 Development Team.
#pragma once

#include <cmath>
#include <cstdint>
#include <cstring>

namespace nra24
{
enum class State { Disconnected, NoTarget, Waiting, OutOfRange, Incomplete, Valid };

class Parser
{
public:
	Parser(uint8_t radar_id, float min_m, float max_m, uint64_t timeout_us) :
		_radar_id(radar_id), _min(min_m), _max(max_m), _timeout(timeout_us) {}

	// flags != 0 means extended, RTR, or error frame: none belong to this protocol.
	bool consume(uint32_t id, const uint8_t *data, uint8_t length, uint8_t flags, uint64_t now)
	{
		if (flags || length != 8 || !data) { return false; }

		const uint32_t offset = uint32_t(_radar_id) * 0x10U;

		if (id == 0x60AU + offset) {
			_heartbeat = now;
			++_heartbeats;
			return false;
		}

		if (id == 0x70BU + offset) {
			++_statuses;
			const bool incomplete = _active;
			_active = false;
			_status_time = now;
			_targets = data[0];
			_received = 0;
			_best = INFINITY;
			std::memset(_seen, 0, sizeof(_seen));

			if (_targets == 0) {
				++_zero_targets;
				invalidate(State::NoTarget);
				return true;
			}

			// The manual defines target indices 0..127.
			if (_targets > 128) { invalidate(State::Incomplete); return true; }

			_active = true;

			if (incomplete) { invalidate(State::Incomplete); return true; }

			return false;
		}

		if (id != 0x70CU + offset) { return false; }

		++_range_frames;

		if (!_active || now - _status_time >= _timeout || data[0] >= 128) {
			++_rejected_ranges;
			return false;
		}

		const uint8_t index = data[0];
		const uint8_t mask = uint8_t(1U << (index % 8));

		if (_seen[index / 8] & mask) { ++_rejected_ranges; return false; }

		_seen[index / 8] |= mask;
		++_received;
		const float range_m = float((uint16_t(data[2]) << 8) | data[3]) * 0.01f;

		if (range_m >= _min && range_m <= _max && range_m < _best) { _best = range_m; }

		if (_received < _targets) { return false; }

		_active = false;

		if (connected(now) && std::isfinite(_best)) {
			_distance = _best;
			_last_range = now;
			_state = State::Valid;
			++_valid_cycles;

		} else {
			invalidate(connected(now) ? State::OutOfRange : State::Disconnected);
		}

		return true;
	}

	bool tick(uint64_t now)
	{
		if (_active && now - _status_time >= _timeout) {
			_active = false;
			invalidate(State::Incomplete);
			return true;
		}

		const State previous = _state;

		if (!connected(now)) {
			_active = false;
			invalidate(State::Disconnected);

		} else if (_state == State::Disconnected) {
			invalidate(State::Waiting);

		} else if (_state == State::Valid && now - _last_range >= _timeout) {
			invalidate(State::Waiting);
		}

		return previous != _state;
	}

	bool valid() const { return _state == State::Valid; }
	bool connected(uint64_t now) const { return _heartbeat != 0 && now - _heartbeat < _timeout; }
	float distance() const { return _distance; }
	State state() const { return _state; }
	uint64_t heartbeat() const { return _heartbeat; }
	uint64_t last_range() const { return _last_range; }
	uint32_t heartbeats() const { return _heartbeats; }
	uint32_t statuses() const { return _statuses; }
	uint32_t zero_targets() const { return _zero_targets; }
	uint32_t range_frames() const { return _range_frames; }
	uint32_t valid_cycles() const { return _valid_cycles; }
	uint32_t rejected_ranges() const { return _rejected_ranges; }
	void invalidate(State state) { _state = state; _distance = NAN; }
	void discard_cycle(State state) { _active = false; invalidate(state); }

	static const char *name(State state)
	{
		switch (state) {
		case State::Disconnected: return "disconnected";

		case State::NoTarget: return "connected, no target";

		case State::Waiting: return "connected, range timeout";

		case State::OutOfRange: return "connected, out of range";

		case State::Incomplete: return "incomplete target cycle";

		case State::Valid: return "valid range";
		}

		return "unknown";
	}

private:
	uint8_t _radar_id;
	float _min;
	float _max;
	uint64_t _timeout;
	State _state{State::Disconnected};
	uint64_t _heartbeat{0}, _status_time{0}, _last_range{0};
	uint16_t _targets{0}, _received{0};
	uint8_t _seen[16] {};
	bool _active{false};
	float _best{INFINITY}, _distance{NAN};
	uint32_t _heartbeats{0}, _statuses{0}, _zero_targets{0}, _range_frames{0};
	uint32_t _valid_cycles{0}, _rejected_ranges{0};
};
} // namespace nra24
