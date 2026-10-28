// SPDX-License-Identifier: BSD-3-Clause
#include "../NRA24Parser.hpp"
#include "RadarDistanceEncoding.hpp"
#include <cassert>
#include <fstream>
#include <iostream>
#include <regex>
#include <string>

using nra24::Parser;
using nra24::State;

static void heartbeat(Parser &p, uint64_t t, uint8_t id = 0)
{
	const uint8_t d[8] {};
	p.consume(0x60A + 16 * id, d, 8, 0, t);
}
static bool status(Parser &p, uint8_t count, uint64_t t, uint8_t id = 0)
{
	const uint8_t d[8] {count, 0, 0, 0, 0, 0, 0, 0};
	return p.consume(0x70B + 16 * id, d, 8, 0, t);
}
static bool range(Parser &p, uint8_t index, uint16_t cm, uint64_t t, uint8_t id = 0)
{
	const uint8_t d[8] {index, 0, uint8_t(cm >> 8), uint8_t(cm), 0, 0, 0, 0};
	return p.consume(0x70C + 16 * id, d, 8, 0, t);
}

static void transitions()
{
	Parser p(0, .5f, 200.f, 200000);
	p.tick(100);
	assert(!p.valid() && p.state() == State::Disconnected);
	heartbeat(p, 1000);
	p.tick(1000);
	assert(p.connected(1000) && p.state() == State::Waiting);
	status(p, 1, 1100);
	assert(range(p, 1, 260, 1200));
	assert(p.valid() && std::fabs(p.distance() - 2.6f) < 1e-5f);
	status(p, 0, 1300);
	assert(p.state() == State::NoTarget && !p.valid() && std::isnan(p.distance()));
	assert(!range(p, 1, 260, 1400)); // a delayed range cannot resurrect zero-target status
	assert(!p.valid());
	status(p, 1, 2000);
	range(p, 1, 260, 2100);
	heartbeat(p, 202000);
	p.tick(202100);
	assert(p.state() == State::Waiting && !p.valid() && std::isnan(p.distance()));
	p.tick(402000);
	assert(p.state() == State::Disconnected);
}

static void filtering_and_cycles()
{
	Parser p(15, .5f, 200.f, 200000);
	const uint8_t sample[8] {1, 0xC8, 0x07, 0xD0, 0x32, 0x02, 0xEE, 0x96};
	heartbeat(p, 1000, 15);
	status(p, 1, 1100, 15);
	assert(!p.consume(0x70C, sample, 8, 0, 1150)); // wrong radar
	assert(!p.consume(0x7FC, sample, 8, 1, 1151)); // extended
	assert(!p.consume(0x7FC, sample, 8, 2, 1152)); // RTR
	assert(!p.consume(0x7FC, sample, 8, 4, 1153)); // error
	assert(!p.consume(0x7FC, sample, 7, 0, 1154)); // short
	assert(p.consume(0x7FC, sample, 8, 0, 1200));
	assert(p.valid() && p.distance() == 20.f); // manual's 20 m sample, ID 15

	status(p, 2, 2000, 15);
	assert(!range(p, 1, 300, 2100, 15));
	assert(!range(p, 1, 300, 2200, 15)); // duplicate target does not complete a cycle
	assert(range(p, 2, 125, 2300, 15));
	assert(p.valid() && std::fabs(p.distance() - 1.25f) < 1e-5f); // nearest valid target
	status(p, 1, 3000, 15);
	range(p, 1, 49, 3100, 15);
	assert(!p.valid() && p.state() == State::OutOfRange);
	status(p, 1, 4000, 15);
	range(p, 1, 20001, 4100, 15);
	assert(!p.valid());
	status(p, 1, 5000, 15);
	range(p, 1, 0, 5100, 15);
	assert(!p.valid());
	status(p, 2, 6000, 15);
	range(p, 1, 260, 6100, 15);
	status(p, 1, 6200, 15); // incomplete old cycle is invalidated
	assert(p.state() == State::Incomplete);
	range(p, 1, 270, 6300, 15);
	assert(p.valid());
	status(p, 2, 7000, 15);
	range(p, 1, 260, 7100, 15);
	heartbeat(p, 207000, 15);
	p.tick(207000);
	assert(!p.valid() && p.state() == State::Incomplete);
	assert(!range(p, 2, 250, 207001, 15));
	assert(!p.valid());
	status(p, 255, 208000, 15);
	assert(!p.valid());
	status(p, 1, 209000, 15);
	p.discard_cycle(State::Incomplete); // hardware queue loss cancels partial cycle
	assert(!range(p, 1, 260, 209001, 15));
}

static void mavlink_encoding()
{
	auto value = encode_radar_distance(2.60f, -1);
	assert(value.centimetres == 260 && value.signal_quality == 0);
	assert(encode_radar_distance(1.04f, -1).centimetres == 104);

	for (float bad : {NAN, INFINITY, -INFINITY, -1.f, 100000.f}) {
		value = encode_radar_distance(bad, -1);
		assert(value.centimetres == UINT16_MAX && value.signal_quality == 1);
	}
	value = encode_radar_distance(2.60f, 0);
	assert(value.centimetres == UINT16_MAX && value.signal_quality == 1);
	assert(encode_radar_distance(2.60f, 1).signal_quality == 2);
	assert(encode_radar_distance(2.60f, 100).signal_quality == 100);
	assert(encode_radar_covariance(0.f) == UINT8_MAX);
	assert(encode_radar_covariance(NAN) == UINT8_MAX);
	assert(encode_radar_covariance(.0025f) == 25);
	assert(encode_radar_covariance(100.f) == 254);
}

static void replay(const char *filename, unsigned expected_valid, unsigned expected_status, float minimum = .1f)
{
	std::ifstream input(filename);
	assert(input.good());
	Parser p(0, minimum, 200.f, 200000);
	const std::regex frame(R"(\(([0-9]+\.[0-9]+)\)\s+can0\s+([0-9A-Fa-f]{3})#([0-9A-Fa-f]{16}))");
	std::string line;
	uint64_t last = 0;
	unsigned valid = 0;

	while (std::getline(input, line)) {
		std::smatch m;

		if (!std::regex_search(line, m, frame)) { continue; }

		const uint64_t t = uint64_t(std::llround(std::stod(m[1]) * 1000000));
		assert(t >= last);
		last = t;
		const uint32_t id = std::stoul(m[2], nullptr, 16);
		const std::string hex = m[3];
		uint8_t data[8];

		for (unsigned i = 0; i < 8; ++i) { data[i] = uint8_t(std::stoul(hex.substr(2 * i, 2), nullptr, 16)); }

		if (p.consume(id, data, 8, 0, t) && p.valid()) { ++valid; }

		p.tick(t);
	}

	assert(valid == expected_valid);
	assert(p.statuses() == expected_status);
	p.tick(last + 200000);
	assert(!p.valid() && std::isnan(p.distance()));
	std::cout << "Replay passed: " << filename << ", min=" << minimum << " m, valid cycles=" << valid << ", statuses=" <<
		  p.statuses() << "\n";
}

int main(int argc, char **argv)
{
	transitions();
	filtering_and_cycles();
	mavlink_encoding();
	std::cout << "Passed: no-target, heartbeat/range timeouts, recovery, IDs 0/15, manual example,\n"
		  << "extended/RTR/error/short frame rejection, limits, nearest target, duplicates,\n"
		  << "incomplete cycles and queue-loss invalidation.\n";
	std::cout << "Passed: MAVLink centimetre rounding, invalid/unknown quality, NaN/Inf/negative\n"
		  << "distance encoding, and unknown/saturated covariance.\n";

	if (argc == 3) {
		replay(argv[1], 698, 1194); // first range of 699 precedes the capture's first status
		replay(argv[1], 687, 1194, .5f); // default physical minimum additionally rejects 11 close returns
		replay(argv[2], 0, 387);
	}
}
