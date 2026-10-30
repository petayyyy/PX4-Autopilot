// SPDX-License-Identifier: BSD-3-Clause
#include <gtest/gtest.h>
#include <lib/sensor_selection/RangeSensorRoles.hpp>
#include "RangeSourceSelection.hpp"
#include <array>

using range_sensor::Roles;
using range_sensor::Role;
using range_sensor::UniqueInstance;

TEST(RangeSensorRoles, IdentityNotTransportOrModel)
{
	// Deliberately reverse the former policy: MAVLink short, ordinary CAN long.
	const Roles roles{9699598, 9010179};
	EXPECT_EQ(roles.classify(9699598), Role::SHORT);
	EXPECT_EQ(roles.classify(9010179), Role::LONG);
	EXPECT_EQ(roles.classify(15007747), Role::NONE); // unassigned native radar
	EXPECT_EQ(roles.classify(0), Role::NONE);
	const Roles arbitrary{0xFFFFFFFF, 12345};
	EXPECT_EQ(arbitrary.classify(0xFFFFFFFF), Role::SHORT);
	EXPECT_EQ(arbitrary.classify(12345), Role::LONG);
}

TEST(RangeSensorRoles, MissingAndConflictingAssignments)
{
	const Roles none{};
	EXPECT_TRUE(none.valid());
	EXPECT_FALSE(none.configured());
	EXPECT_TRUE(none.allowEmbeddedFlowRange());
	EXPECT_EQ(none.classify(9010179), Role::NONE);
	const Roles conflict{123, 123};
	EXPECT_FALSE(conflict.valid());
	EXPECT_EQ(conflict.classify(123), Role::NONE);
	EXPECT_FALSE(conflict.allowEmbeddedFlowRange());
}

TEST(RangeSensorRoles, SingleRoleDoesNotAssignUnrelatedDevices)
{
	const Roles short_only{123, 0}, long_only{0, 456};
	EXPECT_EQ(short_only.classify(123), Role::SHORT);
	EXPECT_EQ(short_only.classify(456), Role::NONE);
	EXPECT_EQ(long_only.classify(456), Role::LONG);
	EXPECT_EQ(long_only.classify(123), Role::NONE);
	EXPECT_FALSE(short_only.allowEmbeddedFlowRange());
	EXPECT_FALSE(long_only.allowEmbeddedFlowRange());
}

TEST(RangeSensorRoles, PublisherOrderDoesNotAssignRoles)
{
	const Roles roles{123, 456};

	for (const auto &ids : {std::array<uint32_t, 3> {123, 456, 789}, {789, 456, 123}, {456, 123, 789}}) {
		UniqueInstance short_match, long_match;

		for (unsigned i = 0; i < ids.size(); ++i) {
			if (roles.classify(ids[i]) == Role::SHORT) { short_match.add(i); }

			if (roles.classify(ids[i]) == Role::LONG) { long_match.add(i); }
		}

		ASSERT_GE(short_match.index(), 0);
		ASSERT_GE(long_match.index(), 0);
		EXPECT_EQ(ids[short_match.index()], 123u);
		EXPECT_EQ(ids[long_match.index()], 456u);
	}
}

TEST(RangeSensorRoles, DuplicatePublishersFailClosed)
{
	UniqueInstance match;
	EXPECT_EQ(match.index(), -1);
	match.add(2);
	EXPECT_EQ(match.index(), 2);
	match.add(0);
	EXPECT_TRUE(match.ambiguous());
	EXPECT_EQ(match.index(), -1);
	match.add(1);
	EXPECT_EQ(match.index(), -1);
}

TEST(RangeSensorRoles, RoleMappingPreservesSwitchingAndHysteresis)
{
	// Opaque IDs do not enter the FSM. Reverse publication order and roles.
	const Roles roles{9699598, 9010179};
	ASSERT_EQ(roles.classify(9699598), Role::SHORT);
	ASSERT_EQ(roles.classify(9010179), Role::LONG);
	range_selection::Selector selector;
	range_selection::Candidate short_sensor{1, 1000000, 19.f}, long_sensor{0, 1000000, 19.f};
	EXPECT_EQ(selector.choose(1000000, short_sensor, long_sensor, 20.f, 2.f).index, 1);
	short_sensor.distance = 21.f;
	EXPECT_EQ(selector.choose(1100000, short_sensor, long_sensor, 20.f, 2.f).index, 0);
	short_sensor.distance = 17.f;
	EXPECT_EQ(selector.choose(1200000, short_sensor, long_sensor, 20.f, 2.f).index, 0);
	EXPECT_EQ(selector.choose(2100000, short_sensor, long_sensor, 20.f, 2.f).index, 1);
	EXPECT_EQ(selector.choose(2200000, {}, long_sensor, 20.f, 2.f).index, 0);
	EXPECT_EQ(selector.choose(2300000, short_sensor, {}, 20.f, 2.f).index, 1);
}
