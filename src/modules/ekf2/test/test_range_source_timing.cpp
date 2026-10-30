// SPDX-License-Identifier: BSD-3-Clause
#include <gtest/gtest.h>
#include "EKF/estimator_interface.h"
#include "RangeSourceSelection.hpp"

class RangeTimingInterface : public EstimatorInterface
{
public:
	bool init(uint64_t timestamp) override
	{
		_initialised = initialise_interface(timestamp);
		return _initialised;
	}

	void setTiming(uint64_t horizon, float delay_ms = 0.f, float dt = .008f)
	{
		_time_delayed_us = horizon;
		_params.range_delay_ms = delay_ms;
		_dt_ekf_avg = dt;
	}

	bool popRange(rangeSample &sample)
	{
		return _range_buffer && _range_buffer->pop_first_older_than(_time_delayed_us, &sample);
	}
};

class RangeSourceTimingTest : public ::testing::Test
{
protected:
	RangeTimingInterface ekf;
	range_selection::Selector selector;

	void SetUp() override
	{
		ASSERT_TRUE(ekf.init(1000000));
		ekf.setTiming(848000);
	}

	range_selection::Candidate candidate(int index, uint64_t now, uint64_t timestamp,
					     uint64_t timeout = 300000, int quality = 100)
	{
		if (range_selection::valid(now, timestamp, timeout, 2.f, .1f, 200.f, quality)
		    && ekf.isRangeSampleRecent(timestamp)) {
			return {index, timestamp, 2.f};
		}

		return {};
	}
};

TEST_F(RangeSourceTimingTest, StrictBufferBoundary)
{
	// 152 ms horizon lag and 4 ms timestamp adjustment: effective age < 248 ms.
	EXPECT_TRUE(ekf.isRangeSampleRecent(752001));
	EXPECT_FALSE(ekf.isRangeSampleRecent(752000));
	EXPECT_FALSE(ekf.isRangeSampleRecent(751999));

	// Check against the actual buffer consumer, not just another age calculation.
	rangeSample popped{};
	ekf.setRangeData(rangeSample{752000, 2.f, 100});
	EXPECT_FALSE(ekf.popRange(popped));
	ekf.setRangeData(rangeSample{752001, 2.f, 100});
	ASSERT_TRUE(ekf.popRange(popped));
	EXPECT_EQ(popped.time_us, 748001);
}

TEST_F(RangeSourceTimingTest, FutureSampleWaitsForHorizon)
{
	EXPECT_TRUE(ekf.isRangeSampleRecent(1000000));
	ekf.setRangeData(rangeSample{1000000, 2.f, 100});
	rangeSample popped{};
	EXPECT_FALSE(ekf.popRange(popped));
	ekf.setTiming(996000);
	ASSERT_TRUE(ekf.popRange(popped));
	EXPECT_EQ(popped.time_us, 996000);
}

TEST_F(RangeSourceTimingTest, TracksDelayUpdateIntervalAndHorizon)
{
	// Same observation becomes too old after increasing EKF2_RNG_DELAY.
	EXPECT_TRUE(ekf.isRangeSampleRecent(770000));
	ekf.setTiming(848000, 20.f);
	EXPECT_FALSE(ekf.isRangeSampleRecent(770000));
	EXPECT_FALSE(ekf.isRangeSampleRecent(772000));
	EXPECT_TRUE(ekf.isRangeSampleRecent(772001));

	// A changed update interval affects the half-step adjustment too.
	ekf.setTiming(848000, 20.f, .010f);
	EXPECT_FALSE(ekf.isRangeSampleRecent(773000));
	EXPECT_TRUE(ekf.isRangeSampleRecent(773001));
	ekf.setTiming(850000, 20.f, .010f);
	EXPECT_FALSE(ekf.isRangeSampleRecent(773001));
}

TEST_F(RangeSourceTimingTest, StartupDoesNotWrapNegativeAdjustedTime)
{
	ekf.setTiming(0);
	EXPECT_FALSE(ekf.isRangeSampleRecent(0));
	EXPECT_FALSE(ekf.isRangeSampleRecent(3999));
	EXPECT_FALSE(ekf.isRangeSampleRecent(4000));
	EXPECT_TRUE(ekf.isRangeSampleRecent(4001));
}

TEST_F(RangeSourceTimingTest, BothFreshnessLimitsAndQualityApply)
{
	EXPECT_EQ(candidate(0, 1000000, 760000).index, 0);
	EXPECT_EQ(candidate(0, 1000000, 760000, 200000).index, -1);
	EXPECT_EQ(candidate(0, 1000000, 740000, 300000).index, -1);
	EXPECT_EQ(candidate(0, 1000000, 740000, 2000000).index, -1);
	EXPECT_EQ(candidate(0, 1000000, 990000, 300000, 0).index, -1);
}

TEST_F(RangeSourceTimingTest, RejectsRecordedCachedLongHandover)
{
	// The recorded long sample was only 264 ms old at handover, passing its
	// 2 s source timeout, but already outside the delayed EKF's input window.
	ekf.setTiming(60112000);
	const auto short_sensor = candidate(0, 60268000, 59964000);
	const auto long_sensor = candidate(1, 60268000, 60004000, 2000000);
	ASSERT_EQ(short_sensor.index, -1);
	ASSERT_EQ(long_sensor.index, -1);
	EXPECT_EQ(selector.choose(60268000, short_sensor, long_sensor, 20.f, 2.f).index, -1);

	// Recovery is immediate: an unusable cached fallback cannot hold selection.
	ekf.setTiming(60212000);
	EXPECT_EQ(selector.choose(60364000, candidate(0, 60364000, 60344000),
				  candidate(1, 60364000, 60004000, 2000000), 20.f, 2.f).index, 0);
}

TEST_F(RangeSourceTimingTest, TimingExpiryBreaksHoldButHealthySourceKeepsIt)
{
	ASSERT_EQ(selector.choose(1000000, {}, candidate(1, 1000000, 1000000), 20.f, 2.f).index, 1);

	ekf.setTiming(1048000);
	// Recovered short sensor cannot break the hold while long remains usable.
	EXPECT_EQ(selector.choose(1200000, candidate(0, 1200000, 1200000),
				  candidate(1, 1200000, 1000000), 20.f, 2.f).index, 1);

	ekf.setTiming(1096000);
	// Long is still inside 300 ms, but has reached the EKF timing boundary.
	EXPECT_EQ(selector.choose(1248000, candidate(0, 1248000, 1248000),
				  candidate(1, 1248000, 1000000), 20.f, 2.f).index, 0);
}

TEST_F(RangeSourceTimingTest, ContinuousFiveHzShortDoesNotChatter)
{
	// The new check must not recreate the old 100 ms wall-clock timeout bug.
	for (uint64_t now = 1000000; now < 4000000; now += 4000) {
		ekf.setTiming(now - 152000);
		const uint64_t short_timestamp = now - (now - 1000000) % 200000;
		const uint64_t long_timestamp = now - (now - 1000000) % 40000;
		EXPECT_EQ(selector.choose(now, candidate(0, now, short_timestamp),
					  candidate(1, now, long_timestamp), 20.f, 2.f).index, 0);
	}
}
