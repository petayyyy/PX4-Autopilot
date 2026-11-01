# Range selection validation — 30 October 2026

SITL and disarmed Cube Orange+ bench tests verified source selection and sensor
input delivery. No real-aircraft flight was performed. See [setup and migration](RangeSensorRoles.md).

The tested implementation is `1bae8d4d6a`. The refactor comparison below uses
`366c6a16ec` as its baseline, which already includes the EKF input-timing fix.

## SITL and automated tests

- SITL and Cube Orange+ builds passed.
- Four CTest suites passed: role mapping, input timing, EKF ring buffer, and height fusion.
- Seven parameter-migration tests and four SITL configuration checks passed.
- Reversing device roles and publication order preserved the configured mapping.
  Duplicate publishers were covered by a unit test.

Three matched Gazebo scenarios used optical-flow fusion, with GPS/EV fusion disabled:

| Scenario | Source transitions before / after refactor | Height resets |
|---|---:|---:|
| Continuous short source at 5 Hz | 0 / 0 | 0 in both |
| Twenty 350 ms short-source pauses; long source at 1 Hz | 60 / 60 | 0 in both |
| Threshold crossings | 4 / 4 | 0 in both |

Transitions include NONE. Selection sequences matched in every comparison.
Some broader height/tracking checks flagged both builds; these results do not
constitute a general flight-performance pass. Simulation bypassed physical drivers.

## Hardware setup

| Input | Device ID | Connection |
|---|---:|---|
| Short range: H-Flow | 9010435 | DroneCAN CAN2, node 125 |
| Long range: NRA24 | 15007755 | Native CAN1, 500 kbit/s |
| Optical flow: H-Flow | 8682755 | Same CAN2 node |

Bench settings: switch to long above **1 m**, return below or at **0.8 m**,
**300 ms** freshness limits for both sources, long-range ceiling **200 m**.
Device IDs describe the tested installation; identify the devices on each controller
before assigning roles. The firmware upload passed CRC verification.

## Bench results

- **Reception and switching:** both sensors published measurements. Repeated target
  sweeps produced matching source sequences across all three EKFs.
- **EKF delivery:** both range inputs reached all three EKFs. The oldest recorded
  sample at forwarding was **21.3 ms** old relative to the EKF input reference time.
- **Optical flow:** with fusion enabled, fused updates were recorded in all three
  EKFs, with no reported flow-innovation rejections in the capture. Range-height
  fusion remained inactive; disarmed ground handling limits conclusions about flight fusion.
- **No target and recovery:** NRA24 reported no target at rest and briefly during
  movement. One dropout, with H-Flow above the 1 m cutoff, caused a **370 ms** range
  aid-source update gap. Fresh radar data restored updates. Neither source was
  eligible during the gap; the one-second hold was not blocking recovery.
- **Flow distance:** a 3 m short limit admitted a ~2.01 m target reading into flow
  preprocessing; a 1 m limit excluded it as expected.
- **Driver restart:** stopping NRA24 for 15 seconds and restarting it left H-Flow
  operating on CAN2. CAN1 queue overflows during that deliberate stop were expected.
- **Logging:** both detailed ULogs had zero logger dropouts. Sampled logs and
  throttled console messages cannot prove every transient state or sample delivery.
- **Configuration persistence:** assigned role IDs and selection thresholds survived reboot.

## Status-command effect

Printing `uavcan status` over the MAVLink console holds the DroneCAN node mutex.
In a controlled check, one print added **30 CAN aggregate errors and 5 transfer
errors**; counters stayed unchanged during the quiet 15-second intervals before
and after it. Redirecting the report to a file and reading it afterward avoided
that increase. Use ULog for timing measurements, rather than repeated console status prints.

## Remaining validation

Real-flight navigation, physical handover at the default 20/18 m thresholds,
vibration tolerance, and reliable NRA24 returns in stationary high-altitude hover
remain unverified. The reduced 1/0.8 m thresholds were used for bench testing.
