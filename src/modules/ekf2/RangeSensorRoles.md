# Range sensor roles and configuration migration

Range selection uses two explicitly assigned `distance_sensor.device_id` values.
Drivers decode their devices and publish measurements; the selection policy only
knows the short-range and long-range roles. Changing the sensor model or its
transport does not require changing the selector, provided a suitable driver is
included and enabled.

The CAN arrangement is unchanged: enabling `SENS_NRA24_EN` reserves CAN1 for the
native NRA24 driver and leaves CAN2 for DroneCAN. Disabling it restores the normal
DroneCAN path. Role parameters do not configure physical CAN ports or start drivers.

## Assign the devices

Read every advertised distance-sensor instance while the devices are publishing:

```text
listener distance_sensor -i 0 -n 1
listener distance_sensor -i 1 -n 1
```

Identify each physical sensor using its measurements and driver status. Record its
`device_id`, not the uORB instance number after `distance_sensor:`. Instance numbers
can change with startup order. Device IDs normally remain stable for the same
driver, bus and device address; moving ports or changing addresses can change them.

Set `EKF2_RNG_S_ID` to the short-range device ID and `EKF2_RNG_L_ID` to the long-range
device ID in QGC or with `param set` in the PX4 shell. Save the parameters and reboot.
Both EKF2 and optical-flow preprocessing snapshot these assignments when their
modules start. Parameter updates during operation do not change the active IDs;
restart the controller rather than restarting only one module.

An ID of zero disables that role. Both nonzero IDs must differ. A full-width
unsigned ID above 2147483647 is represented as `ID - 4294967296` in the INT32
parameter; the migration tool performs this conversion.

## Selection settings

| Parameter | Meaning | Default |
|---|---|---|
| `EKF2_RNG_S_ID` | Short-range device ID; zero disables the role | 0 |
| `EKF2_RNG_L_ID` | Long-range device ID; zero disables the role | 0 |
| `EKF2_RNG_S_MAX` | Short-range selection ceiling | 20 m |
| `EKF2_RNG_HYST` | Return hysteresis below that ceiling | 2 m |
| `EKF2_RNG_L_MAX` | Long-range selection ceiling | 200 m |
| `EKF2_RNG_S_TO` | Short-range sample freshness limit | 1 s |
| `EKF2_RNG_L_TO` | Long-range sample freshness limit | 0.3 s |

The distance and freshness settings can update during operation. Device assignments
require reboot. Migrate existing distance and freshness settings explicitly rather
than assuming the defaults match the installation.

All existing selection rules still apply: downward orientation, valid measurement
and limits, nonzero signal quality, configured freshness, and the adjusted sample
timestamp fitting the EKF input window. An unknown quality of -1 remains acceptable.
The short role is preferred below its ceiling; return from long requires short to
be at or below `EKF2_RNG_S_MAX - EKF2_RNG_HYST`. A healthy current source has a
one-second hold from the last selection change. An unusable current source can be
left immediately. No cached sample is given a new timestamp to make it look fresh.

The configured timeout does not override the driver's own invalidation timeout or
the EKF input window. Selecting a sample does not guarantee innovation acceptance
or fusion.

## Optical flow association

When any range role is configured, optical-flow preprocessing uses only the
configured short-range device for its accompanying distance. It applies the
short-range ceiling, freshness and sample-validity checks. The existing flow
buffer additionally requires a distance within 100 ms of the flow timestamp.
The long-range device is not substituted into this preprocessing path.

Optical-flow angular measurements continue without an accompanying distance. This
does not prevent EKF from using its own terrain/range estimate, including the
selected long-range sensor, for optical-flow fusion.

With explicit roles, distance embedded in optical-flow messages cannot bypass a
missing or invalid assigned device. With both IDs zero, the existing embedded
flow-distance path remains available for flow-only setups; external range devices
are not automatically assigned. This distinction also applies to EKF2's embedded
flow-distance fallback after one second without external range updates.

## Migrate an existing configuration

Export the existing controller parameters to a QGC `.params` text file before
updating. Record the two published device IDs. The old parameters remain in the
firmware as **unused legacy settings**, so previously saved values are retained,
but changing them no longer changes selection.

| Old setting | New setting |
|---|---|
| `EKF2_RNG_CAN_MAX` | `EKF2_RNG_S_MAX` |
| `EKF2_RNG_CAN_HYS` | `EKF2_RNG_HYST` |
| `EKF2_RNG_MAV_MAX` | `EKF2_RNG_L_MAX` |
| `EKF2_RNG_CAN_TO` | `EKF2_RNG_S_TO` |
| `EKF2_RNG_RAD_TO` **or** `EKF2_RNG_MAV_TO` | `EKF2_RNG_L_TO` |

Choose the old timeout that actually governed the previous long-range sensor:
`radar` for the native NRA24 path, or `mavlink` for the old MAVLink input path.
There is no reliable automatic choice when both old values are present.

Run [the migration tool](../../../Tools/migrate_range_roles.py) from the repository
root, replacing the example IDs with the IDs read from your controller:

```text
python Tools/migrate_range_roles.py old.params roles.params --short-id 123 --long-id 456 --long-timeout-from radar
```

The tool keeps the original settings and comments, appends the seven new settings,
and notes any omitted old value for which it used the old firmware default. It
rejects malformed input, conflicting new settings, duplicate IDs and multiple
system/component exports. It refuses to overwrite an existing output file and
never connects to hardware.

Review the generated values, import them into the updated firmware while disarmed,
save, and reboot. A direct import of only the old settings does not configure the
new roles. Confirm active assignments and reception before proceeding with tests.

## Diagnostics and missing devices

`ekf2 status` reports the startup IDs, selected uORB instance and unavailable mask.
`sensors status` includes the optical-flow module's active range association.
The unavailable mask combines these bits:

| Bit | Meaning |
|---|---|
| 1 | Assigned short role has no valid, timely candidate |
| 2 | Assigned long role has no valid, timely candidate |
| 4 | Both roles have the same nonzero ID; neither is accepted |
| 8 | Multiple usable publishers claim an assigned ID; that role is ambiguous |

Missing assignments are reported at module startup. Unavailable configured roles
produce a throttled warning. An absent, invalid or ambiguous role never recruits an
unassigned sensor. The other usable assigned role may still provide fallback. If
neither is usable, selection is NONE; driver and estimator health checks continue
to operate normally.

## Code and checks

See [recorded SITL and hardware bench validation](RangeSensorValidation.md) for
test conditions, results, and validation limits.

- [Shared role matching and validity](../../lib/sensor_selection/RangeSensorRoles.hpp)
- [Selection state machine](RangeSourceSelection.hpp)
- [EKF2 candidate collection](EKF2.cpp)
- [Optical-flow preprocessing](../sensors/vehicle_optical_flow/VehicleOpticalFlow.cpp)
- [Role tests](test/test_range_sensor_roles.cpp) and [timing tests](test/test_range_source_timing.cpp)

```text
python Tools/test_migrate_range_roles.py
make px4_sitl_test unit-test_range_sensor_roles unit-test_range_source_timing unit-test_EKF_ringbuffer unit-test_EKF_height_fusion
ctest --test-dir build/px4_sitl_test -R "unit-test_(range_sensor_roles|range_source_timing|EKF_ringbuffer|EKF_height_fusion)$" --output-on-failure
make px4_sitl_default
make cubepilot_cubeorangeplus_default
```
