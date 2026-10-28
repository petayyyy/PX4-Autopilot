/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef DISTANCE_SENSOR_HPP
#define DISTANCE_SENSOR_HPP

#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/distance_sensor.h>
#include "RadarDistanceEncoding.hpp"

class MavlinkStreamDistanceSensor : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamDistanceSensor(mavlink); }

	static constexpr const char *get_name_static() { return "DISTANCE_SENSOR"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_DISTANCE_SENSOR; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _distance_sensor_subs.advertised_count() * (MAVLINK_MSG_ID_DISTANCE_SENSOR_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES);
	}

private:
	explicit MavlinkStreamDistanceSensor(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::SubscriptionMultiArray<distance_sensor_s> _distance_sensor_subs{ORB_ID::distance_sensor};

	bool send() override
	{
		bool updated = false;

		for (int i = 0; i < _distance_sensor_subs.size(); i++) {
			distance_sensor_s dist_sensor;

			if (_distance_sensor_subs[i].update(&dist_sensor)) {
				mavlink_distance_sensor_t msg{};

				msg.time_boot_ms = dist_sensor.timestamp / 1000; /* us to ms */

				switch (dist_sensor.type) {
				case MAV_DISTANCE_SENSOR_ULTRASOUND:
					msg.type = MAV_DISTANCE_SENSOR_ULTRASOUND;
					break;

				case MAV_DISTANCE_SENSOR_LASER:
					msg.type = MAV_DISTANCE_SENSOR_LASER;
					break;

				case MAV_DISTANCE_SENSOR_RADAR:
					msg.type = MAV_DISTANCE_SENSOR_RADAR;
					break;

				case MAV_DISTANCE_SENSOR_INFRARED:
					msg.type = MAV_DISTANCE_SENSOR_INFRARED;
					break;

				default:
					msg.type = MAV_DISTANCE_SENSOR_LASER;
					break;
				}

				if (dist_sensor.type == MAV_DISTANCE_SENSOR_RADAR) {
					// NaN is used in uORB for no target/timeouts. Never cast NaN to an integer.
					// 65535 cm is an out-of-range placeholder, accompanied by invalid quality.
					const auto encoded = encode_radar_distance(dist_sensor.current_distance, dist_sensor.signal_quality);
					msg.current_distance = encoded.centimetres;
					msg.signal_quality = encoded.signal_quality;
					msg.horizontal_fov = dist_sensor.h_fov;
					msg.vertical_fov = dist_sensor.v_fov;

				} else {
					msg.current_distance = dist_sensor.current_distance * 1e2f; // m to cm
				}

				msg.id               = i;
				msg.max_distance     = dist_sensor.max_distance * 1e2f;     // m to cm
				msg.min_distance     = dist_sensor.min_distance * 1e2f;     // m to cm
				msg.orientation      = dist_sensor.orientation;
				msg.covariance = dist_sensor.type == MAV_DISTANCE_SENSOR_RADAR ?
						 encode_radar_covariance(dist_sensor.variance) : uint8_t(dist_sensor.variance * 1e4f);

				mavlink_msg_distance_sensor_send_struct(_mavlink->get_channel(), &msg);

				updated = true;
			}
		}

		return updated;
	}
};

#endif // DISTANCE_SENSOR
