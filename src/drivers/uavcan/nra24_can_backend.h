// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 PX4 Development Team.
#pragma once
#include <stdint.h>
#include <px4_platform_common/defines.h>

struct nra24_can_frame {
	uint32_t id;
	uint8_t data[8];
	uint8_t length;
	uint8_t flags; // 1 extended, 2 RTR, 4 error
};

__EXPORT int nra24_can_acquire(const void *owner, uint32_t bitrate);
__EXPORT int nra24_can_receive(const void *owner, nra24_can_frame &frame);
__EXPORT void nra24_can_release(const void *owner);
__EXPORT int nra24_can_counters(const void *owner, uint64_t &errors, uint32_t &overflows);
