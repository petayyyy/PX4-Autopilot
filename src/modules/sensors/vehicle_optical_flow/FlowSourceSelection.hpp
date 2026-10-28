// SPDX-License-Identifier: BSD-3-Clause
#pragma once

namespace flow_selection
{
template<typename Source>
Source choose(Source primary, Source secondary, Source current, bool primary_fresh,
	      bool secondary_fresh, bool fallback_enabled, bool primary_ready)
{
	const bool secondary_available = fallback_enabled && secondary_fresh;

	if (primary_fresh) {
		if (current == secondary && secondary_available && !primary_ready) { return secondary; }

		return primary;
	}

	return secondary_available ? secondary : Source::NONE;
}
}
