#pragma once

#include <cstddef>
#include <vector>

#include "MotionEvent.h"

class MotionEventTrack {
public:
	MotionEventTrack() = default;
	~MotionEventTrack() = default;

	MotionEventTrack(const MotionEventTrack &) = delete;
	MotionEventTrack &operator=(const MotionEventTrack &) = delete;

	size_t GetNumEvents() const { return m_events.size(); }
	const MotionEvent &GetEvent(size_t eventIndex) const {
		static const MotionEvent s_default;
		if (eventIndex >= m_events.size()) {
			return s_default;
		}
		return m_events[eventIndex];
	}
	void AddEvent(const MotionEvent &event) { m_events.push_back(event); }
	void Clear() { m_events.clear(); }

private:
	std::vector<MotionEvent> m_events;
};
