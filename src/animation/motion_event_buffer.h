#pragma once

#include <cstddef>
#include <memory>
#include <vector>

#include "runtime/event_data.h"

class MotionEventBuffer {
public:
	struct FiredEvent {
		float time = 0.0f;
		std::shared_ptr<godot::MotionMatching::EventData> data;
	};

	MotionEventBuffer() = default;
	~MotionEventBuffer() = default;

	MotionEventBuffer(const MotionEventBuffer &) = delete;
	MotionEventBuffer &operator=(const MotionEventBuffer &) = delete;

	void Clear() { m_fired.clear(); }
	bool IsEmpty() const { return m_fired.empty(); }
	size_t GetNumFired() const { return m_fired.size(); }
	const FiredEvent &GetFired(size_t index) const { return m_fired[index]; }

	void AddFired(float time, const std::shared_ptr<godot::MotionMatching::EventData> &data) {
		if (data) {
			m_fired.push_back(FiredEvent{ time, data });
		}
	}

private:
	std::vector<FiredEvent> m_fired;
};