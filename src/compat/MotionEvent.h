#pragma once

#include <cstddef>
#include <memory>
#include <vector>

#include "../runtime/EventData.h"

class MotionEvent {
public:
	MotionEvent() = default;
	~MotionEvent() = default;

	MotionEvent(const MotionEvent &) = default;
	MotionEvent &operator=(const MotionEvent &) = default;

	bool GetIsTickEvent() const { return m_isTickEvent; }
	void SetIsTickEvent(bool isTick) { m_isTickEvent = isTick; }
	float GetStartTime() const { return m_startTime; }
	void SetStartTime(float t) { m_startTime = t; }
	float GetEndTime() const { return m_endTime; }
	void SetEndTime(float t) { m_endTime = t; }
	const std::vector<std::shared_ptr<godot::MotionMatching::EventData>> &GetEventDatas() const {
		return m_eventDatas;
	}
	void AddEventData(const std::shared_ptr<godot::MotionMatching::EventData> &data) {
		if (data) {
			m_eventDatas.push_back(data);
		}
	}

private:
	bool m_isTickEvent = true;
	float m_startTime = 0.0f;
	float m_endTime = 0.0f;
	std::vector<std::shared_ptr<godot::MotionMatching::EventData>> m_eventDatas;
};
