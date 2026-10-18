#pragma once

#include <cstddef>

#include <godot_cpp/variant/color.hpp>

namespace godot::MotionMatching {

class MotionMatchingMonitorRequests {
public:
	virtual ~MotionMatchingMonitorRequests() = default;

	virtual void PushPerformanceHistogramValue(const char *performanceMetricName, float value) = 0;
	virtual void PushCostHistogramValue(const char *costName, float value, const Color &color) = 0;

	struct FrameDatabaseInfo {
		size_t m_memoryUsedInBytes = 0;
		size_t m_numFrames = 0;
		size_t m_numMotions = 0;
		float m_motionDataInSeconds = 0.0f;
	};
	virtual void SetFrameDatabaseInfo(const FrameDatabaseInfo &info) = 0;

	struct FeatureMatrixInfo {
		size_t m_memoryUsedInBytes = 0;
		size_t m_numFrames = 0;
		size_t m_numDimensions = 0;
	};
	virtual void SetFeatureMatrixInfo(const FeatureMatrixInfo &info) = 0;

	struct KdTreeInfo {
		size_t m_memoryUsedInBytes = 0;
		size_t m_numNodes = 0;
		size_t m_numDimensions = 0;
	};
	virtual void SetKdTreeInfo(const KdTreeInfo &info) = 0;
};

class MotionMatchingMonitorRequestBus {
public:
	static void SetHandler(MotionMatchingMonitorRequests *handler) { Handler() = handler; }

	template <typename... MArgs, typename... Args>
	static void Broadcast(void (MotionMatchingMonitorRequests::*method)(MArgs...), Args &&...args) {
		MotionMatchingMonitorRequests *handler = Handler();
		if (handler) {
			(handler->*method)(args...);
		}
	}

private:
	static MotionMatchingMonitorRequests *&Handler() {
		static MotionMatchingMonitorRequests *handler = nullptr;
		return handler;
	}
};

}
