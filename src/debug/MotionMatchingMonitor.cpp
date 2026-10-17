#include "MotionMatchingMonitor.h"

namespace godot::MotionMatching {

void MotionMatchingMonitor::Histogram::PushHistogramValue(const char *label, float value, const Color &color) {
	std::lock_guard<std::mutex> lock(m_mutex);
	m_values.push_back(HistogramEntry{ label ? label : "", value, color });
	while (m_values.size() > m_binCount) {
		m_values.erase(m_values.begin());
	}
}

MotionMatchingMonitor::Histogram MotionMatchingMonitor::CopyPerformanceStats() const {
	Histogram copy;
	{
		std::lock_guard<std::mutex> lock(m_performanceStats.m_mutex);
		copy.m_name = m_performanceStats.m_name;
		copy.m_binCount = m_performanceStats.m_binCount;
		copy.m_show = m_performanceStats.m_show;
		copy.m_values = m_performanceStats.m_values;
	}
	return copy;
}

MotionMatchingMonitor::Histogram MotionMatchingMonitor::CopyFeatureCosts() const {
	Histogram copy;
	{
		std::lock_guard<std::mutex> lock(m_featureCosts.m_mutex);
		copy.m_name = m_featureCosts.m_name;
		copy.m_binCount = m_featureCosts.m_binCount;
		copy.m_show = m_featureCosts.m_show;
		copy.m_values = m_featureCosts.m_values;
	}
	return copy;
}

MotionMatchingMonitor::MotionMatchingMonitor() {
	m_performanceStats.SetName("Performance Statistics");
	m_performanceStats.SetHistogramBinCount(500);

	m_featureCosts.SetName("Feature Costs");
	m_featureCosts.SetHistogramBinCount(100);

	MotionMatchingMonitorRequestBus::SetHandler(this);
}

MotionMatchingMonitor::~MotionMatchingMonitor() {
	MotionMatchingMonitorRequestBus::SetHandler(nullptr);
}

void MotionMatchingMonitor::PushPerformanceHistogramValue(const char *performanceMetricName, float value) {
	m_performanceStats.PushHistogramValue(performanceMetricName, value, Color(229.0f / 255.0f, 56.0f / 255.0f, 59.0f / 255.0f, 1.0f));
}

void MotionMatchingMonitor::PushCostHistogramValue(const char *costName, float value, const Color &color) {
	m_featureCosts.PushHistogramValue(costName, value, color);
}

MotionMatchingMonitorRequests::FrameDatabaseInfo MotionMatchingMonitor::CopyFrameDatabaseInfo() const {
	std::lock_guard<std::mutex> lock(m_mutex);
	return m_frameDatabaseInfo;
}

MotionMatchingMonitorRequests::FeatureMatrixInfo MotionMatchingMonitor::CopyFeatureMatrixInfo() const {
	std::lock_guard<std::mutex> lock(m_mutex);
	return m_featureMatrixInfo;
}

MotionMatchingMonitorRequests::KdTreeInfo MotionMatchingMonitor::CopyKdTreeInfo() const {
	std::lock_guard<std::mutex> lock(m_mutex);
	return m_kdTreeInfo;
}

}
