#pragma once

#include <cstddef>
#include <mutex>
#include <string>
#include <vector>

#include <godot_cpp/variant/color.hpp>

#include "MotionMatchingMonitorBus.h"

namespace godot::MotionMatching {

class MotionMatchingMonitor : public MotionMatchingMonitorRequests {
public:
	MotionMatchingMonitor();
	~MotionMatchingMonitor() override;

	void PushPerformanceHistogramValue(const char *performanceMetricName, float value) override;
	void PushCostHistogramValue(const char *costName, float value, const Color &color) override;

	void SetFrameDatabaseInfo(const FrameDatabaseInfo &info) override {
		std::lock_guard<std::mutex> lock(m_mutex);
		m_frameDatabaseInfo = info;
	}
	void SetFeatureMatrixInfo(const FeatureMatrixInfo &info) override {
		std::lock_guard<std::mutex> lock(m_mutex);
		m_featureMatrixInfo = info;
	}
	void SetKdTreeInfo(const KdTreeInfo &info) override {
		std::lock_guard<std::mutex> lock(m_mutex);
		m_kdTreeInfo = info;
	}

	struct HistogramEntry {
		std::string m_label;
		float m_value = 0.0f;
		Color m_color;
	};
	struct Histogram {
		void SetName(const char *name) { m_name = name; }
		void SetHistogramBinCount(size_t bins) { m_binCount = bins; }
		void PushHistogramValue(const char *label, float value, const Color &color);

		std::string m_name;
		size_t m_binCount = 100;
		bool m_show = false;
		std::vector<HistogramEntry> m_values;
		mutable std::mutex m_mutex;
	};

	const Histogram &GetPerformanceStats() const { return m_performanceStats; }
	const Histogram &GetFeatureCosts() const { return m_featureCosts; }
	const FrameDatabaseInfo &GetFrameDatabaseInfo() const { return m_frameDatabaseInfo; }
	const FeatureMatrixInfo &GetFeatureMatrixInfo() const { return m_featureMatrixInfo; }
	const KdTreeInfo &GetKdTreeInfo() const { return m_kdTreeInfo; }

	Histogram CopyPerformanceStats() const;
	Histogram CopyFeatureCosts() const;
	FrameDatabaseInfo CopyFrameDatabaseInfo() const;
	FeatureMatrixInfo CopyFeatureMatrixInfo() const;
	KdTreeInfo CopyKdTreeInfo() const;

private:
	mutable std::mutex m_mutex;
	Histogram m_performanceStats;
	Histogram m_featureCosts;

	FrameDatabaseInfo m_frameDatabaseInfo;
	FeatureMatrixInfo m_featureMatrixInfo;
	KdTreeInfo m_kdTreeInfo;
};

}
