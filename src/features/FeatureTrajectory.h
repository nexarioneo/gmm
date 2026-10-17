#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>

#include <godot_cpp/core/property_info.hpp>
#include <godot_cpp/templates/list.hpp>
#include <godot_cpp/variant/color.hpp>
#include <godot_cpp/variant/string.hpp>
#include <godot_cpp/variant/string_name.hpp>
#include <godot_cpp/variant/variant.hpp>
#include <godot_cpp/variant/vector2.hpp>
#include <godot_cpp/variant/vector3.hpp>

#include "Feature.h"
#include "../math/FeatureMatrix.h"
#include "../debug/MotionMatchingCompat.h"

class Pose;
class Transform;

namespace godot::MotionMatching {
class FrameDatabase;

class FeatureTrajectory : public Feature {
	GDCLASS(FeatureTrajectory, Feature)

public:
	enum class Axis {
		X = 0,
		Y = 1,
		X_NEGATIVE = 2,
		Y_NEGATIVE = 3,
		Z = 4,
		Z_NEGATIVE = 5,
	};

	struct Sample {
		Vector2 m_position;
		Vector2 m_facingDirection;

		static constexpr size_t s_componentsPerSample = 4;
	};

	FeatureTrajectory() = default;
	~FeatureTrajectory() override = default;

	bool Init(const InitSettings &settings) override;

	void ExtractFeatureValues(const ExtractFeatureContext &context) override;
	void FillQueryVector(QueryVector &queryVector, const QueryVectorContext &context) override;

	float CalculateFutureFrameCost(size_t frameIndex, const FrameCostContext &context) const;
	float CalculatePastFrameCost(size_t frameIndex, const FrameCostContext &context) const;

	void DebugDraw(DebugDrawOutput &debugDisplay,
		const Pose &currentPose,
		const FeatureMatrix &featureMatrix,
		const FeatureMatrixTransformer *featureTransformer,
		size_t frameIndex) override;

	void SetNumPastSamplesPerFrame(size_t numHistorySamples);
	void SetNumFutureSamplesPerFrame(size_t numFutureSamples);
	void SetPastTimeRange(float timeInSeconds);
	void SetFutureTimeRange(float timeInSeconds);
	void SetFacingAxis(const Axis axis);
	void UpdateFacingAxis();

	float GetPastTimeRange() const { return m_pastTimeRange; }
	size_t GetNumPastSamples() const { return m_numPastSamples; }
	int64_t get_num_past_samples() const;
	float GetPastCostFactor() const { return m_pastCostFactor; }

	float GetFutureTimeRange() const { return m_futureTimeRange; }
	size_t GetNumFutureSamples() const { return m_numFutureSamples; }
	int64_t get_num_future_samples() const;
	float GetFutureCostFactor() const { return m_futureCostFactor; }

	Vector2 CalculateFacingDirection(const Pose &pose, const Transform &invRootTransform) const;
	Vector3 GetFacingAxisDir() const { return m_facingAxisDir; }

	size_t GetNumDimensions() const override;
	std::string GetDimensionName(size_t index) const override;

	static void DebugDrawFacingDirection(DebugDrawOutput &debugDisplay,
		const Vector3 &positionWorldSpace,
		const Vector3 &facingDirectionWorldSpace,
		const Color &color);

	bool _set(const StringName &p_name, const Variant &p_value);
	bool _get(const StringName &p_name, Variant &r_ret) const;

protected:
	void AppendCustomProperties(List<PropertyInfo> *p_list) const override;
	static void _bind_methods();

private:
	size_t CalcMidFrameIndex() const;
	size_t CalcPastFrameIndex(size_t historyFrameIndex) const;
	size_t CalcFutureFrameIndex(size_t futureFrameIndex) const;
	size_t CalcNumSamplesPerFrame() const;

	using SplineToFeatureMatrixIndex = std::function<size_t(size_t)>;
	float CalculateCost(const FeatureMatrix &featureMatrix,
		size_t frameIndex,
		size_t numControlPoints,
		const SplineToFeatureMatrixIndex &splineToFeatureMatrixIndex,
		const FrameCostContext &context) const;

	Sample GetSampleFromPose(const Pose &pose, const Transform &invRootTransform) const;

	Sample GetFeatureData(const FeatureMatrix &featureMatrix, size_t frameIndex, size_t sampleIndex) const;
	void SetFeatureData(FeatureMatrix &featureMatrix, size_t frameIndex, size_t sampleIndex, const Sample &sample);

	Sample GetFeatureDataInverseTransformed(const FeatureMatrix &featureMatrix,
		const FeatureMatrixTransformer *featureTransformer,
		size_t frameIndex,
		size_t sampleIndex) const;

	void DebugDrawTrajectory(DebugDrawOutput &debugDisplay,
		const FeatureMatrix &featureMatrix,
		const FeatureMatrixTransformer *featureTransformer,
		size_t frameIndex,
		const Transform &transform,
		const Color &color,
		size_t numSamples,
		const SplineToFeatureMatrixIndex &splineToFeatureMatrixIndex) const;

	void DebugDrawFacingDirection(DebugDrawOutput &debugDisplay,
		const Transform &worldSpaceTransform,
		const Sample &sample,
		const Vector3 &samplePosWorldSpace,
		const Color &color) const;

	bool GetCostFactorVisibility() const override;

	float m_pastTimeRange = 0.7f;
	size_t m_numPastSamples = 4;
	float m_pastCostFactor = 0.5f;

	float m_futureTimeRange = 1.2f;
	size_t m_numFutureSamples = 6;
	float m_futureCostFactor = 0.75f;

	Axis m_facingAxis = Axis::Z_NEGATIVE;
	Vector3 m_facingAxisDir = Vector3(0.0f, 0.0f, -1.0f);
};
}
