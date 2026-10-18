#pragma once

#include <cstddef>
#include <string>

#include <godot_cpp/variant/vector3.hpp>

#include "features/feature.h"
#include "core/math/feature_matrix.h"

namespace godot::MotionMatching {
class FrameDatabase;

class FeaturePosition : public Feature {
	GDCLASS(FeaturePosition, Feature)

public:
	FeaturePosition() = default;
	~FeaturePosition() override = default;

	void ExtractFeatureValues(const ExtractFeatureContext &context) override;
	void FillQueryVector(QueryVector &queryVector, const QueryVectorContext &context) override;
	float CalculateFrameCost(size_t frameIndex, const FrameCostContext &context) const override;

	void DebugDraw(DebugDrawOutput &debugDisplay,
		const Pose &currentPose,
		const FeatureMatrix &featureMatrix,
		const FeatureMatrixTransformer *featureTransformer,
		size_t frameIndex) override;

	size_t GetNumDimensions() const override;
	std::string GetDimensionName(size_t index) const override;
	Vector3 GetFeatureData(const FeatureMatrix &featureMatrix, size_t frameIndex) const;

protected:
	static void _bind_methods();
};
}
