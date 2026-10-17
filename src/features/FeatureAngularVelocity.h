#pragma once

#include <cstddef>
#include <string>

#include <godot_cpp/variant/color.hpp>
#include <godot_cpp/variant/vector3.hpp>

#include "Feature.h"
#include "../math/FeatureMatrix.h"

namespace godot::MotionMatching {
class FrameDatabase;

class FeatureAngularVelocity : public Feature {
	GDCLASS(FeatureAngularVelocity, Feature)

public:
	FeatureAngularVelocity() = default;
	~FeatureAngularVelocity() override = default;

	void ExtractFeatureValues(const ExtractFeatureContext &context) override;
	void FillQueryVector(QueryVector &queryVector, const QueryVectorContext &context) override;
	float CalculateFrameCost(size_t frameIndex, const FrameCostContext &context) const override;

	static void DebugDraw(DebugDrawOutput &debugDisplay,
		const Pose &pose,
		const Vector3 &velocity,
		size_t jointIndex,
		size_t relativeToJointIndex,
		const Color &color);

	void DebugDraw(DebugDrawOutput &debugDisplay,
		const Pose &currentPose,
		const FeatureMatrix &featureMatrix,
		const FeatureMatrixTransformer *featureTransformer,
		size_t frameIndex) override;

	size_t GetNumDimensions() const override;
	std::string GetDimensionName(size_t index) const override;

protected:
	static void _bind_methods();
};
}
