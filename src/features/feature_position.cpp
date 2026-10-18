#include "features/feature_position.h"

#include "actor/actor_instance.h"
#include "pose/pose.h"
#include "core/types/transform.h"
#include "pose/transform_data.h"
#include "core/math/feature_matrix_transformer.h"
#include "database/frame_database.h"
#include "runtime/motion_matching_instance.h"

namespace godot::MotionMatching {

void FeaturePosition::ExtractFeatureValues(const ExtractFeatureContext &context) {
	if (context.m_framePose == nullptr) {
		return;
	}
	const Transform invRootTransform = context.m_framePose->GetWorldSpaceTransform(m_relativeToNodeIndex).Inversed();
	const Vector3 nodeWorldPosition = context.m_framePose->GetWorldSpaceTransform(m_jointIndex).m_position;
	const Vector3 position = invRootTransform.TransformPoint(nodeWorldPosition);

	context.m_featureMatrix.SetVector3(context.m_frameIndex, m_featureColumnOffset, position);
}

void FeaturePosition::FillQueryVector(QueryVector &queryVector, const QueryVectorContext &context) {
	const Transform invRootTransform = context.m_currentPose.GetWorldSpaceTransform(m_relativeToNodeIndex).Inversed();
	const Vector3 worldInputPosition = context.m_currentPose.GetWorldSpaceTransform(m_jointIndex).m_position;
	const Vector3 relativeInputPosition = invRootTransform.TransformPoint(worldInputPosition);

	queryVector.SetVector3(relativeInputPosition, m_featureColumnOffset);
}

float FeaturePosition::CalculateFrameCost(size_t frameIndex, const FrameCostContext &context) const {
	const Vector3 queryPosition = context.m_queryVector.GetVector3(m_featureColumnOffset);
	const Vector3 framePosition = GetFeatureData(context.m_featureMatrix, frameIndex);

	return CalcResidual(queryPosition, framePosition);
}

void FeaturePosition::DebugDraw(DebugDrawOutput &debugDisplay,
	const Pose &currentPose,
	const FeatureMatrix &featureMatrix,
	const FeatureMatrixTransformer *featureTransformer,
	size_t frameIndex) {
	const Transform relativeToWorldTM = currentPose.GetWorldSpaceTransform(m_relativeToNodeIndex);

	Vector3 position = GetFeatureData(featureMatrix, frameIndex);
	if (featureTransformer) {
		position = featureTransformer->InverseTransform(position, m_featureColumnOffset);
	}
	const Vector3 transformedPos = relativeToWorldTM.TransformPoint(position);

	constexpr float markerSize = 0.03f;
	debugDisplay.ball_centers.push_back(transformedPos);
	debugDisplay.ball_sizes.push_back(markerSize);
	debugDisplay.ball_colors.push_back(m_debugColor);
}

void FeaturePosition::_bind_methods() {

}

size_t FeaturePosition::GetNumDimensions() const {
	return 3;
}

std::string FeaturePosition::GetDimensionName(size_t index) const {
	std::string result = m_jointName;
	result += '.';

	switch (index) {
		case 0: {
			result += "PosX";
			break;
		}
		case 1: {
			result += "PosY";
			break;
		}
		case 2: {
			result += "PosZ";
			break;
		}
		default: {
			result += Feature::GetDimensionName(index);
		}
	}

	return result;
}

Vector3 FeaturePosition::GetFeatureData(const FeatureMatrix &featureMatrix, size_t frameIndex) const {
	return featureMatrix.GetVector3(frameIndex, m_featureColumnOffset);
}

}