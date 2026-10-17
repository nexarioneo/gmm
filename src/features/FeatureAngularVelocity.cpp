#include "FeatureAngularVelocity.h"

#include <cassert>

#include <godot_cpp/classes/project_settings.hpp>

#include "ActorInstance.h"
#include "AnimGraphPose.h"
#include "AnimGraphPosePool.h"
#include "EMotionFXManager.h"
#include "EventManager.h"
#include "Motion.h"
#include "Pose.h"
#include "Transform.h"
#include "TransformData.h"
#include "Velocity.h"
#include "../database/Frame.h"
#include "../database/FrameDatabase.h"
#include "../math/FeatureMatrixTransformer.h"
#include "../runtime/MotionMatchingInstance.h"
#include "../runtime/PoseDataJointVelocities.h"

namespace godot::MotionMatching {

static float GetDebugDrawAngularVelocityScale() {
	ProjectSettings *settings = ProjectSettings::get_singleton();
	if (settings == nullptr) {
		return 0.1f;
	}
	const double scale = settings->get_setting(
		"motion_matching/debug_draw_velocity_scale", 0.1);
	return static_cast<float>(scale);
}

void FeatureAngularVelocity::ExtractFeatureValues(const ExtractFeatureContext &context) {
	const ActorInstance *actorInstance = context.m_actorInstance;
	if (context.m_frameDatabase == nullptr || context.m_frameIndex >= context.m_frameDatabase->GetNumFrames()) {
		return;
	}
	const Frame &frame = context.m_frameDatabase->GetFrame(context.m_frameIndex);

	AnimGraphPose *tempPose = context.m_posePool.RequestPose(actorInstance);
	if (tempPose == nullptr) {
		return;
	}
	{

		PoseDataJointVelocities *velocityPoseData = tempPose->GetPose().GetAndPreparePoseData<PoseDataJointVelocities>(actorInstance);
		if (velocityPoseData == nullptr) {
			context.m_posePool.FreePose(tempPose);
			return;
		}
		velocityPoseData->CalculateVelocity(
			actorInstance, context.m_posePool, frame.GetSourceMotion(), frame.GetSampleTime(), m_relativeToNodeIndex);

		if (m_jointIndex >= velocityPoseData->GetAngularVelocities().size()) {
			context.m_posePool.FreePose(tempPose);
			return;
		}
		const Vector3 &angularVelocity = velocityPoseData->GetAngularVelocities()[m_jointIndex];
		context.m_featureMatrix.SetVector3(context.m_frameIndex, m_featureColumnOffset, angularVelocity);
	}
	context.m_posePool.FreePose(tempPose);
}

void FeatureAngularVelocity::FillQueryVector(QueryVector &queryVector, const QueryVectorContext &context) {
	const PoseDataJointVelocities *velocityPoseData = context.m_currentPose.GetPoseData<PoseDataJointVelocities>();
	if (velocityPoseData == nullptr) {
		return;
	}
	const Vector3 currentVelocity = velocityPoseData->GetAngularVelocity(m_jointIndex);

	queryVector.SetVector3(currentVelocity, m_featureColumnOffset);
}

float FeatureAngularVelocity::CalculateFrameCost(size_t frameIndex, const FrameCostContext &context) const {
	const Vector3 queryVelocity = context.m_queryVector.GetVector3(m_featureColumnOffset);
	const Vector3 frameVelocity = context.m_featureMatrix.GetVector3(frameIndex, m_featureColumnOffset);

	return CalcResidual(queryVelocity, frameVelocity);
}

void FeatureAngularVelocity::DebugDraw(DebugDrawOutput &debugDisplay,
	const Pose &pose,
	const Vector3 &velocity,
	size_t jointIndex,
	size_t relativeToJointIndex,
	const Color &color) {
	const Transform jointModelTM = pose.GetModelSpaceTransform(jointIndex);
	const Transform relativeToWorldTM = pose.GetWorldSpaceTransform(relativeToJointIndex);

	const Vector3 jointPosition = relativeToWorldTM.TransformPoint(jointModelTM.m_position);
	const Vector3 velocityWorldSpace = relativeToWorldTM.TransformVector(velocity);

	DebugDrawVelocity(debugDisplay, jointPosition, velocityWorldSpace * GetDebugDrawAngularVelocityScale(), color);
}

void FeatureAngularVelocity::DebugDraw(DebugDrawOutput &debugDisplay,
	const Pose &currentPose,
	const FeatureMatrix &featureMatrix,
	const FeatureMatrixTransformer *featureTransformer,
	size_t frameIndex) {
	if (m_jointIndex == InvalidIndex) {
		return;
	}

	Vector3 angularVelocity = featureMatrix.GetVector3(frameIndex, m_featureColumnOffset);
	if (featureTransformer) {
		angularVelocity = featureTransformer->InverseTransform(angularVelocity, m_featureColumnOffset);
	}
	DebugDraw(debugDisplay, currentPose, angularVelocity, m_jointIndex, m_relativeToNodeIndex, m_debugColor);
}

void FeatureAngularVelocity::_bind_methods() {

}

size_t FeatureAngularVelocity::GetNumDimensions() const {
	return 3;
}

std::string FeatureAngularVelocity::GetDimensionName(size_t index) const {
	std::string result = m_jointName;
	result += '.';

	switch (index) {
		case 0: {
			result += "AngularVelocityX";
			break;
		}
		case 1: {
			result += "AngularVelocityY";
			break;
		}
		case 2: {
			result += "AngularVelocityZ";
			break;
		}
		default: {
			result += Feature::GetDimensionName(index);
		}
	}

	return result;
}

}
