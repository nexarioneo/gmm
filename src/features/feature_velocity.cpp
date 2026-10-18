#include "features/feature_velocity.h"

#include <cassert>

#include <godot_cpp/classes/project_settings.hpp>

#include "actor/actor_instance.h"
#include "actor/anim_graph_pose.h"
#include "actor/anim_graph_pose_pool.h"
#include "animation/motion.h"
#include "pose/pose.h"
#include "core/types/transform.h"
#include "pose/transform_data.h"
#include "database/frame.h"
#include "database/frame_database.h"
#include "core/math/feature_matrix_transformer.h"
#include "runtime/motion_matching_instance.h"
#include "pose/pose_data_joint_velocities.h"

namespace godot::MotionMatching {

static float GetDebugDrawVelocityScale() {
	ProjectSettings *settings = ProjectSettings::get_singleton();
	if (settings == nullptr) {
		return static_cast<float>(0.1);
	}
	const double scale = settings->get_setting("motion_matching/debug_draw_velocity_scale", 0.1);
	return static_cast<float>(scale);
}

void FeatureVelocity::ExtractFeatureValues(const ExtractFeatureContext &context) {
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
		velocityPoseData->CalculateVelocity(actorInstance,
			context.m_posePool,
			frame.GetSourceMotion(),
			frame.GetSampleTime(),
			m_relativeToNodeIndex);

		if (m_jointIndex >= velocityPoseData->GetVelocities().size()) {
			context.m_posePool.FreePose(tempPose);
			return;
		}
		const Vector3 &velocity = velocityPoseData->GetVelocities()[m_jointIndex];
		context.m_featureMatrix.SetVector3(context.m_frameIndex, m_featureColumnOffset, velocity);
	}
	context.m_posePool.FreePose(tempPose);
}

void FeatureVelocity::FillQueryVector(QueryVector &queryVector, const QueryVectorContext &context) {
	PoseDataJointVelocities *velocityPoseData = context.m_currentPose.GetPoseData<PoseDataJointVelocities>();
	if (velocityPoseData == nullptr) {
		return;
	}
	const Vector3 currentVelocity = velocityPoseData->GetVelocity(m_jointIndex);

	queryVector.SetVector3(currentVelocity, m_featureColumnOffset);
}

float FeatureVelocity::CalculateFrameCost(size_t frameIndex, const FrameCostContext &context) const {
	const Vector3 queryVelocity = context.m_queryVector.GetVector3(m_featureColumnOffset);
	const Vector3 frameVelocity = context.m_featureMatrix.GetVector3(frameIndex, m_featureColumnOffset);

	return CalcResidual(queryVelocity, frameVelocity);
}

void FeatureVelocity::DebugDraw(DebugDrawOutput &debugDisplay,
	const Pose &pose,
	const Vector3 &velocity,
	size_t jointIndex,
	size_t relativeToJointIndex,
	const Color &color) {
	const Transform jointModelTM = pose.GetModelSpaceTransform(jointIndex);
	const Transform relativeToWorldTM = pose.GetWorldSpaceTransform(relativeToJointIndex);

	const Vector3 jointPosition = relativeToWorldTM.TransformPoint(jointModelTM.m_position);
	const Vector3 velocityWorldSpace = relativeToWorldTM.TransformVector(velocity);

	DebugDrawVelocity(debugDisplay, jointPosition, velocityWorldSpace * GetDebugDrawVelocityScale(), color);
}

void FeatureVelocity::DebugDraw(DebugDrawOutput &debugDisplay,
	const Pose &currentPose,
	const FeatureMatrix &featureMatrix,
	const FeatureMatrixTransformer *featureTransformer,
	size_t frameIndex) {
	if (m_jointIndex == InvalidIndex) {
		return;
	}

	Vector3 velocity = featureMatrix.GetVector3(frameIndex, m_featureColumnOffset);
	if (featureTransformer) {
		velocity = featureTransformer->InverseTransform(velocity, m_featureColumnOffset);
	}
	DebugDraw(debugDisplay, currentPose, velocity, m_jointIndex, m_relativeToNodeIndex, m_debugColor);
}

void FeatureVelocity::_bind_methods() {

}

size_t FeatureVelocity::GetNumDimensions() const {
	return 3;
}

std::string FeatureVelocity::GetDimensionName(size_t index) const {
	std::string result = m_jointName;
	result += '.';

	switch (index) {
		case 0: {
			result += "VelocityX";
			break;
		}
		case 1: {
			result += "VelocityY";
			break;
		}
		case 2: {
			result += "VelocityZ";
			break;
		}
		default: {
			result += Feature::GetDimensionName(index);
		}
	}

	return result;
}

}