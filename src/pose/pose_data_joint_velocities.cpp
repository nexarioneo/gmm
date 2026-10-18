#include "pose/pose_data_joint_velocities.h"

#include <algorithm>
#include <cassert>

#include "actor/actor.h"
#include "actor/actor_instance.h"
#include "actor/anim_graph_pose.h"
#include "actor/anim_graph_pose_pool.h"
#include "animation/motion.h"
#include "animation/motion_data.h"
#include "animation/motion_instance.h"
#include "pose/pose.h"
#include "core/types/transform.h"
#include "pose/transform_data.h"
#include "core/types/velocity.h"
#include "features/feature.h"

namespace godot::MotionMatching {

PoseDataJointVelocities::PoseDataJointVelocities()
	: PoseData() {
}

PoseDataJointVelocities::~PoseDataJointVelocities() {
	Clear();
}

void PoseDataJointVelocities::Clear() {
	m_velocities.clear();
	m_angularVelocities.clear();
}

void PoseDataJointVelocities::LinkToActorInstance(const ActorInstance *actorInstance) {
	if (!actorInstance || !actorInstance->GetActor()) {
		Clear();
		return;
	}
	m_velocities.resize(actorInstance->GetNumNodes());
	m_angularVelocities.resize(actorInstance->GetNumNodes());

	SetRelativeToJointIndex(actorInstance->GetActor()->GetMotionExtractionNodeIndex());
}

void PoseDataJointVelocities::SetRelativeToJointIndex(size_t relativeToJointIndex) {
	if (relativeToJointIndex == InvalidIndex) {
		m_relativeToJointIndex = 0;
	} else {
		m_relativeToJointIndex = relativeToJointIndex;
	}
}

void PoseDataJointVelocities::LinkToActor([[maybe_unused]] const Actor *actor) {
	Clear();
}

void PoseDataJointVelocities::Reset() {
	const size_t numJoints = m_velocities.size();
	for (size_t i = 0; i < numJoints; ++i) {
		m_velocities[i] = Vector3();
		m_angularVelocities[i] = Vector3();
	}
}

void PoseDataJointVelocities::CopyFrom(const PoseData *from) {

	if (!from) {
		return;
	}
	assert(dynamic_cast<const PoseDataJointVelocities *>(from) != nullptr && "Cannot copy from pose data other than joint velocity pose data.");
	const PoseDataJointVelocities *fromVelocityPoseData = static_cast<const PoseDataJointVelocities *>(from);

	m_isUsed = fromVelocityPoseData->m_isUsed;
	m_velocities = fromVelocityPoseData->m_velocities;
	m_angularVelocities = fromVelocityPoseData->m_angularVelocities;
	m_relativeToJointIndex = fromVelocityPoseData->m_relativeToJointIndex;
}

std::shared_ptr<PoseData> PoseDataJointVelocities::Clone() const {
	auto copy = std::make_shared<PoseDataJointVelocities>();
	copy->CopyFrom(this);
	return copy;
}

void PoseDataJointVelocities::Blend(const Pose *destPose, float weight) {
	if (!destPose) {
		return;
	}
	const PoseDataJointVelocities *destPoseData = destPose->GetPoseData<PoseDataJointVelocities>();

	if (destPoseData && destPoseData->IsUsed()) {
		assert(m_velocities.size() == destPoseData->m_velocities.size() && "Expected the same number of joints and velocities in the destination pose data.");

		if (m_isUsed) {

			for (size_t i = 0; i < m_velocities.size(); ++i) {
				m_velocities[i] = m_velocities[i].lerp(destPoseData->m_velocities[i], weight);
				m_angularVelocities[i] = m_angularVelocities[i].lerp(destPoseData->m_angularVelocities[i], weight);
			}
		} else {

			m_velocities = destPoseData->m_velocities;
			m_angularVelocities = destPoseData->m_angularVelocities;
		}
	} else {

	}
}

void PoseDataJointVelocities::DebugDraw(DebugDrawOutput &debugDisplay, const Color &color) const {
	if (!m_pose) {
		return;
	}
	assert(m_pose->GetNumTransforms() == m_velocities.size() && "Expected a joint velocity for each joint in the pose.");

	const Pose *pose = m_pose;
	for (size_t i = 0; i < m_velocities.size(); ++i) {
		const size_t jointIndex = i;

		{
			const Transform jointModelTM = pose->GetModelSpaceTransform(jointIndex);
			const Transform relativeToWorldTM = pose->GetWorldSpaceTransform(m_relativeToJointIndex);
			const Vector3 jointPosition = relativeToWorldTM.TransformPoint(jointModelTM.m_position);

			const Vector3 &velocity = m_velocities[i];
			const Vector3 velocityWorldSpace = relativeToWorldTM.TransformVector(velocity);

			const float scale = 0.1f;
			DebugDrawVelocity(debugDisplay, jointPosition, velocityWorldSpace * scale, color);
		}
	}
}

void PoseDataJointVelocities::CalculateVelocity(const ActorInstance *actorInstance, AnimGraphPosePool &posePool, Motion *motion, float requestedSampleTime, size_t relativeToJointIndex) {
	if (!actorInstance || !motion || !actorInstance->GetTransformData() || !actorInstance->GetTransformData()->GetBindPose()) {
		return;
	}
	MotionDataSampleSettings sampleSettings;
	sampleSettings.m_actorInstance = const_cast<ActorInstance *>(actorInstance);
	sampleSettings.m_inPlace = false;
	sampleSettings.m_mirror = false;
	sampleSettings.m_retarget = false;
	sampleSettings.m_inputPose = sampleSettings.m_actorInstance->GetTransformData()->GetBindPose();

	const size_t numJoints = m_velocities.size();
	SetRelativeToJointIndex(relativeToJointIndex);
	m_velocities.resize(numJoints);
	m_angularVelocities.resize(numJoints);

	AnimGraphPose *prevPose = posePool.RequestPose(actorInstance);
	AnimGraphPose *currentPose = posePool.RequestPose(actorInstance);
	if (!prevPose || !currentPose) {
		if (prevPose) {
			posePool.FreePose(prevPose);
		}
		if (currentPose) {
			posePool.FreePose(currentPose);
		}
		return;
	}

	const size_t numSamples = 3;
	const float timeRange = 0.05f;

	const float halfTimeRange = timeRange * 0.5f;
	const float startTime = requestedSampleTime - halfTimeRange;
	const float numInbetweens = static_cast<float>(numSamples - 1);
	const float frameDelta = timeRange / numInbetweens;
	const float motionDuration = motion->GetDuration();

	Reset();

	for (size_t sampleIndex = 0; sampleIndex < numSamples; ++sampleIndex) {
		float sampleTime = startTime + sampleIndex * frameDelta;
		sampleTime = std::clamp(sampleTime, 0.0f, motionDuration);

		if (sampleIndex == 0) {
			sampleSettings.m_sampleTime = sampleTime;
			motion->SamplePose(&prevPose->GetPose(), sampleSettings);
			continue;
		}

		sampleSettings.m_sampleTime = sampleTime;
		motion->SamplePose(&currentPose->GetPose(), sampleSettings);

		const Transform inverseJointWorldTransform = currentPose->GetPose().GetWorldSpaceTransform(relativeToJointIndex).Inversed();

		for (size_t jointIndex = 0; jointIndex < numJoints; ++jointIndex) {
			const Transform prevWorldTransform = prevPose->GetPose().GetWorldSpaceTransform(jointIndex);
			const Transform currentWorldTransform = currentPose->GetPose().GetWorldSpaceTransform(jointIndex);

			const Vector3 prevPosition = inverseJointWorldTransform.TransformPoint(prevWorldTransform.m_position);
			const Vector3 currentPosition = inverseJointWorldTransform.TransformPoint(currentWorldTransform.m_position);

			const Vector3 linearVelocity = CalculateLinearVelocity(prevPosition, currentPosition, frameDelta);
			m_velocities[jointIndex] += linearVelocity;

			const Quaternion prevRotation = inverseJointWorldTransform.m_rotation * prevWorldTransform.m_rotation;
			const Quaternion currentRotation = inverseJointWorldTransform.m_rotation * currentWorldTransform.m_rotation;

			const Vector3 angularVelocity = CalculateAngularVelocity(prevRotation, currentRotation, frameDelta);
			m_angularVelocities[jointIndex] += angularVelocity;
		}

		*prevPose = *currentPose;
	}

	for (size_t i = 0; i < numJoints; ++i) {
		m_velocities[i] /= numInbetweens;
		m_angularVelocities[i] /= numInbetweens;
	}

	posePool.FreePose(prevPose);
	posePool.FreePose(currentPose);
}

}
