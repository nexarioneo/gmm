#pragma once

#include <cstddef>
#include <vector>

#include <godot_cpp/variant/color.hpp>
#include <godot_cpp/variant/quaternion.hpp>
#include <godot_cpp/variant/vector3.hpp>

#include "../debug/MotionMatchingCompat.h"
#include "PoseData.h"
#include "Velocity.h"

class Actor;
class ActorInstance;
class AnimGraphPosePool;
class Motion;
class Pose;

namespace godot::MotionMatching {

class PoseDataJointVelocities
	: public PoseData {
public:
	PoseDataJointVelocities();
	~PoseDataJointVelocities();

	void Clear() override;

	void LinkToActorInstance(const ActorInstance *actorInstance) override;
	void LinkToActor(const Actor *actor) override;

	void Reset() override;

	void CopyFrom(const PoseData *from) override;
	std::shared_ptr<PoseData> Clone() const override;
	void Blend(const Pose *destPose, float weight) override;

	void CalculateVelocity(const ActorInstance *actorInstance, AnimGraphPosePool &posePool, Motion *motion, float requestedSampleTime, size_t relativeToJointIndex);

	void DebugDraw(DebugDrawOutput &debugDisplay, const Color &color) const override;

	std::vector<Vector3> &GetVelocities() { return m_velocities; }
	const std::vector<Vector3> &GetVelocities() const { return m_velocities; }
	Vector3 &GetVelocity(size_t jointIndex) {
		thread_local static Vector3 s_fallback;
		return (jointIndex < m_velocities.size()) ? m_velocities[jointIndex] : s_fallback;
	}
	const Vector3 &GetVelocity(size_t jointIndex) const {
		thread_local static const Vector3 s_fallback;
		return (jointIndex < m_velocities.size()) ? m_velocities[jointIndex] : s_fallback;
	}

	std::vector<Vector3> &GetAngularVelocities() { return m_angularVelocities; }
	const std::vector<Vector3> &GetAngularVelocities() const { return m_angularVelocities; }
	Vector3 &GetAngularVelocity(size_t jointIndex) {
		thread_local static Vector3 s_fallback;
		return (jointIndex < m_angularVelocities.size()) ? m_angularVelocities[jointIndex] : s_fallback;
	}
	const Vector3 &GetAngularVelocity(size_t jointIndex) const {
		thread_local static const Vector3 s_fallback;
		return (jointIndex < m_angularVelocities.size()) ? m_angularVelocities[jointIndex] : s_fallback;
	}

	size_t GetRelativeToJointIndex() const { return m_relativeToJointIndex; }
	void SetRelativeToJointIndex(size_t relativeToJointIndex);

private:
	std::vector<Vector3> m_velocities;
	std::vector<Vector3> m_angularVelocities;
	size_t m_relativeToJointIndex = InvalidIndex;
};
}
