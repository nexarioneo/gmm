#include "pose/pose.h"

#include <cstdint>

#include <godot_cpp/classes/skeleton3d.hpp>
#include <godot_cpp/variant/basis.hpp>
#include <godot_cpp/variant/transform3d.hpp>

#include "actor/actor.h"
#include "actor/actor_instance.h"
#include "core/types/skeleton.h"

namespace {

godot::Skeleton3D *ResolveSkeletonFromActor(const ActorInstance *actorInstance) {
	if (!actorInstance) {
		return nullptr;
	}
	const Actor *actor = actorInstance->GetActor();
	if (!actor) {
		return nullptr;
	}
	const Skeleton *skeleton = actor->GetSkeleton();
	return skeleton ? skeleton->GetLinkedSkeleton() : nullptr;
}

}

void Pose::LinkSkeleton(godot::Skeleton3D *skeleton) {
	m_skeleton = skeleton;
}

godot::Skeleton3D *Pose::ResolveLiveSkeleton() const {

	godot::Skeleton3D *viaActor = ResolveSkeletonFromActor(m_actorInstance);
	return viaActor ? viaActor : m_skeleton;
}

void Pose::LinkToActorInstance(ActorInstance *actorInstance) {
	m_actorInstance = actorInstance;
	godot::Skeleton3D *viaActor = ResolveSkeletonFromActor(actorInstance);
	if (viaActor) {
		m_skeleton = viaActor;
	}
}

void Pose::LinkToActorInstance(const ActorInstance *actorInstance) {
	LinkToActorInstance(const_cast<ActorInstance *>(actorInstance));
}

void Pose::InitFromBindPose(ActorInstance *actorInstance) {
	m_actorInstance = actorInstance;
	godot::Skeleton3D *skeleton = ResolveSkeletonFromActor(actorInstance);
	if (skeleton) {
		m_skeleton = skeleton;
	} else {
		skeleton = m_skeleton;
	}

	size_t count = 0;
	if (actorInstance) {
		count = actorInstance->GetNumNodes();
	}
	if (count == 0 && skeleton) {

		const int32_t boneCount = skeleton->get_bone_count();
		if (boneCount > 0) {
			count = static_cast<size_t>(boneCount);
		}
	}
	m_transforms.resize(count, Transform::CreateIdentity());
	if (!skeleton) {
		return;
	}

	for (size_t i = 0; i < count; ++i) {
		const godot::Transform3D rest = skeleton->get_bone_rest(static_cast<int32_t>(i));
		Transform bind;
		bind.m_position = rest.get_origin();
		bind.m_rotation = rest.get_basis().get_rotation_quaternion();
		m_transforms[i] = bind;
	}
}

void Pose::InitFromBindPose(const ActorInstance *actorInstance) {
	InitFromBindPose(const_cast<ActorInstance *>(actorInstance));
}

Transform Pose::GetWorldSpaceTransform(size_t jointIndex) const {
	godot::Skeleton3D *skeleton = ResolveLiveSkeleton();
	if (skeleton) {

		const int32_t boneCount = skeleton->get_bone_count();
		if (boneCount > 0 && jointIndex < static_cast<size_t>(boneCount)) {
			const godot::Transform3D globalPose =
				skeleton->get_bone_global_pose(static_cast<int32_t>(jointIndex));
			Transform result;
			result.m_position = globalPose.get_origin();
			result.m_rotation = globalPose.get_basis().get_rotation_quaternion();
			return result;
		}
	}
	return GetTransformSafe(jointIndex);
}

Transform Pose::GetModelSpaceTransform(size_t jointIndex) const {

	return GetWorldSpaceTransform(jointIndex);
}

Transform Pose::GetLocalSpaceTransform(size_t jointIndex) const {
	godot::Skeleton3D *skeleton = ResolveLiveSkeleton();
	if (skeleton) {

		const int32_t boneCount = skeleton->get_bone_count();
		if (boneCount > 0 && jointIndex < static_cast<size_t>(boneCount)) {
			const int32_t bone = static_cast<int32_t>(jointIndex);
			Transform result;
			result.m_position = skeleton->get_bone_pose_position(bone);
			result.m_rotation = skeleton->get_bone_pose_rotation(bone);
			return result;
		}
	}
	return GetTransformSafe(jointIndex);
}

void Pose::CompensateForMotionExtraction() {
	if (m_actorInstance == nullptr) {
		return;
	}
	Actor *actor = m_actorInstance->GetActor();
	if (actor == nullptr || !actor->GetMotionExtractionEnabled()) {
		return;
	}

	size_t rootIndex = actor->GetMotionExtractionNodeIndex();
	if (rootIndex >= m_transforms.size()) {
		if (m_transforms.empty()) {
			return;
		}
		rootIndex = 0;
	}
	m_transforms[rootIndex].m_position = godot::Vector3();
}

void Pose::DebugDraw(godot::MotionMatching::DebugDrawOutput &debugDisplay,
	const godot::Color &color) const {
	const size_t count = m_transforms.size();
	for (size_t i = 0; i < count; ++i) {
		const Transform world = GetWorldSpaceTransform(i);
		debugDisplay.ball_centers.push_back(world.m_position);
		debugDisplay.ball_sizes.push_back(0.015f);
		debugDisplay.ball_colors.push_back(color);
	}
}
