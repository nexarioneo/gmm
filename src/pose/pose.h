#pragma once

#include <cstddef>
#include <memory>
#include <vector>

#include <godot_cpp/variant/color.hpp>

#include "debug/compat.h"
#include "pose/pose_data.h"
#include "core/types/transform.h"

class ActorInstance;

namespace godot {
class Skeleton3D;
}

class Pose {
public:
	Pose() = default;
	~Pose() = default;

	Pose(const Pose &other)
		: m_actorInstance(other.m_actorInstance)
		, m_skeleton(other.m_skeleton)
		, m_transforms(other.m_transforms) {
		for (const auto &data : other.m_poseDatas) {
			if (data) {
				auto cloned = data->Clone();
				if (cloned) {
					cloned->SetPose(this);
					m_poseDatas.push_back(cloned);
				}
			}
		}
	}

	Pose &operator=(const Pose &other) {
		if (this != &other) {
			m_actorInstance = other.m_actorInstance;
			m_skeleton = other.m_skeleton;
			m_transforms = other.m_transforms;
			m_poseDatas.clear();
			for (const auto &data : other.m_poseDatas) {
				if (data) {
					auto cloned = data->Clone();
					if (cloned) {
						cloned->SetPose(this);
						m_poseDatas.push_back(cloned);
					}
				}
			}
		}
		return *this;
	}

	ActorInstance *GetActorInstance() { return m_actorInstance; }
	ActorInstance *GetActorInstance() const { return m_actorInstance; }

	void LinkSkeleton(godot::Skeleton3D *skeleton);

	godot::Skeleton3D *GetLinkedSkeleton() const { return m_skeleton; }

	Transform GetWorldSpaceTransform(size_t jointIndex) const;

	Transform GetModelSpaceTransform(size_t jointIndex) const;

	Transform GetLocalSpaceTransform(size_t jointIndex) const;

	void SetWorldSpaceTransform(size_t jointIndex, const Transform &transform) {
		SetTransformSafe(jointIndex, transform);
	}
	void SetLocalSpaceTransform(size_t jointIndex, const Transform &transform) {
		SetTransformSafe(jointIndex, transform);
	}
	void SetModelSpaceTransform(size_t jointIndex, const Transform &transform) {
		SetTransformSafe(jointIndex, transform);
	}

	template <typename T>
	T *GetPoseData() {
		for (auto &data : m_poseDatas) {
			T *casted = dynamic_cast<T *>(data.get());
			if (casted) {
				return casted;
			}
		}
		return nullptr;
	}

	template <typename T>
	T *GetPoseData() const {
		for (const auto &data : m_poseDatas) {
			T *casted = dynamic_cast<T *>(data.get());
			if (casted) {
				return casted;
			}
		}
		return nullptr;
	}

	template <typename T>
	T *GetAndPreparePoseData(const ActorInstance *actorInstance) {
		T *existing = GetPoseData<T>();
		if (existing) {
			return existing;
		}
		auto newData = std::make_shared<T>();
		T *result = newData.get();
		result->SetPose(this);
		result->SetIsUsed(true);
		result->LinkToActorInstance(actorInstance);
		m_poseDatas.push_back(std::move(newData));
		return result;
	}

	template <typename T>
	T *GetAndPreparePoseData(const ActorInstance *actorInstance) const {
		return const_cast<Pose *>(this)->GetAndPreparePoseData<T>(actorInstance);
	}

	void LinkToActorInstance(ActorInstance *actorInstance);

	void LinkToActorInstance(const ActorInstance *actorInstance);

	void InitFromBindPose(ActorInstance *actorInstance);

	void InitFromBindPose(const ActorInstance *actorInstance);

	void Blend(const Pose *destPose, float weight) {
		if (destPose == nullptr) {
			return;
		}
		const float t = weight < 0.0f ? 0.0f : (weight > 1.0f ? 1.0f : weight);
		const size_t count = m_transforms.size() < destPose->m_transforms.size()
			? m_transforms.size()
			: destPose->m_transforms.size();
		for (size_t i = 0; i < count; ++i) {
			m_transforms[i].m_position = m_transforms[i].m_position.lerp(destPose->m_transforms[i].m_position, t);
			m_transforms[i].m_rotation = m_transforms[i].m_rotation.slerp(destPose->m_transforms[i].m_rotation, t);
		}
	}

	void CompensateForMotionExtraction();

	void DebugDraw(godot::MotionMatching::DebugDrawOutput &debugDisplay,
		const godot::Color &color) const;

	size_t GetNumTransforms() const { return m_transforms.size(); }

	void ResizeTransforms(size_t count) {
		m_transforms.resize(count, Transform::CreateIdentity());
	}

private:

	godot::Skeleton3D *ResolveLiveSkeleton() const;

	Transform GetTransformSafe(size_t jointIndex) const {
		if (jointIndex < m_transforms.size()) {
			return m_transforms[jointIndex];
		}
		return Transform::CreateIdentity();
	}
	void SetTransformSafe(size_t jointIndex, const Transform &transform) {
		if (jointIndex >= m_transforms.size()) {
			m_transforms.resize(jointIndex + 1, Transform::CreateIdentity());
		}
		m_transforms[jointIndex] = transform;
	}

	ActorInstance *m_actorInstance = nullptr;
	godot::Skeleton3D *m_skeleton = nullptr;
	std::vector<Transform> m_transforms;
	std::vector<std::shared_ptr<PoseData>> m_poseDatas;
};