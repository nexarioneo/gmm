#pragma once

#include <cstddef>

#include "Actor.h"
#include "MotionSystem.h"
#include "Transform.h"
#include "TransformData.h"

namespace godot {
class Node3D;
class Skeleton3D;
}

class ActorInstance {
public:
	ActorInstance() = default;
	~ActorInstance() = default;

	ActorInstance(const ActorInstance &) = delete;
	ActorInstance &operator=(const ActorInstance &) = delete;

	Actor *GetActor() { return &m_actor; }
	Actor *GetActor() const { return const_cast<Actor *>(&m_actor); }

	TransformData *GetTransformData() { return &m_transformData; }
	TransformData *GetTransformData() const { return const_cast<TransformData *>(&m_transformData); }

	size_t GetNumNodes() const {
		const Skeleton *skeleton = m_actor.GetSkeleton();
		return skeleton ? skeleton->GetNumNodes() : 0;
	}

	size_t GetNumEnabledNodes() const { return GetNumNodes(); }

	size_t GetEnabledNode(size_t index) const {
		return index < GetNumEnabledNodes() ? index : 0;
	}

	Transform GetWorldSpaceTransform() const;

	MotionSystem *GetMotionSystem() { return &m_motionSystem; }
	MotionSystem *GetMotionSystem() const { return const_cast<MotionSystem *>(&m_motionSystem); }

	size_t GetThreadIndex() const { return 0; }

	void LinkActor(godot::Skeleton3D *skeleton, godot::Node3D *root = nullptr);

	void LinkRoot(godot::Node3D *root) { m_root = root; }

	void SetRoot(godot::Node3D *root) { LinkRoot(root); }

	godot::Node3D *GetRoot() const { return m_root; }

private:
	Actor m_actor;
	TransformData m_transformData;
	MotionSystem m_motionSystem;
	godot::Node3D *m_root = nullptr;
};
