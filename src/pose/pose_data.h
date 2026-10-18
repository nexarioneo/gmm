#pragma once

#include <memory>
#include <godot_cpp/variant/color.hpp>

#include "debug/compat.h"

class Actor;
class ActorInstance;
class Pose;

class PoseData {
public:
	virtual ~PoseData() = default;

	virtual void Clear() {}

	virtual void LinkToActorInstance(const ActorInstance * ) {}

	virtual void LinkToActor(const Actor * ) {}

	virtual void Reset() {}

	virtual void CopyFrom(const PoseData * ) {}
	virtual std::shared_ptr<PoseData> Clone() const { return nullptr; }

	virtual void Blend(const Pose * , float ) {}

	virtual void DebugDraw(godot::MotionMatching::DebugDrawOutput & ,
		const godot::Color & ) const {
	}

	virtual bool IsUsed() const { return m_isUsed; }
	virtual void SetIsUsed(bool isUsed) { m_isUsed = isUsed; }

	virtual void SetPose(Pose *pose) { m_pose = pose; }
	virtual Pose *GetPose() const { return m_pose; }

protected:
	bool m_isUsed = false;
	Pose *m_pose = nullptr;
};