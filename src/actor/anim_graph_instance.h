#pragma once

#include "actor/actor_instance.h"
#include "animation/motion_set.h"

class AnimGraphInstance {
public:
	AnimGraphInstance() = default;
	~AnimGraphInstance() = default;

	AnimGraphInstance(const AnimGraphInstance &) = delete;
	AnimGraphInstance &operator=(const AnimGraphInstance &) = delete;

	MotionSet *GetMotionSet() { return &m_motionSet; }
	MotionSet *GetMotionSet() const { return const_cast<MotionSet *>(&m_motionSet); }

	ActorInstance *GetActorInstance() { return &m_actorInstance; }
	ActorInstance *GetActorInstance() const { return const_cast<ActorInstance *>(&m_actorInstance); }

	bool IsInitialized() const { return false; }

private:
	MotionSet m_motionSet;
	ActorInstance m_actorInstance;
};