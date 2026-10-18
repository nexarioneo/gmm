#pragma once

#include "pose/pose.h"

class ActorInstance;

class TransformData {
public:
	TransformData() = default;
	TransformData(const TransformData &) = default;
	TransformData &operator=(const TransformData &) = default;
	~TransformData() = default;

	void LinkActorInstance(ActorInstance *actorInstance) {
		m_actorInstance = actorInstance;
		m_bindPose.LinkToActorInstance(actorInstance);
		m_currentPose.LinkToActorInstance(actorInstance);
	}

	void LinkActorInstance(const ActorInstance *actorInstance) {
		m_actorInstance = const_cast<ActorInstance *>(actorInstance);
		m_bindPose.LinkToActorInstance(actorInstance);
		m_currentPose.LinkToActorInstance(actorInstance);
	}

	Pose *GetBindPose() {
		SyncPoses();
		return &m_bindPose;
	}
	Pose *GetBindPose() const {
		const_cast<TransformData *>(this)->SyncPoses();
		return const_cast<Pose *>(&m_bindPose);
	}

	Pose *GetCurrentPose() {
		SyncPoses();
		return &m_currentPose;
	}
	Pose *GetCurrentPose() const {
		const_cast<TransformData *>(this)->SyncPoses();
		return const_cast<Pose *>(&m_currentPose);
	}

private:

	void SyncPoses() {
		if (!m_actorInstance) {
			return;
		}
		if (m_bindPose.GetActorInstance() != m_actorInstance) {
			m_bindPose.LinkToActorInstance(m_actorInstance);
		}
		if (m_currentPose.GetActorInstance() != m_actorInstance) {
			m_currentPose.LinkToActorInstance(m_actorInstance);
		}
	}

	ActorInstance *m_actorInstance = nullptr;
	Pose m_bindPose;
	Pose m_currentPose;
};