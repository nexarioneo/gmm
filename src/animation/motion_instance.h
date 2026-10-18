#pragma once

#include "animation/motion.h"
#include "animation/motion_event_buffer.h"
#include "core/types/transform.h"

#include <cmath>

#include "actor/actor.h"
#include "pose/pose.h"

class ActorInstance;

class MotionInstance {
public:

	struct PlayState {
		float m_currentTime = 0.0f;
	};

	MotionInstance() = default;
	~MotionInstance() = default;

	MotionInstance(const MotionInstance &) = delete;
	MotionInstance &operator=(const MotionInstance &) = delete;

	Motion *GetMotion() const { return m_motion; }

	float GetCurrentTime() const { return m_currentTime; }

	float GetLastCurrentTime() const { return m_lastCurrentTime; }

	ActorInstance *GetActorInstance() const { return m_actorInstance; }

	void ExtractMotion(Transform &outDelta);

	void Update(float timePassedInSeconds) {
		m_lastCurrentTime = m_currentTime;
		m_currentTime = ComputeTimeAfterUpdate(timePassedInSeconds);
	}

	void UpdateByTimeValues(float preSyncTime, float currentPlayTime) {
		m_lastCurrentTime = preSyncTime;
		m_currentTime = currentPlayTime;
	}

	void UpdateByTimeValues(float preSyncTime, float currentPlayTime, MotionEventBuffer &eventBuffer) {
		m_lastCurrentTime = preSyncTime;
		m_currentTime = currentPlayTime;
		if (m_motion == nullptr) {
			return;
		}
		MotionEventTable *eventTable = m_motion->GetEventTable();
		if (eventTable == nullptr) {
			return;
		}
		for (size_t t = 0; t < eventTable->GetNumTracks(); ++t) {
			MotionEventTrack *track = eventTable->GetTrack(t);
			if (track == nullptr) {
				continue;
			}
			for (size_t e = 0; e < track->GetNumEvents(); ++e) {
				const MotionEvent &event = track->GetEvent(e);
				const float start = event.GetStartTime();
				if (start > preSyncTime && start <= currentPlayTime) {
					for (const auto &data : event.GetEventDatas()) {
						eventBuffer.AddFired(start, data);
					}
				} else if (!event.GetIsTickEvent() && event.GetStartTime() <= currentPlayTime && event.GetEndTime() >= preSyncTime) {
					for (const auto &data : event.GetEventDatas()) {
						eventBuffer.AddFired(start, data);
					}
				}
			}
		}
	}

	void SetMotion(Motion *motion) { m_motion = motion; }
	void SetActorInstance(ActorInstance *actorInstance) { m_actorInstance = actorInstance; }

	void SetMirrorMotion(bool mirror) { m_mirror = mirror; }
	bool GetMirrorMotion() const { return m_mirror; }

	void SetCurrentTime(float currentTime) { m_currentTime = currentTime; }

	void SetLastCurrentTime(float lastCurrentTime) { m_lastCurrentTime = lastCurrentTime; }

	void SetFreezeAtLastFrame(bool freeze) { m_freezeAtLastFrame = freeze; }

	void SetMaxLoops(int maxLoops) { m_maxLoops = maxLoops; }

	PlayState CalcPlayStateAfterUpdate(float deltaTime) const {
		PlayState state;
		state.m_currentTime = ComputeTimeAfterUpdate(deltaTime);
		return state;
	}

private:

	float ComputeTimeAfterUpdate(float deltaTime) const {
		float result = m_currentTime + deltaTime;
		if (result < 0.0f) {
			result = 0.0f;
		}
		const float duration = m_motion != nullptr ? m_motion->GetDuration() : 0.0f;
		if (duration <= 0.0f) {
			return result;
		}
		if (m_freezeAtLastFrame) {
			if (result > duration) {
				result = duration;
			}
			return result;
		}
		while (result >= duration) {
			result = std::fmod(result, duration);
		}
		return result;
	}

	Motion *m_motion = nullptr;
	ActorInstance *m_actorInstance = nullptr;
	float m_currentTime = 0.0f;
	float m_lastCurrentTime = 0.0f;
	bool m_mirror = false;
	bool m_freezeAtLastFrame = false;
	int m_maxLoops = 0;
};