#pragma once

#include <cstddef>

#include "ActorManager.h"
#include "AnimGraphPosePool.h"

class EMotionFXManager {
public:

	class ThreadData {
	public:
		ThreadData() = default;
		~ThreadData() = default;

		ThreadData(const ThreadData &) = delete;
		ThreadData &operator=(const ThreadData &) = delete;

		AnimGraphPosePool &GetPosePool() { return SharedPosePool(); }
		AnimGraphPosePool &GetPosePool() const { return SharedPosePool(); }

	private:
		static AnimGraphPosePool &SharedPosePool() {
			static AnimGraphPosePool s_posePool;
			return s_posePool;
		}
	};

	EMotionFXManager() = default;
	~EMotionFXManager() = default;

	EMotionFXManager(const EMotionFXManager &) = delete;
	EMotionFXManager &operator=(const EMotionFXManager &) = delete;

	ThreadData *GetThreadData(size_t ) { return &SharedThreadData(); }
	ThreadData *GetThreadData(size_t ) const { return &SharedThreadData(); }

	ActorManager *GetActorManager() { return &SharedActorManager(); }
	ActorManager *GetActorManager() const { return &SharedActorManager(); }

private:
	static ThreadData &SharedThreadData() {
		static ThreadData s_threadData;
		return s_threadData;
	}
	static ActorManager &SharedActorManager() {
		static ActorManager s_actorManager;
		return s_actorManager;
	}
};

inline EMotionFXManager &GetEMotionFX() {
	static EMotionFXManager s_manager;
	return s_manager;
}
