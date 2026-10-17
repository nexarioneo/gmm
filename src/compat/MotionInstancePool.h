#pragma once

#include <cstddef>
#include <memory>
#include <vector>

#include "MotionInstance.h"

class Motion;
class ActorInstance;

class MotionInstancePool {
public:
	MotionInstancePool() = default;
	~MotionInstancePool() = default;

	MotionInstancePool(const MotionInstancePool &) = delete;
	MotionInstancePool &operator=(const MotionInstancePool &) = delete;

	MotionInstance *RequestNew(Motion *motion, ActorInstance *actorInstance) {
		MotionInstance *instance = nullptr;
		if (!m_freeList.empty()) {
			instance = m_freeList.back();
			m_freeList.pop_back();
		} else {
			m_pool.emplace_back(new MotionInstance());
			instance = m_pool.back().get();
		}
		if (instance) {
			instance->SetMotion(motion);
			instance->SetActorInstance(actorInstance);
			instance->SetMirrorMotion(false);
			instance->SetCurrentTime(0.0f);
			instance->SetLastCurrentTime(0.0f);
			instance->SetFreezeAtLastFrame(false);
			instance->SetMaxLoops(0);
		}
		return instance;
	}

	void Free(MotionInstance *motionInstance) {
		if (motionInstance == nullptr) {
			return;
		}
		for (size_t i = 0; i < m_freeList.size(); ++i) {
			if (m_freeList[i] == motionInstance) {
				return;
			}
		}
		bool owned = false;
		for (size_t i = 0; i < m_pool.size(); ++i) {
			if (m_pool[i].get() == motionInstance) {
				owned = true;
				break;
			}
		}
		if (!owned) {
			return;
		}
		m_freeList.push_back(motionInstance);
	}

private:
	std::vector<std::unique_ptr<MotionInstance>> m_pool;
	std::vector<MotionInstance *> m_freeList;
};

inline MotionInstancePool &GetMotionInstancePool() {
	static MotionInstancePool s_pool;
	return s_pool;
}
