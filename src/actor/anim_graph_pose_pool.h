#pragma once

#include <cstddef>
#include <memory>
#include <vector>

#include "actor/actor_instance.h"
#include "actor/anim_graph_pose.h"

class AnimGraphPosePool {
public:
	AnimGraphPosePool() = default;
	~AnimGraphPosePool() = default;

	AnimGraphPosePool(const AnimGraphPosePool &) = delete;
	AnimGraphPosePool &operator=(const AnimGraphPosePool &) = delete;

	AnimGraphPose *RequestPose(const ActorInstance *actorInstance) {
		AnimGraphPose *pose = nullptr;
		if (!m_freeList.empty()) {
			pose = m_freeList.back();
			m_freeList.pop_back();
		} else {
			m_pool.emplace_back(new AnimGraphPose());
			pose = m_pool.back().get();
		}
		if (pose != nullptr) {
			pose->GetPose().LinkToActorInstance(actorInstance);
		}
		return pose;
	}

	void FreePose(AnimGraphPose *pose) {
		if (pose == nullptr) {
			return;
		}
		for (size_t i = 0; i < m_freeList.size(); ++i) {
			if (m_freeList[i] == pose) {
				return;
			}
		}
		m_freeList.push_back(pose);
	}

private:
	std::vector<std::unique_ptr<AnimGraphPose>> m_pool;
	std::vector<AnimGraphPose *> m_freeList;
};