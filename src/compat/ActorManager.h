#pragma once

#include <cstddef>
#include <vector>

#include "ActorInstance.h"

class ActorManager {
public:
	ActorManager() = default;
	~ActorManager() = default;

	ActorManager(const ActorManager &) = delete;
	ActorManager &operator=(const ActorManager &) = delete;

	size_t GetNumActorInstances() const { return m_actorInstances.size(); }

	ActorInstance *GetActorInstance(size_t index) const {
		return index < m_actorInstances.size() ? m_actorInstances[index] : nullptr;
	}

	void RegisterActorInstance(ActorInstance *actorInstance) {
		if (!actorInstance) {
			return;
		}
		for (ActorInstance *registered : m_actorInstances) {
			if (registered == actorInstance) {
				return;
			}
		}
		m_actorInstances.push_back(actorInstance);
	}

	void UnregisterActorInstance(ActorInstance *actorInstance) {
		if (!actorInstance) {
			return;
		}
		for (size_t i = 0; i < m_actorInstances.size(); ++i) {
			if (m_actorInstances[i] == actorInstance) {
				m_actorInstances.erase(m_actorInstances.begin() + static_cast<std::ptrdiff_t>(i));
				return;
			}
		}
	}

	void Clear() { m_actorInstances.clear(); }

private:

	std::vector<ActorInstance *> m_actorInstances;
};
