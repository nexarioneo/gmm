#pragma once

#include <cstddef>
#include <vector>

#include "MotionInstance.h"

class MotionSystem {
public:
	MotionSystem() = default;
	~MotionSystem() = default;

	MotionSystem(const MotionSystem &) = delete;
	MotionSystem &operator=(const MotionSystem &) = delete;

	bool GetIsPlaying() const { return m_isPlaying; }

	bool IsPlaying() const { return GetIsPlaying(); }

	void SetIsPlaying(bool isPlaying) { m_isPlaying = isPlaying; }

	MotionInstance *GetMotionInstance(size_t index) const {
		return index < m_motionInstances.size() ? m_motionInstances[index] : nullptr;
	}

	size_t GetNumMotionInstances() const { return m_motionInstances.size(); }

	void AddMotionInstance(MotionInstance *motionInstance) {
		if (!motionInstance) {
			return;
		}
		for (MotionInstance *registered : m_motionInstances) {
			if (registered == motionInstance) {
				return;
			}
		}
		m_motionInstances.push_back(motionInstance);
	}

	void RemoveMotionInstance(MotionInstance *motionInstance) {
		if (!motionInstance) {
			return;
		}
		for (size_t i = 0; i < m_motionInstances.size(); ++i) {
			if (m_motionInstances[i] == motionInstance) {
				m_motionInstances.erase(m_motionInstances.begin() + static_cast<std::ptrdiff_t>(i));
				return;
			}
		}
	}

	void Clear() { m_motionInstances.clear(); }

private:

	std::vector<MotionInstance *> m_motionInstances;
	bool m_isPlaying = false;
};
