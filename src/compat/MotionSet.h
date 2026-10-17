#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include "Motion.h"

namespace godot {
class Animation;
}

class MotionSet {
public:
	MotionSet() = default;
	~MotionSet() = default;

	MotionSet(const MotionSet &) = delete;
	MotionSet &operator=(const MotionSet &) = delete;

	size_t GetNumMotions() const { return m_motions.size(); }

	bool IsEmpty() const { return m_motions.empty(); }

	Motion *GetMotion(size_t index) const {
		if (index >= m_motions.size()) {
			return nullptr;
		}
		return m_motions[index].get();
	}

	Motion *RecursiveFindMotionById(uint32_t motionId) const {
		return GetMotion(static_cast<size_t>(motionId));
	}

	Motion *FindMotionByAnimation(const godot::Animation *animation) const {
		if (animation == nullptr) {
			return nullptr;
		}
		for (const auto &motion : m_motions) {
			if (motion && motion->GetAnimation() == animation) {
				return motion.get();
			}
		}
		return nullptr;
	}

	Motion *AddMotion(godot::Animation *animation) {
		if (animation == nullptr) {
			return nullptr;
		}
		if (Motion *existing = FindMotionByAnimation(animation)) {
			return existing;
		}
		auto motion = std::make_unique<Motion>();
		motion->LinkAnimation(animation);
		Motion *result = motion.get();
		m_motions.push_back(std::move(motion));
		return result;
	}

	void Clear() { m_motions.clear(); }

private:
	std::vector<std::unique_ptr<Motion>> m_motions;
};
