#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include <godot_cpp/classes/skeleton3d.hpp>

#include "Skeleton.h"

class MmNode;

class Actor {
public:
	Actor() = default;
	~Actor() = default;

	Actor(const Actor &) = delete;
	Actor &operator=(const Actor &) = delete;

	Skeleton *GetSkeleton() { return &m_skeleton; }
	Skeleton *GetSkeleton() const { return const_cast<Skeleton *>(&m_skeleton); }

	void LinkActor(godot::Skeleton3D *skeleton) {
		m_skeleton.LinkSkeleton(skeleton);
		RefreshMotionExtractionNode();
	}

	void SetSkeleton(godot::Skeleton3D *skeleton) { LinkActor(skeleton); }

	size_t GetMotionExtractionNodeIndex() const { return m_motionExtractionNodeIndex; }

	void SetMotionExtractionNodeIndex(size_t nodeIndex) {
		m_motionExtractionNodeIndex = nodeIndex;
		RefreshMotionExtractionNode();
	}

	void SetMotionExtractionNodeByName(const char *name) {
		godot::Skeleton3D *skeleton = m_skeleton.GetLinkedSkeleton();
		if (skeleton && name && *name) {

			const int32_t found = skeleton->find_bone(godot::String(name));
			m_motionExtractionNodeIndex = found < 0 ? kInvalidNodeIndex : static_cast<size_t>(found);
		} else {
			m_motionExtractionNodeIndex = kInvalidNodeIndex;
		}
		RefreshMotionExtractionNode();
	}
	void SetMotionExtractionNodeByName(const std::string &name) {
		SetMotionExtractionNodeByName(name.c_str());
	}

	MmNode *GetMotionExtractionNode() {
		RefreshMotionExtractionNode();
		return m_motionExtractionNode.IsLinked() ? &m_motionExtractionNode : nullptr;
	}
	MmNode *GetMotionExtractionNode() const {
		RefreshMotionExtractionNode();
		return m_motionExtractionNode.IsLinked() ? const_cast<MmNode *>(&m_motionExtractionNode) : nullptr;
	}

	bool GetMotionExtractionEnabled() const { return m_motionExtractionEnabled; }
	void SetMotionExtractionEnabled(bool enabled) { m_motionExtractionEnabled = enabled; }

private:

	void RefreshMotionExtractionNode() const {
		godot::Skeleton3D *skeleton = m_skeleton.GetLinkedSkeleton();
		if (skeleton && m_motionExtractionNodeIndex != kInvalidNodeIndex) {

			const int32_t boneCount = skeleton->get_bone_count();
			if (boneCount > 0 && m_motionExtractionNodeIndex < static_cast<size_t>(boneCount)) {
				m_motionExtractionNode.Link(skeleton, static_cast<int>(m_motionExtractionNodeIndex));
				return;
			}
		}
		m_motionExtractionNode.Unlink();
	}

	static constexpr size_t kInvalidNodeIndex = static_cast<size_t>(-1);

	Skeleton m_skeleton;
	size_t m_motionExtractionNodeIndex = kInvalidNodeIndex;
	mutable MmNode m_motionExtractionNode;
	bool m_motionExtractionEnabled = false;
};
