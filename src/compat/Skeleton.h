#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <godot_cpp/classes/skeleton3d.hpp>

#include "MmNode.h"

class Skeleton {
public:
	Skeleton() = default;
	~Skeleton() = default;

	Skeleton(const Skeleton &) = delete;
	Skeleton &operator=(const Skeleton &) = delete;

	void LinkSkeleton(godot::Skeleton3D *skeleton) { m_skeleton = skeleton; }

	void SetSkeleton(godot::Skeleton3D *skeleton) { LinkSkeleton(skeleton); }

	godot::Skeleton3D *GetLinkedSkeleton() const { return m_skeleton; }

	MmNode *FindNodeByNameNoCase(const char *name) const {
		if (!m_skeleton || !name || !*name) {
			return nullptr;
		}
		const godot::String wanted = godot::String(name).to_lower();
		return FindByLoweredName(wanted);
	}

	MmNode *FindNodeByNameNoCase(const std::string &name) const {
		if (!m_skeleton || name.empty()) {
			return nullptr;
		}
		const godot::String wanted = godot::String(name.c_str()).to_lower();
		return FindByLoweredName(wanted);
	}

	MmNode *FindNodeByName(const char *name) const {
		if (!m_skeleton || !name || !*name) {
			return nullptr;
		}
		return FindByExactName(godot::String(name));
	}

	MmNode *FindNodeByName(const std::string &name) const {
		if (!m_skeleton || name.empty()) {
			return nullptr;
		}
		return FindByExactName(godot::String(name.c_str()));
	}

	MmNode *GetNode(size_t jointIndex) const {
		SyncCache();
		if (jointIndex >= m_nodeCache.size()) {
			return nullptr;
		}
		return &m_nodeCache[jointIndex];
	}

	MmNode *GetNode(size_t jointIndex) {
		return static_cast<const Skeleton *>(this)->GetNode(jointIndex);
	}

	size_t GetNumNodes() const {
		if (!m_skeleton) {
			return 0;
		}
		const int32_t count = m_skeleton->get_bone_count();
		return count > 0 ? static_cast<size_t>(count) : 0;
	}

private:

	void SyncCache() const {
		if (!m_skeleton) {
			if (!m_nodeCache.empty()) {
				m_nodeCache.clear();
			}
			m_cachedSkeleton = nullptr;
			return;
		}
		int32_t count = m_skeleton->get_bone_count();
		if (count < 0) {
			count = 0;
		}
		if (m_cachedSkeleton != m_skeleton || m_nodeCache.size() != static_cast<size_t>(count)) {
			m_nodeCache.clear();
			for (int32_t i = 0; i < count; ++i) {
				m_nodeCache.emplace_back();
				m_nodeCache.back().Link(m_skeleton, i);
			}
			m_cachedSkeleton = m_skeleton;
		}
	}

	MmNode *FindByExactName(const godot::String &name) const {

		const int32_t index = m_skeleton->find_bone(name);
		if (index < 0) {
			return nullptr;
		}
		SyncCache();
		const size_t idx = static_cast<size_t>(index);
		if (idx >= m_nodeCache.size()) {
			return nullptr;
		}
		return &m_nodeCache[idx];
	}

	MmNode *FindByLoweredName(const godot::String &lowered) const {
		SyncCache();
		for (size_t i = 0; i < m_nodeCache.size(); ++i) {

			if (m_skeleton->get_bone_name(static_cast<int32_t>(i)).to_lower() == lowered) {
				return &m_nodeCache[i];
			}
		}
		return nullptr;
	}

	godot::Skeleton3D *m_skeleton = nullptr;
	mutable std::vector<MmNode> m_nodeCache;
	mutable godot::Skeleton3D *m_cachedSkeleton = nullptr;
};
