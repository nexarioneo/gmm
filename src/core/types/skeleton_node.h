#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>

#include <godot_cpp/classes/skeleton3d.hpp>

class MmNode {
public:
	MmNode() = default;
	~MmNode() = default;

	MmNode(const MmNode &) = delete;
	MmNode &operator=(const MmNode &) = delete;

	MmNode(MmNode &&other) noexcept
		: m_skeleton(other.m_skeleton)
		, m_boneIndex(other.m_boneIndex)
		, m_nameUtf8(std::move(other.m_nameUtf8)) {
		other.m_skeleton = nullptr;
		other.m_boneIndex = -1;
	}
	MmNode &operator=(MmNode &&other) noexcept {
		if (this != &other) {
			m_skeleton = other.m_skeleton;
			m_boneIndex = other.m_boneIndex;
			m_nameUtf8 = std::move(other.m_nameUtf8);
			other.m_skeleton = nullptr;
			other.m_boneIndex = -1;
		}
		return *this;
	}

	void Link(godot::Skeleton3D *skeleton, int boneIndex) {
		m_skeleton = skeleton;
		m_boneIndex = -1;
		m_nameUtf8.clear();
		if (!m_skeleton || boneIndex < 0) {
			m_skeleton = nullptr;
			return;
		}

		const int32_t boneCount = m_skeleton->get_bone_count();
		if (boneIndex >= boneCount) {
			m_skeleton = nullptr;
			return;
		}
		m_boneIndex = boneIndex;

		const godot::String boneName = m_skeleton->get_bone_name(m_boneIndex);
		const auto utf8Chars = boneName.utf8();
		const char *cstr = utf8Chars.get_data();
		if (cstr) {
			m_nameUtf8.assign(cstr);
		}
	}

	void Unlink() {
		m_skeleton = nullptr;
		m_boneIndex = -1;
		m_nameUtf8.clear();
	}

	bool IsLinked() const { return m_skeleton != nullptr && m_boneIndex >= 0; }

	size_t GetNodeIndex() const { return m_boneIndex >= 0 ? static_cast<size_t>(m_boneIndex) : 0; }

	const char *GetName() const { return m_nameUtf8.c_str(); }

private:
	godot::Skeleton3D *m_skeleton = nullptr;
	int m_boneIndex = -1;
	std::string m_nameUtf8;
};