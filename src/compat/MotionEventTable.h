#pragma once

#include <cstddef>
#include <memory>
#include <vector>

#include "MotionEventTrack.h"

class MotionEventTable {
public:
	MotionEventTable() = default;
	~MotionEventTable() = default;

	MotionEventTable(const MotionEventTable &) = delete;
	MotionEventTable &operator=(const MotionEventTable &) = delete;

	size_t GetNumTracks() const { return m_tracks.size(); }
	MotionEventTrack *GetTrack(size_t trackIndex) const {
		if (trackIndex >= m_tracks.size() || !m_tracks[trackIndex]) {
			return nullptr;
		}
		return m_tracks[trackIndex].get();
	}
	MotionEventTrack *GetTrack(size_t trackIndex) {
		if (trackIndex >= m_tracks.size() || !m_tracks[trackIndex]) {
			return nullptr;
		}
		return m_tracks[trackIndex].get();
	}

	MotionEventTrack *AddTrack() {
		m_tracks.push_back(std::make_unique<MotionEventTrack>());
		return m_tracks.back().get();
	}
	void Clear() { m_tracks.clear(); }

private:
	std::vector<std::unique_ptr<MotionEventTrack>> m_tracks;
};
