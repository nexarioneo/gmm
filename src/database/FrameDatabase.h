#pragma once

#include <cstddef>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "../runtime/EventData.h"
#include "Frame.h"
#include "../debug/MotionMatchingCompat.h"
#include "TransformSpace.h"

class Motion;
class ActorInstance;

namespace godot::MotionMatching {
class MotionMatchingInstance;
class MotionMatchEventData;

class FrameDatabase {
public:

	struct FrameImportSettings {
		size_t m_sampleRate = 30;
		bool m_autoShrink = true;
	};

	FrameDatabase();
	virtual ~FrameDatabase();

	std::tuple<size_t, size_t> ImportFrames(Motion *motion, const FrameImportSettings &settings, bool mirrored);
	void Clear();

	size_t GetNumFrames() const;
	size_t GetNumUsedMotions() const;
	size_t CalcMemoryUsageInBytes() const;

	const Motion *GetUsedMotion(size_t index) const;
	const Frame &GetFrame(size_t index) const;
	const std::vector<Frame> &GetFrames() const;
	std::vector<Frame> &GetFrames();
	const std::vector<const Motion *> &GetUsedMotions() const;
	size_t GetSampleRate() const { return m_sampleRate; }

	size_t FindFrameIndex(Motion *motion, float playtime) const;

	void SaveAsCsv(const char *filename, ActorInstance *actorInstance, const ETransformSpace transformSpace, bool writePositions, bool writeRotations) const;

private:
	void ImportFrame(Motion *motion, float timeValue, bool mirrored);
	bool IsFrameDiscarded(const Motion *motion, float frameTime, std::vector<EventData *> &activeEvents) const;
	void ExtractActiveMotionEventDatas(const Motion *motion, float time, std::vector<EventData *> &activeEventDatas) const;

private:
	std::vector<Frame> m_frames;
	std::unordered_map<Motion *, std::vector<size_t>> m_frameIndexByMotion;
	std::vector<const Motion *> m_usedMotions;
	size_t m_sampleRate = 0;
};
}
