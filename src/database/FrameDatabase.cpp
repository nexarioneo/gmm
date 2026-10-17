#include "FrameDatabase.h"

#include <algorithm>
#include <cassert>
#include <cmath>

#include "ActorInstance.h"
#include "AnimGraphPose.h"
#include "AnimGraphPosePool.h"
#include "../runtime/CsvSerializers.h"
#include "EMotionFXManager.h"
#include "../runtime/EventData.h"
#include "../features/Feature.h"
#include "Frame.h"
#include "Motion.h"
#include "MotionEvent.h"
#include "MotionEventTable.h"
#include "MotionEventTrack.h"
#include "../runtime/MotionMatchingInstance.h"
#include "Pose.h"
#include "TransformData.h"

namespace godot::MotionMatching {

FrameDatabase::FrameDatabase() {
}

FrameDatabase::~FrameDatabase() {
	Clear();
}

void FrameDatabase::Clear() {

	m_frames.clear();
	m_frames.shrink_to_fit();

	m_frameIndexByMotion.clear();

	m_usedMotions.clear();
	m_usedMotions.shrink_to_fit();
}

void FrameDatabase::ExtractActiveMotionEventDatas(const Motion *motion, float time, std::vector<EventData *> &activeEventDatas) const {
	activeEventDatas.clear();

	if (motion == nullptr) {
		return;
	}

	const MotionEventTable *eventTable = motion->GetEventTable();
	if (eventTable == nullptr) {
		return;
	}
	const size_t numTracks = eventTable->GetNumTracks();
	for (size_t t = 0; t < numTracks; ++t) {
		const MotionEventTrack *track = eventTable->GetTrack(t);
		if (track == nullptr) {
			continue;
		}
		const size_t numEvents = track->GetNumEvents();
		for (size_t e = 0; e < numEvents; ++e) {
			const MotionEvent &motionEvent = track->GetEvent(e);

			if (motionEvent.GetIsTickEvent() ||
					motionEvent.GetStartTime() > time ||
					motionEvent.GetEndTime() < time) {
				continue;
			}

			for (auto eventData : motionEvent.GetEventDatas()) {
				activeEventDatas.emplace_back(const_cast<EventData *>(eventData.get()));
			}
		}
	}
}

bool FrameDatabase::IsFrameDiscarded(const Motion *motion, float frameTime, std::vector<EventData *> &activeEvents) const {

	ExtractActiveMotionEventDatas(motion, frameTime, activeEvents);
	for (const EventData *eventData : activeEvents) {
		if (dynamic_cast<const DiscardFrameEventData *>(eventData) != nullptr) {
			return true;
		}
	}

	return false;
}

std::tuple<size_t, size_t> FrameDatabase::ImportFrames(Motion *motion, const FrameImportSettings &settings, bool mirrored) {
	if (motion == nullptr) {
		return { 0, 0 };
	}
	assert(settings.m_sampleRate > 0 && "The sample rate must be bigger than zero frames per second");
	assert(settings.m_sampleRate <= 120 && "The sample rate must be smaller than 120 frames per second");
	if (settings.m_sampleRate == 0 || settings.m_sampleRate > 120) {
		return { 0, 0 };
	}

	size_t numFramesImported = 0;
	size_t numFramesDiscarded = 0;

	m_sampleRate = settings.m_sampleRate;
	const double timeStep = 1.0 / static_cast<double>(settings.m_sampleRate);
	const size_t worstCaseNumFrames = static_cast<size_t>(ceil(motion->GetDuration() / timeStep)) + 1;

	if (m_frames.capacity() < m_frames.size() + worstCaseNumFrames) {
		m_frames.reserve(m_frames.size() + worstCaseNumFrames);
	}

	std::vector<EventData *> activeEvents;

	const double totalTime = static_cast<double>(motion->GetDuration());
	double curTime = 0.0;
	while (curTime <= totalTime) {
		const float frameTime = static_cast<float>(curTime);
		if (!IsFrameDiscarded(motion, frameTime, activeEvents)) {
			ImportFrame(motion, frameTime, mirrored);
			numFramesImported++;
		} else {
			numFramesDiscarded++;
		}
		curTime += timeStep;
	}

	if (curTime - timeStep < totalTime - 0.000001) {
		const float frameTime = static_cast<float>(totalTime);
		if (!IsFrameDiscarded(motion, frameTime, activeEvents)) {
			ImportFrame(motion, frameTime, mirrored);
			numFramesImported++;
		} else {
			numFramesDiscarded++;
		}
	}

	if (settings.m_autoShrink) {
		m_frames.shrink_to_fit();
	}

	if (std::find(m_usedMotions.begin(), m_usedMotions.end(), motion) == m_usedMotions.end()) {
		m_usedMotions.emplace_back(motion);
	}

	return { numFramesImported, numFramesDiscarded };
}

void FrameDatabase::ImportFrame(Motion *motion, float timeValue, bool mirrored) {
	m_frames.emplace_back(Frame(m_frames.size(), motion, timeValue, mirrored));
	m_frameIndexByMotion[motion].emplace_back(m_frames.back().GetFrameIndex());
}

size_t FrameDatabase::CalcMemoryUsageInBytes() const {
	size_t total = 0;

	total += m_frames.capacity() * sizeof(Frame);
	total += sizeof(m_frames);
	total += m_usedMotions.capacity() * sizeof(const Motion *);
	total += sizeof(m_usedMotions);

	return total;
}

size_t FrameDatabase::GetNumFrames() const {
	return m_frames.size();
}

size_t FrameDatabase::GetNumUsedMotions() const {
	return m_usedMotions.size();
}

const Motion *FrameDatabase::GetUsedMotion(size_t index) const {
	assert(index < m_usedMotions.size() && "Motion index is out of range!");
	return m_usedMotions[index];
}

const Frame &FrameDatabase::GetFrame(size_t index) const {
	assert(index < m_frames.size() && "Frame index is out of range!");
	return m_frames[index];
}

std::vector<Frame> &FrameDatabase::GetFrames() {
	return m_frames;
}

const std::vector<Frame> &FrameDatabase::GetFrames() const {
	return m_frames;
}

const std::vector<const Motion *> &FrameDatabase::GetUsedMotions() const {
	return m_usedMotions;
}

size_t FrameDatabase::FindFrameIndex(Motion *motion, float playtime) const {
	auto iterator = m_frameIndexByMotion.find(motion);
	if (iterator == m_frameIndexByMotion.end()) {
		return InvalidIndex;
	}

	const std::vector<size_t> &frameIndices = iterator->second;
	for (const size_t frameIndex : frameIndices) {
		if (frameIndex >= m_frames.size()) {
			continue;
		}
		const Frame &frame = m_frames[frameIndex];
		if (playtime >= frame.GetSampleTime() &&
				frameIndex + 1 < m_frames.size() &&
				playtime <= m_frames[frameIndex + 1].GetSampleTime()) {
			return frameIndex;
		}
	}

	return InvalidIndex;
}

void FrameDatabase::SaveAsCsv(const char *filename, ActorInstance *actorInstance, const ETransformSpace transformSpace, bool writePositions, bool writeRotations) const {
	if (filename == nullptr || actorInstance == nullptr) {
		return;
	}
	PoseWriterCsv poseWriter;
	PoseWriterCsv::WriteSettings writeSettings{ writePositions, writeRotations };
	poseWriter.Begin(filename, actorInstance, writeSettings);

	Pose pose;
	pose.LinkToActorInstance(actorInstance);
	pose.InitFromBindPose(actorInstance);

	for (const Frame &currentFrame : m_frames) {
		currentFrame.SamplePose(&pose);
		poseWriter.WritePose(pose, transformSpace);
	}
	poseWriter.End();
}
}
