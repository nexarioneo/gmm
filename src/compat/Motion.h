#pragma once

#include "MotionData.h"
#include "MotionEventTable.h"

namespace godot {
class Animation;
}

class Pose;
class MotionInstance;

class Motion {
public:
	Motion() = default;
	~Motion() = default;

	Motion(const Motion &) = delete;
	Motion &operator=(const Motion &) = delete;

	float GetDuration() const;

	void SamplePose(Pose *outputPose, MotionDataSampleSettings &settings) const;
	void SamplePose(Pose *outputPose, const MotionDataSampleSettings &settings) const;

	void Update(const Pose *inputPose, Pose *outputPose, MotionInstance *motionInstance) const;

	MotionEventTable *GetEventTable() {
		RebuildEventTable();
		return &m_eventTable;
	}
	MotionEventTable *GetEventTable() const {
		RebuildEventTable();
		return const_cast<MotionEventTable *>(&m_eventTable);
	}

	void LinkAnimation(godot::Animation *animation);

	void SetAnimation(godot::Animation *animation) { LinkAnimation(animation); }

	godot::Animation *GetAnimation() const { return m_animation; }

	godot::Animation *GetLinkedAnimation() const { return m_animation; }

private:

	void SamplePoseInternal(Pose *outputPose, const MotionDataSampleSettings &settings) const;

	void RebuildEventTable() const;

	godot::Animation *m_animation = nullptr;
	mutable MotionEventTable m_eventTable;
	mutable bool m_eventTableDirty = true;
};
