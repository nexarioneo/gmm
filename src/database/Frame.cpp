#include "Frame.h"

#include <algorithm>

#include "ActorInstance.h"
#include "Motion.h"
#include "MotionData.h"
#include "Pose.h"
#include "TransformData.h"

namespace godot::MotionMatching {

Frame::Frame()
	: m_frameIndex(InvalidIndex)
	, m_sampleTime(0.0f)
	, m_sourceMotion(nullptr)
	, m_mirrored(false) {
}

Frame::Frame(size_t frameIndex, Motion *sourceMotion, float sampleTime, bool mirrored)
	: m_frameIndex(frameIndex)
	, m_sourceMotion(sourceMotion)
	, m_sampleTime(sampleTime)
	, m_mirrored(mirrored) {
}

void Frame::SamplePose(Pose *outputPose, float timeOffset) const {
	if (outputPose == nullptr || m_sourceMotion == nullptr) {
		return;
	}
	MotionDataSampleSettings sampleSettings;
	sampleSettings.m_actorInstance = outputPose->GetActorInstance();
	sampleSettings.m_inPlace = false;
	sampleSettings.m_mirror = m_mirrored;
	sampleSettings.m_retarget = false;
	ActorInstance *actorInstance = const_cast<ActorInstance *>(sampleSettings.m_actorInstance);
	sampleSettings.m_inputPose = (actorInstance != nullptr && actorInstance->GetTransformData() != nullptr)
		? actorInstance->GetTransformData()->GetBindPose()
		: nullptr;

	sampleSettings.m_sampleTime = std::clamp(m_sampleTime + timeOffset, 0.0f, m_sourceMotion->GetDuration());

	m_sourceMotion->SamplePose(outputPose, sampleSettings);
}

void Frame::SetFrameIndex(size_t frameIndex) {
	m_frameIndex = frameIndex;
}

Motion *Frame::GetSourceMotion() const {
	return m_sourceMotion;
}

float Frame::GetSampleTime() const {
	return m_sampleTime;
}

void Frame::SetSourceMotion(Motion *sourceMotion) {
	m_sourceMotion = sourceMotion;
}

void Frame::SetMirrored(bool enabled) {
	m_mirrored = enabled;
}

void Frame::SetSampleTime(float sampleTime) {
	m_sampleTime = sampleTime;
}

}
