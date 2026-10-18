#pragma once

#include <cstddef>

#include "debug/compat.h"
#include "pose/pose.h"

class Motion;

namespace godot::MotionMatching {

	class Frame {
	public:
		Frame();
		Frame(size_t frameIndex, Motion *sourceMotion, float sampleTime, bool mirrored);
		~Frame() = default;

		void SamplePose(Pose *outputPose, float timeOffset = 0.0f) const;

		Motion *GetSourceMotion() const;
		float GetSampleTime() const;
		size_t GetFrameIndex() const { return m_frameIndex; }
		bool GetMirrored() const { return m_mirrored; }

		void SetSourceMotion(Motion *sourceMotion);
		void SetSampleTime(float sampleTime);
		void SetFrameIndex(size_t frameIndex);
		void SetMirrored(bool enabled);

	private:
		size_t m_frameIndex = 0;
		float m_sampleTime = 0.0f;
		Motion *m_sourceMotion = nullptr;
		bool m_mirrored = false;
	};
}
