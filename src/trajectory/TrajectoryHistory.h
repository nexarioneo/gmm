#pragma once

#include <cstddef>
#include <vector>

#include <godot_cpp/variant/color.hpp>
#include <godot_cpp/variant/vector3.hpp>

#include "../debug/MotionMatchingCompat.h"
#include "Pose.h"

namespace godot::MotionMatching {

struct Sample {
	Vector3 m_position;
	Vector3 m_facingDirection;
};

Sample operator*(Sample sample, float weight);
Sample operator*(float weight, Sample sample);
Sample operator-(Sample lhs, const Sample &rhs);
Sample operator+(Sample lhs, const Sample &rhs);

class TrajectoryHistory {
public:
	void Init(const Pose &pose, size_t jointIndex, const Vector3 &facingAxisDir, float numSecondsToTrack);
	void Clear();

	void Update(float timeDelta);
	void AddSample(const Pose &pose);

	Sample Evaluate(float time) const;

	Sample EvaluateNormalized(float normalizedTime) const;

	float GetNumSecondsToTrack() const { return m_numSecondsToTrack; }
	float GetCurrentTime() const { return m_currentTime; }
	size_t GetJointIndex() const { return m_jointIndex; }

	void DebugDraw(DebugDrawOutput &output, const Color &color, float timeStart = 0.0f) const;
	void DebugDrawSampled(DebugDrawOutput &output, size_t numSamples, const Color &color) const;

private:
	void PrefillSamples(const Pose &pose, float timeDelta);

	struct Key {
		float m_time = 0.0f;
		Sample m_value;
	};
	void AddKey(float time, const Sample &value);
	void RemoveFirstKey();
	void ClearKeys();
	size_t GetNumKeys() const;
	const Key *GetKey(size_t index) const;
	float GetFirstTime() const;
	float GetLastTime() const;
	size_t FindKeyNumber(float time) const;
	Sample GetValueAtTime(float time) const;

	std::vector<Key> m_keys;
	float m_numSecondsToTrack = 0.0f;
	size_t m_jointIndex = 0;
	float m_currentTime = 0.0f;
	Vector3 m_facingAxisDir;

	static constexpr float m_debugMarkerSize = 0.02f;
};
}
